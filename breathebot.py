import time
import struct
import serial
import math

import collections
import collections.abc

# Python 3.10+ compatibility for DroneKit
if not hasattr(collections, "MutableMapping"):
    collections.MutableMapping = collections.abc.MutableMapping
    collections.MutableSequence = collections.abc.MutableSequence
    collections.MutableSet = collections.abc.MutableSet

from dronekit import connect, VehicleMode, LocationGlobalRelative

import firebase_admin
from firebase_admin import credentials, db

# -------------------- FIREBASE INIT --------------------
cred = credentials.Certificate("/home/drone/firebase_key.json")
firebase_admin.initialize_app(cred, {
    "databaseURL": "https://aqidrone-8050d-default-rtdb.firebaseio.com"
})

mission_ref = db.reference("mission")
readings_ref = db.reference("readings")

# -------------------- DRONE CONNECT --------------------
vehicle = connect('/dev/ttyACM0', baud=57600, wait_ready=True)

# -------------------- PMS7003 SETUP --------------------
pm_serial = serial.Serial('/dev/ttyS0', 9600, timeout=2)

# -------------------- PMS7003 READ --------------------
def read_pm25(timeout=5):
    start = time.time()

    while time.time() - start < timeout:
        if pm_serial.read(1) != b'\x42':
            continue
        if pm_serial.read(1) != b'\x4d':
            continue

        frame = pm_serial.read(30)
        if len(frame) != 30:
            continue

        frame_len = struct.unpack(">H", frame[0:2])[0]
        if frame_len != 28:
            continue

        data = struct.unpack(">13H", frame[2:28])
        checksum_rx = struct.unpack(">H", frame[28:30])[0]
        checksum_calc = sum(b'\x42\x4d' + frame[:28])

        if checksum_calc != checksum_rx:
            continue

        return data[3]  # PM2.5 atmospheric

    raise RuntimeError("PM2.5 sensor timeout")

# -------------------- PM2.5 AVERAGING --------------------
def read_pm25_avg(samples=3):
    values = []
    for _ in range(samples):
        values.append(read_pm25())
        time.sleep(1)
    return sum(values) / len(values)

# -------------------- AQI CALCULATION --------------------
def calculate_aqi(pm25):
    breakpoints = [
        (0.0, 12.0, 0, 50),
        (12.1, 35.4, 51, 100),
        (35.5, 55.4, 101, 150),
        (55.5, 150.4, 151, 200),
        (150.5, 250.4, 201, 300),
        (250.5, 500.4, 301, 500)
    ]

    for c_low, c_high, aqi_low, aqi_high in breakpoints:
        if c_low <= pm25 <= c_high:
            return int(
                ((aqi_high - aqi_low) / (c_high - c_low)) *
                (pm25 - c_low) + aqi_low
            )
    return 500

# -------------------- ARM & TAKEOFF --------------------
def arm_and_takeoff(target_altitude):
    while vehicle.gps_0.fix_type < 3:
        time.sleep(1)

    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(1)

    vehicle.simple_takeoff(target_altitude)

    while True:
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            break
        time.sleep(1)

# -------------------- DISTANCE FUNCTION --------------------
def get_distance_meters(loc1, loc2):
    dlat = loc2.lat - loc1.lat
    dlon = loc2.lon - loc1.lon
    return math.sqrt(dlat * dlat + dlon * dlon) * 1.113195e5

# -------------------- VISIT LOCATION --------------------
def visit_point(lat, lon, index, timeout=60):
    print(f"Going to point {index}")
    target = LocationGlobalRelative(lat, lon, 10)
    vehicle.simple_goto(target)

    start = time.time()
    while time.time() - start < timeout:
        current = vehicle.location.global_relative_frame
        if get_distance_meters(current, target) < 1.5:
            break

    pm25 = read_pm25_avg()
    aqi = calculate_aqi(pm25)

    readings_ref.child(f"point_{index}").set({
        "pm25": round(pm25, 2),
        "aqi": aqi,
        "lat": current.lat,
        "lon": current.lon,
        "alt": round(current.alt, 2),
        "timestamp": int(time.time())
    })

# -------------------- MAIN LOOP --------------------
print("Waiting for mission start...")

while True:
    try:
        mission = mission_ref.get()
    except Exception:
        time.sleep(2)
        continue

    if mission and mission.get("start") is True and "locations" in mission:
        mission_ref.update({
            "start": False,
            "confirmed": True,
            "in_progress": True
        })

        try:
            arm_and_takeoff(10)

            for i, loc in enumerate(mission["locations"]):
                visit_point(loc["lat"], loc["lon"], i)

            vehicle.mode = VehicleMode("RTL")

        except Exception as e:
            print("Mission aborted:", e)
            vehicle.mode = VehicleMode("RTL")

        finally:
            mission_ref.update({"in_progress": False})
            vehicle.close()
            break

    time.sleep(2)
