import os
import sqlite3
import yaml
import matplotlib.pyplot as plt

from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

BAG_DIR = os.path.expanduser("~/robotics_hw3_workspace/bags/hw3_force_land_clean_v2")
OUT_DIR = os.path.expanduser("~/robotics_hw3_workspace/plots")
os.makedirs(OUT_DIR, exist_ok=True)

db_file = os.path.join(BAG_DIR, "hw3_force_land_clean_v2_0.db3")
metadata_file = os.path.join(BAG_DIR, "metadata.yaml")

with open(metadata_file, "r") as f:
    metadata = yaml.safe_load(f)

conn = sqlite3.connect(db_file)
cursor = conn.cursor()

topics = {}
for row in cursor.execute("SELECT * FROM topics"):
    topic_id, name, type_name, serialization_format, offered_qos_profiles = row
    topics[topic_id] = (name, type_name)

t0 = None

alt_t = []
altitude = []

land_t = []
landed = []

manual_t = []
roll = []
pitch = []
yaw = []
throttle = []

for topic_id, timestamp, raw in cursor.execute(
    "SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"
):
    topic_name, type_name = topics[topic_id]

    msg_type = get_message(type_name)
    msg = deserialize_message(raw, msg_type)

    if t0 is None:
        t0 = timestamp

    t = (timestamp - t0) * 1e-9

    if topic_name == "/fmu/out/vehicle_local_position_v1":
        alt_t.append(t)
        altitude.append(float(-msg.z))

    elif topic_name == "/fmu/out/vehicle_land_detected":
        land_t.append(t)
        landed.append(1.0 if msg.landed else 0.0)

    elif topic_name == "/fmu/in/manual_control_input":
        manual_t.append(t)
        roll.append(float(msg.roll))
        pitch.append(float(msg.pitch))
        yaw.append(float(msg.yaw))
        throttle.append(float(msg.throttle))

conn.close()

# 1) Force-land altitude + landing detected
plt.figure()
plt.plot(alt_t, altitude, label="Altitude ENU")
if land_t:
    plt.step(land_t, [v * max(altitude) for v in landed], where="post", label="Landed flag scaled")
plt.axhline(20.0, linestyle="--", label="20 m threshold")
plt.xlabel("Time [s]")
plt.ylabel("Altitude ENU [m]")
plt.title("Force Land Altitude and Landing Detection")
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(OUT_DIR, "force_land_altitude_land_detected.png"), dpi=300, bbox_inches="tight")
plt.close()

# 2) Manual input signals
plt.figure()
plt.plot(manual_t, roll, label="roll")
plt.plot(manual_t, pitch, label="pitch")
plt.plot(manual_t, yaw, label="yaw")
plt.plot(manual_t, throttle, label="throttle")
plt.xlabel("Time [s]")
plt.ylabel("Manual control input")
plt.title("Manual Control Input")
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(OUT_DIR, "force_land_manual_control.png"), dpi=300, bbox_inches="tight")
plt.close()

# 3) Altitude + manual throttle
plt.figure()
plt.plot(alt_t, altitude, label="Altitude ENU")
if manual_t:
    plt.plot(manual_t, [v * max(altitude) for v in throttle], label="Throttle scaled")
plt.axhline(20.0, linestyle="--", label="20 m threshold")
plt.xlabel("Time [s]")
plt.ylabel("Altitude [m] / scaled throttle")
plt.title("Force Land Manual Input vs Altitude")
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(OUT_DIR, "force_land_manual_vs_altitude.png"), dpi=300, bbox_inches="tight")
plt.close()

print("Saved:")
print(os.path.join(OUT_DIR, "force_land_altitude_land_detected.png"))
print(os.path.join(OUT_DIR, "force_land_manual_control.png"))
print(os.path.join(OUT_DIR, "force_land_manual_vs_altitude.png"))
print("Samples:")
print("altitude:", len(alt_t))
print("land_detected:", len(land_t))
print("manual:", len(manual_t))
