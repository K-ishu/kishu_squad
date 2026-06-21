import os
import sqlite3
import yaml
import matplotlib.pyplot as plt

from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

BAG_DIR = os.path.expanduser("~/robotics_hw3_workspace/bags/hw3_force_land_manual_live_v2")
OUT_DIR = os.path.expanduser("~/robotics_hw3_workspace/plots")
os.makedirs(OUT_DIR, exist_ok=True)

db_file = os.path.join(BAG_DIR, "hw3_force_land_manual_live_v2_0.db3")
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
t = []
roll = []
pitch = []
yaw = []
throttle = []

for topic_id, timestamp, raw in cursor.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"):
    topic_name, type_name = topics[topic_id]

    if topic_name != "/fmu/in/manual_control_input":
        continue

    msg_type = get_message(type_name)
    msg = deserialize_message(raw, msg_type)

    if t0 is None:
        t0 = timestamp

    t.append((timestamp - t0) * 1e-9)
    roll.append(float(msg.roll))
    pitch.append(float(msg.pitch))
    yaw.append(float(msg.yaw))
    throttle.append(float(msg.throttle))

conn.close()

plt.figure()
plt.plot(t, roll, label="roll")
plt.plot(t, pitch, label="pitch")
plt.plot(t, yaw, label="yaw")
plt.plot(t, throttle, label="throttle")
plt.xlabel("Time [s]")
plt.ylabel("Manual control input")
plt.title("Manual Control Input")
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(OUT_DIR, "force_land_manual_control.png"), dpi=300, bbox_inches="tight")
plt.close()

print("Saved:", os.path.join(OUT_DIR, "force_land_manual_control.png"))
print("Manual input samples:", len(t))
