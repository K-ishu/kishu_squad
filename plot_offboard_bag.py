import os
import math
import sqlite3
import yaml
import matplotlib.pyplot as plt

from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

BAG_DIR = os.path.expanduser("~/robotics_hw3_workspace/bags/hw3_offboard_landing_final")
OUT_DIR = os.path.expanduser("~/robotics_hw3_workspace/plots")
os.makedirs(OUT_DIR, exist_ok=True)

db_file = os.path.join(BAG_DIR, "hw3_offboard_landing_final_0.db3")
metadata_file = os.path.join(BAG_DIR, "metadata.yaml")

with open(metadata_file, "r") as f:
    metadata = yaml.safe_load(f)

topic_types = {}
for topic in metadata["rosbag2_bagfile_information"]["topics_with_message_count"]:
    topic_types[topic["topic_metadata"]["name"]] = topic["topic_metadata"]["type"]

conn = sqlite3.connect(db_file)
cursor = conn.cursor()

topics = {}
for topic_id, name, type_name, serialization_format, offered_qos_profiles in cursor.execute("SELECT * FROM topics"):
    topics[topic_id] = (name, type_name)

data = {
    "t": [],
    "x": [],
    "y": [],
    "z_enu": [],
    "vx": [],
    "vy": [],
    "vz": [],
    "ax": [],
    "ay": [],
    "az": [],
    "yaw": [],
}

start_time = None

for topic_id, timestamp, raw in cursor.execute("SELECT topic_id, timestamp, data FROM messages ORDER BY timestamp"):
    topic_name, type_name = topics[topic_id]

    if topic_name not in [
        "/fmu/out/vehicle_local_position_v1",
        "/fmu/out/vehicle_attitude",
    ]:
        continue

    msg_type = get_message(type_name)
    msg = deserialize_message(raw, msg_type)

    if start_time is None:
        start_time = timestamp

    t = (timestamp - start_time) * 1e-9

    if topic_name == "/fmu/out/vehicle_local_position_v1":
        data["t"].append(t)
        data["x"].append(float(msg.x))
        data["y"].append(float(msg.y))
        data["z_enu"].append(float(-msg.z))
        data["vx"].append(float(msg.vx))
        data["vy"].append(float(msg.vy))
        data["vz"].append(float(-msg.vz))
        data["ax"].append(float(msg.ax))
        data["ay"].append(float(msg.ay))
        data["az"].append(float(-msg.az))

    elif topic_name == "/fmu/out/vehicle_attitude":
        q = msg.q
        qw, qx, qy, qz = q[0], q[1], q[2], q[3]
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz)
        )
        data["yaw"].append((t, yaw))

conn.close()

# XY trajectory
plt.figure()
plt.plot(data["x"], data["y"])
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Offboard XY Trajectory")
plt.grid(True)
plt.axis("equal")
plt.savefig(os.path.join(OUT_DIR, "offboard_xy_trajectory.png"), dpi=300, bbox_inches="tight")
plt.close()

# Altitude
plt.figure()
plt.plot(data["t"], data["z_enu"])
plt.xlabel("Time [s]")
plt.ylabel("Altitude ENU [m]")
plt.title("Offboard Altitude")
plt.grid(True)
plt.savefig(os.path.join(OUT_DIR, "offboard_altitude.png"), dpi=300, bbox_inches="tight")
plt.close()

# Velocity
vel_norm = [
    math.sqrt(vx*vx + vy*vy + vz*vz)
    for vx, vy, vz in zip(data["vx"], data["vy"], data["vz"])
]
plt.figure()
plt.plot(data["t"], data["vx"], label="vx")
plt.plot(data["t"], data["vy"], label="vy")
plt.plot(data["t"], data["vz"], label="vz ENU")
plt.plot(data["t"], vel_norm, label="|v|")
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")
plt.title("Offboard Velocity")
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(OUT_DIR, "offboard_velocity.png"), dpi=300, bbox_inches="tight")
plt.close()

# Acceleration
acc_norm = [
    math.sqrt(ax*ax + ay*ay + az*az)
    for ax, ay, az in zip(data["ax"], data["ay"], data["az"])
]
plt.figure()
plt.plot(data["t"], data["ax"], label="ax")
plt.plot(data["t"], data["ay"], label="ay")
plt.plot(data["t"], data["az"], label="az ENU")
plt.plot(data["t"], acc_norm, label="|a|")
plt.xlabel("Time [s]")
plt.ylabel("Acceleration [m/s²]")
plt.title("Offboard Acceleration")
plt.legend()
plt.grid(True)
plt.savefig(os.path.join(OUT_DIR, "offboard_acceleration.png"), dpi=300, bbox_inches="tight")
plt.close()

# Yaw
yaw_t = [p[0] for p in data["yaw"]]
yaw_val = [p[1] for p in data["yaw"]]
plt.figure()
plt.plot(yaw_t, yaw_val)
plt.xlabel("Time [s]")
plt.ylabel("Yaw [rad]")
plt.title("Offboard Yaw")
plt.grid(True)
plt.savefig(os.path.join(OUT_DIR, "offboard_yaw.png"), dpi=300, bbox_inches="tight")
plt.close()

print("Saved plots in:", OUT_DIR)
