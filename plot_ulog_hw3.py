import os
import numpy as np
import matplotlib.pyplot as plt
from pyulog import ULog

ULOG = os.path.expanduser(
    "~/robotics_hw3_workspace/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-06-19/08_42_46.ulg"
)

OUT_DIR = os.path.expanduser("~/robotics_hw3_workspace/plots")
os.makedirs(OUT_DIR, exist_ok=True)

ulog = ULog(ULOG)

print("Available datasets:")
for d in ulog.data_list:
    print(d.name)

def get_dataset(name_candidates):
    for name in name_candidates:
        for d in ulog.data_list:
            if d.name == name:
                return d
    return None

# Actuator outputs
act = get_dataset(["actuator_outputs", "actuator_motors", "actuator_servos"])
if act is not None:
    data = act.data
    t = (data["timestamp"] - data["timestamp"][0]) * 1e-6

    plt.figure()
    for key in data.keys():
        if key.startswith("output[") or key.startswith("control["):
            plt.plot(t, data[key], label=key)

    if len(plt.gca().lines) == 0:
        for key in data.keys():
            if key not in ["timestamp", "timestamp_sample"]:
                arr = np.asarray(data[key])
                if arr.ndim == 1 and np.issubdtype(arr.dtype, np.number):
                    plt.plot(t, arr, label=key)

    plt.xlabel("Time [s]")
    plt.ylabel("Actuator output")
    plt.title("Actuator Outputs")
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(OUT_DIR, "kishu_quad_actuator_outputs.png"), dpi=300, bbox_inches="tight")
    plt.close()
    print("Saved kishu_quad_actuator_outputs.png")
else:
    print("No actuator output dataset found.")

# Manual control setpoint
manual = get_dataset(["manual_control_setpoint"])
if manual is not None:
    data = manual.data
    t = (data["timestamp"] - data["timestamp"][0]) * 1e-6

    plt.figure()
    for key in ["throttle", "roll", "pitch", "yaw", "aux1", "aux2"]:
        if key in data:
            plt.plot(t, data[key], label=key)

    plt.xlabel("Time [s]")
    plt.ylabel("Manual control setpoint")
    plt.title("Manual Control Setpoint")
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(OUT_DIR, "force_land_manual_control.png"), dpi=300, bbox_inches="tight")
    plt.close()
    print("Saved force_land_manual_control.png")
else:
    print("No manual_control_setpoint dataset found.")
