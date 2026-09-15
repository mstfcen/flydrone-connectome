#!/usr/bin/env python3
import json
import time
from pathlib import Path

from pymavlink import mavutil

OUT = Path(__file__).with_name("mavlink_obstacle_distance.json")

def finite_distances(msg):
    vals = [int(v) for v in msg.distances]
    return [v for v in vals if 0 < v < 65535]

def main():
    conn = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    hb = conn.wait_heartbeat(timeout=45)
    if hb is None:
        raise TimeoutError("No MAVLink heartbeat on UDP 14550")
    target_system = conn.target_system
    target_component = conn.target_component
    conn.mav.command_long_send(
        target_system, target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_OBSTACLE_DISTANCE,
        100000, 0, 0, 0, 0, 0,
    )
    print(f"MAVLINK_HEARTBEAT system={target_system} component={target_component}")
    deadline = time.time() + 25
    samples = []
    while time.time() < deadline:
        msg = conn.recv_match(type="OBSTACLE_DISTANCE", blocking=True, timeout=2)
        if msg is None:
            continue
        vals = finite_distances(msg)
        if vals:
            sample = {
                "min_cm": min(vals),
                "max_cm": max(vals),
                "count": len(vals),
                "increment_deg": float(msg.increment),
                "angle_offset_deg": float(getattr(msg, "angle_offset", 0.0)),
            }
            samples.append(sample)
            print("OBSTACLE_DISTANCE", sample)
            if sample["min_cm"] < 1200:
                break

    result = {
        "heartbeat": True,
        "received_obstacle_distance": bool(samples),
        "samples": samples[-5:],
    }
    OUT.write_text(json.dumps(result, indent=2), encoding="utf-8")
    if not samples:
        raise RuntimeError("No OBSTACLE_DISTANCE MAVLink message received")

if __name__ == "__main__":
    main()
