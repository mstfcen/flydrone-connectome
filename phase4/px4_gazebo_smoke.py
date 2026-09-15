#!/usr/bin/env python3
import asyncio
import json
import time
from pathlib import Path

from mavsdk import System

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "px4_gazebo_smoke.json"

async def first_matching(stream, predicate, timeout):
    async def _inner():
        async for item in stream:
            if predicate(item):
                return item
    return await asyncio.wait_for(_inner(), timeout=timeout)

async def main():
    started = time.time()
    result = {
        "connected": False,
        "health_ready": False,
        "armed": False,
        "takeoff_confirmed": False,
        "landed": False,
        "max_relative_altitude_m": 0.0,
        "duration_s": None,
    }

    drone = System()
    await drone.connect(system_address="udpin://0.0.0.0:14540")
    state = await first_matching(
        drone.core.connection_state(), lambda s: s.is_connected, 75
    )
    result["connected"] = bool(state.is_connected)
    print("PX4_CONNECTED")

    health = await first_matching(
        drone.telemetry.health(),
        lambda h: h.is_global_position_ok and h.is_home_position_ok,
        90,
    )
    result["health_ready"] = bool(
        health.is_global_position_ok and health.is_home_position_ok
    )
    print("PX4_HEALTH_READY")

    await drone.action.set_takeoff_altitude(3.0)
    await drone.action.arm()
    result["armed"] = True
    print("PX4_ARMED")

    await drone.action.takeoff()

    async def wait_for_takeoff():
        async for pos in drone.telemetry.position():
            alt = float(pos.relative_altitude_m)
            result["max_relative_altitude_m"] = max(
                result["max_relative_altitude_m"], alt
            )
            if alt >= 1.5:
                return alt
    takeoff_alt = await asyncio.wait_for(wait_for_takeoff(), timeout=35)
    result["takeoff_confirmed"] = True
    result["takeoff_altitude_m"] = takeoff_alt
    print(f"PX4_TAKEOFF_CONFIRMED altitude={takeoff_alt:.2f}")

    await asyncio.sleep(3.0)
    await drone.action.land()
    print("PX4_LAND_SENT")

    async def wait_for_landed():
        async for in_air in drone.telemetry.in_air():
            if not in_air:
                return True

    await asyncio.wait_for(wait_for_landed(), timeout=45)
    result["landed"] = True
    print("PX4_LANDED")

    result["duration_s"] = round(time.time() - started, 3)
    OUT.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as exc:
        failure = {
            "error": type(exc).__name__,
            "message": str(exc),
        }
        OUT.write_text(json.dumps(failure, indent=2), encoding="utf-8")
        raise
