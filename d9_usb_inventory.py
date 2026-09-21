#!/usr/bin/env python3
"""D9-1 USB inventory.  It neither starts CAN nor sends any CAN frame."""
from __future__ import annotations

import json
import time
from pathlib import Path


def main() -> int:
    result: dict[str, object] = {"recorded_at": time.strftime("%Y-%m-%d %H:%M:%S")}

    from gs_usb.gs_usb import GsUsb

    can_devices = []
    for index, device in enumerate(GsUsb.scan()):
        info = device.device_info
        can_devices.append({
            "index": index,
            "usb_bus": device.bus,
            "usb_address": device.address,
            "serial": device.serial_number,
            "interface_count": info.icount,
            "firmware_version": info.fw_version,
            "hardware_version": info.hw_version,
        })
    result["gs_usb_devices"] = can_devices

    try:
        import pyrealsense2 as rs
        context = rs.context()
        result["t265_devices"] = [
            {
                "name": device.get_info(rs.camera_info.name),
                "serial": device.get_info(rs.camera_info.serial_number),
                "firmware": device.get_info(rs.camera_info.firmware_version),
            }
            for device in context.query_devices()
        ]
        del context
    except Exception as error:
        result["t265_error"] = f"{type(error).__name__}: {error}"

    output_dir = Path("logs") / "d9_usb"
    output_dir.mkdir(parents=True, exist_ok=True)
    output = output_dir / f"inventory_{time.strftime('%Y%m%d_%H%M%S')}.json"
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, ensure_ascii=False))
    print(f"記録: {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
