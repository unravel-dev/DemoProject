#!/usr/bin/env python3
"""Patch animation .anim sources to match a reference clip (format v2 root_motion metadata).

For Mixamo exports, facing correction at import only rotates the skeleton *scene root*
channel. Exported clips contain mixamorig:* channels only, so channel keys usually need
no transform — copy root_motion parent_bind_rotation from a freshly reimported reference.

Usage:
  python patch_animation_clips.py --reference path/to/Idle_mixamo_com.anim --folder path/to/Mannequin
  python patch_animation_clips.py --reference Idle_mixamo_com.anim --folder . --dry-run
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any


def quat_mul_y180(w: float, x: float, y: float, z: float) -> tuple[float, float, float, float]:
    """Multiply quaternion by 180-degree rotation around Y (engine facing fix)."""
    # G = (0, 1, 0, ~0) for pi rotation around Y in (w,x,y,z) if using xyzw... 
    # Engine stores quat as x,y,z,w in JSON. G_y180 in x,y,z,w = (0, 1, 0, 0) for angle pi.
    gx, gy, gz, gw = 0.0, 1.0, 0.0, 0.0
    # q_result = G * q  (hamilton product), q = (x,y,z,w)
    qw, qx, qy, qz = w, x, y, z
    rw = gw * qw - gx * qx - gy * qy - gz * qz
    rx = gw * qx + gx * qw + gy * qz - gz * qy
    ry = gw * qy - gx * qz + gy * qw + gz * qx
    rz = gw * qz + gx * qy - gy * qx + gz * qw
    return rx, ry, rz, rw


def rotate_vec3_y180(x: float, y: float, z: float) -> tuple[float, float, float]:
    # 180 Y: (x,y,z) -> (-x, y, -z)
    return -x, y, -z


def patch_channel_facing(channel: dict[str, Any]) -> bool:
    changed = False
    for key in channel.get("position_keys", []):
        v = key["value"]
        nx, ny, nz = rotate_vec3_y180(v["x"], v["y"], v["z"])
        if (nx, ny, nz) != (v["x"], v["y"], v["z"]):
            v["x"], v["y"], v["z"] = nx, ny, nz
            changed = True
    for key in channel.get("rotation_keys", []):
        v = key["value"]
        rx, ry, rz, rw = quat_mul_y180(v["w"], v["x"], v["y"], v["z"])
        if (rx, ry, rz, rw) != (v["x"], v["y"], v["z"], v["w"]):
            v["x"], v["y"], v["z"], v["w"] = rx, ry, rz, rw
            changed = True
    return changed


def load_anim(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def save_anim(path: Path, data: dict[str, Any]) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as f:
        json.dump(data, f, indent=1)
        f.write("\n")


def patch_file(
    target_path: Path,
    reference_root_motion: dict[str, Any],
    rotate_root_channel: str | None,
    dry_run: bool,
) -> None:
    data = load_anim(target_path)
    anim = data["animation"]
    rm = anim.setdefault("root_motion", {})

    fields = ("position_parent_bind_rotation", "rotation_parent_bind_rotation")
    for field in fields:
        if field in reference_root_motion:
            rm[field] = json.loads(json.dumps(reference_root_motion[field]))

    keys_changed = False
    if rotate_root_channel:
        for channel in anim.get("channels", []):
            if channel.get("node_name") == rotate_root_channel:
                keys_changed = patch_channel_facing(channel)
                break

    print(
        f"{'[dry-run] ' if dry_run else ''}{target_path.name}: "
        f"root_motion metadata updated"
        + (f", rotated channel '{rotate_root_channel}'" if keys_changed else "")
    )

    if not dry_run:
        save_anim(target_path, data)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, required=True, help="Reimported reference .anim")
    parser.add_argument("--folder", type=Path, required=True, help="Folder with clips to patch")
    parser.add_argument(
        "--rotate-root-channel",
        default="",
        help="Optional scene-root node name whose keys need Y-180 (usually empty for Mixamo)",
    )
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    ref_path = args.reference.resolve()
    folder = args.folder.resolve()

    if not ref_path.is_file():
        print(f"Reference not found: {ref_path}", file=sys.stderr)
        return 1

    ref_rm = load_anim(ref_path)["animation"]["root_motion"]
    for field in ("position_parent_bind_rotation", "rotation_parent_bind_rotation"):
        if field not in ref_rm:
            print(f"Reference missing {field}; reimport reference mesh/anim first.", file=sys.stderr)
            return 1

    rotate_root = args.rotate_root_channel or None
    patched = 0
    for anim_path in sorted(folder.glob("*.anim")):
        if anim_path.resolve() == ref_path:
            continue
        patch_file(anim_path, ref_rm, rotate_root, args.dry_run)
        patched += 1

    print(f"Done. Patched {patched} file(s). Recompile .anim assets in the editor.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
