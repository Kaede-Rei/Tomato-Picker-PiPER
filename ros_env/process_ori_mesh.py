#!/usr/bin/env python3
# Process original STL meshes for visualization, collision, and optional convex hull generation
# Usage: Put your mesh in ros_env/meshes/ and then bash "python ./ros_env/process_ori_mesh.py"

import trimesh
from pathlib import Path

# 目标面数比例
TARGET_FACE_RATIO = 0.1
# 是否启用凸包生成
CONVEX_ENABLE = True

script_dir = Path(__file__).resolve().parent
src = script_dir / "meshes"

out_visual = script_dir / "processed_meshes" / "visual"
out_collision = script_dir / "processed_meshes" / "collision"
out_convex = script_dir / "processed_meshes" / "convex"

out_visual.mkdir(parents=True, exist_ok=True)
out_collision.mkdir(parents=True, exist_ok=True)
out_convex.mkdir(parents=True, exist_ok=True)

files = list(src.glob("*.stl")) + list(src.glob("*.STL"))
if not files:
    print("No STL files found in the source directory.")
    exit(1)
for file in files:
    print("-" * 40)
    print("Processing:", file.name)
    mesh = trimesh.load(file)

    mesh.export(out_visual / file.name)
    if len(mesh.faces) > int(TARGET_FACE_RATIO * len(mesh.faces)):
        print(
            f"Simplifying mesh from {len(mesh.faces)} faces to {int(TARGET_FACE_RATIO * len(mesh.faces))} faces for:",
            file.name,
        )
        simplified = mesh.simplify_quadric_decimation(1 - TARGET_FACE_RATIO)
        simplified.export(out_collision / file.name)
        print(
            "Simplification completed for:",
            file.name,
            "and final face count:",
            len(simplified.faces),
        )
    else:
        print("Mesh already has fewer than target faces, skipping simplification.")
        mesh.export(out_collision / file.name)

    if CONVEX_ENABLE:
        convex = mesh.convex_hull
        convex.export(out_convex / file.name)
        print(
            "Generating convex hull for:",
            file.name,
            "and final face count:",
            len(convex.faces),
        )
    print("-" * 40)

print("done!")
