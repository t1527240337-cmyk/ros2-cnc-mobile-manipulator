#!/usr/bin/env python3
"""Generate deterministic AprilTag textures used by the Gazebo factory."""

from pathlib import Path

import cv2
import numpy as np


TAG_IDS = (1, 2, 3, 10, 11, 20)
MARKER_PIXELS = 256
QUIET_ZONE_PIXELS = 32


def create_marker(dictionary, tag_id: int) -> np.ndarray:
    """Return a tag with a white quiet zone required by the detector."""
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(dictionary, tag_id, MARKER_PIXELS)
    else:
        marker = cv2.aruco.drawMarker(dictionary, tag_id, MARKER_PIXELS)

    image_size = MARKER_PIXELS + 2 * QUIET_ZONE_PIXELS
    image = np.full((image_size, image_size), 255, dtype=np.uint8)
    edge = QUIET_ZONE_PIXELS + MARKER_PIXELS
    image[QUIET_ZONE_PIXELS:edge, QUIET_ZONE_PIXELS:edge] = marker
    return image


def create_obj(tag_id: int) -> str:
    """Return a square mesh with explicit UV coordinates."""
    return f"""mtllib tag36h11_{tag_id}.mtl
v -0.10 -0.10 0
v  0.10 -0.10 0
v  0.10  0.10 0
v -0.10  0.10 0
vt 0 0
vt 1 0
vt 1 1
vt 0 1
vn 0 0 1
usemtl tag_material
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
"""


def create_mtl(tag_id: int) -> str:
    """Return the material which binds a tag texture to the UV mesh."""
    return f"""newmtl tag_material
Ka 1.0 1.0 1.0
Kd 1.0 1.0 1.0
Ks 0.0 0.0 0.0
illum 1
map_Kd ../materials/textures/tag36h11_{tag_id}.png
"""


def main() -> None:
    project_root = Path(__file__).resolve().parents[1]
    texture_dir = (
        project_root
        / "src/mobile_manipulator_description/models/factory_tags/materials/textures"
    )
    mesh_dir = (
        project_root / "src/mobile_manipulator_description/models/factory_tags/meshes"
    )
    texture_dir.mkdir(parents=True, exist_ok=True)
    mesh_dir.mkdir(parents=True, exist_ok=True)

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    for tag_id in TAG_IDS:
        texture_path = texture_dir / f"tag36h11_{tag_id}.png"
        mesh_path = mesh_dir / f"tag36h11_{tag_id}.obj"
        material_path = mesh_dir / f"tag36h11_{tag_id}.mtl"
        if not cv2.imwrite(str(texture_path), create_marker(dictionary, tag_id)):
            raise RuntimeError(f"Failed to write {texture_path}")
        mesh_path.write_text(create_obj(tag_id), encoding="utf-8")
        material_path.write_text(create_mtl(tag_id), encoding="utf-8")
        print(texture_path.relative_to(project_root))
        print(mesh_path.relative_to(project_root))


if __name__ == "__main__":
    main()
