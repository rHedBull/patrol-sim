"""Split a GLB mesh into structural and equipment parts using a labeled point cloud.

Usage:
    python scripts/split_mesh.py <source.glb> <labeled.ply> <output_dir>

The labeled PLY should have RGB colors where grey (80,80,80) = equipment/rest
and any other color = structural. The script splits the mesh triangles based on
nearest-neighbor lookup against the point cloud.
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import trimesh
from plyfile import PlyData
from scipy.spatial import cKDTree


def load_keep_points(ply_path: str, flip_yz: bool = False) -> np.ndarray:
    """Load point cloud of points to KEEP (floor + equipment). Returns positions array."""
    print(f"Loading keep-points PLY: {ply_path}")
    ply = PlyData.read(ply_path)
    v = ply["vertex"]
    positions = np.column_stack([v["x"], v["y"], v["z"]])
    if flip_yz:
        # GLB Y-up convention: negate Y and Z to match
        positions[:, 1] = -positions[:, 1]
        positions[:, 2] = -positions[:, 2]
        print("  Applied Y/Z flip to match GLB coordinate system")
    print(f"  Points to keep: {len(positions)}")
    return positions


def split_mesh(
    mesh_path: str, ply_path: str, output_dir: str, distance_threshold: float = 0.15
) -> None:
    """Split mesh into structural.glb and equipment.glb.

    The PLY contains points to KEEP (floor + equipment). Mesh faces whose centroids
    are within distance_threshold of a keep-point are classified as equipment.
    Everything else is structural (walls/ceiling).
    """
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)

    # Load mesh first to get the scene transform
    print(f"Loading mesh: {mesh_path}")
    scene = trimesh.load(mesh_path, force="scene")

    # Get the common node transform (GLB Y-up convention)
    node_transform = np.eye(4)
    geom_items = []
    for node_name in scene.graph.nodes_geometry:
        transform, geom_name = scene.graph[node_name]
        geom = scene.geometry[geom_name]
        if isinstance(geom, trimesh.Trimesh):
            node_transform = transform
            geom_items.append((geom_name, geom, transform))

    if not geom_items:
        for name, geom in scene.geometry.items():
            if isinstance(geom, trimesh.Trimesh):
                geom_items.append((name, geom, np.eye(4)))

    # Load keep-points and transform them into local mesh space
    keep_positions = load_keep_points(ply_path)
    # Transform PLY world-space points into local mesh space using inverse transform
    inv_transform = np.linalg.inv(node_transform)
    keep_positions_local = trimesh.transformations.transform_points(keep_positions, inv_transform)
    print(f"Transformed PLY points to local mesh space (inverse of GLB node transform)")
    print("Building KD-tree...")
    tree = cKDTree(keep_positions_local)

    structural_meshes = []
    equipment_meshes = []

    for name, geom, transform in geom_items:
        print(f"  Processing '{name}': {len(geom.faces)} faces")

        # Compute face centroids in LOCAL space (matching the KD-tree)
        face_verts = geom.vertices[geom.faces]  # (n_faces, 3, 3)
        centroids = face_verts.mean(axis=1)  # (n_faces, 3)

        # Query nearest keep-point for each centroid
        distances, _ = tree.query(centroids, k=1)

        # Faces close to a keep-point = equipment, far away = structural
        face_is_structural = distances > distance_threshold

        structural_faces = geom.faces[face_is_structural]
        equipment_faces = geom.faces[~face_is_structural]

        print(f"    Structural faces: {len(structural_faces)}")
        print(f"    Equipment faces:  {len(equipment_faces)}")

        # Apply world transform to vertices so exported GLB has correct orientation
        vertices_world = trimesh.transformations.transform_points(geom.vertices, transform)
        normals_world = None
        if geom.vertex_normals is not None:
            normal_matrix = np.linalg.inv(transform[:3, :3]).T
            normals_world = geom.vertex_normals @ normal_matrix.T

        # Create sub-meshes with world-space vertices
        if len(structural_faces) > 0:
            sm = trimesh.Trimesh(
                vertices=vertices_world,
                faces=structural_faces,
                vertex_normals=normals_world,
                process=False,
            )
            if hasattr(geom, "visual") and geom.visual is not None:
                sm.visual = geom.visual
            structural_meshes.append(sm)

        if len(equipment_faces) > 0:
            em = trimesh.Trimesh(
                vertices=vertices_world,
                faces=equipment_faces,
                vertex_normals=normals_world,
                process=False,
            )
            if hasattr(geom, "visual") and geom.visual is not None:
                em.visual = geom.visual
            equipment_meshes.append(em)

    # Export
    if structural_meshes:
        structural_scene = trimesh.Scene(structural_meshes)
        structural_path = output / "structural.glb"
        structural_scene.export(str(structural_path))
        print(f"\nExported structural mesh: {structural_path}")
    else:
        print("\nNo structural faces found!")

    if equipment_meshes:
        equipment_scene = trimesh.Scene(equipment_meshes)
        equipment_path = output / "equipment.glb"
        equipment_scene.export(str(equipment_path))
        print(f"Exported equipment mesh: {equipment_path}")
    else:
        print("No equipment faces found!")


if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python scripts/split_mesh.py <source.glb> <labeled.ply> <output_dir>")
        sys.exit(1)

    split_mesh(sys.argv[1], sys.argv[2], sys.argv[3])
