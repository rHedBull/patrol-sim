"""Compress + simplify a large GLB so the browser can actually load it.

Pipeline tuned for huge LiDAR-reconstructed meshes (tens of millions of
triangles, hundreds of MB):

  1. Load the GLB with ``trimesh`` (handles the geometry + scene graph).
  2. Decimate each mesh with ``fast-simplification`` (native quadric edge
     collapse — handles 40M-tri meshes fine; WASM-based tools OOM at this size).
  3. Write the decimated scene back to GLB with ``trimesh``.
  4. Optionally run ``npx gltfpack -cc`` for Meshopt compression on top of the
     decimated mesh (now small enough that gltfpack's WASM heap is fine).

A 783 MB / 40M-tri GLB typically comes out at 30-80 MB and renders in Three.js
via ``MeshoptDecoder``.

Usage:
    uv run python scripts/optimize_mesh.py <input.glb> [output.glb] [--ratio 0.2]
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np
import trimesh


def default_output(input_path: Path) -> Path:
    return input_path.with_name(input_path.stem + ".optimized.glb")


def _decimate_geometry(mesh: trimesh.Trimesh, ratio: float) -> trimesh.Trimesh:
    """Quadric decimation via fast-simplification, preserving vertex colors when possible."""
    import fast_simplification

    if len(mesh.faces) == 0:
        return mesh

    # Drop any unreferenced vertices before simplifying — fast-simplification's
    # color-replay step has indexing bugs when the input has isolated vertices.
    mesh = mesh.copy()
    mesh.remove_unreferenced_vertices()

    target = max(4, int(len(mesh.faces) * ratio))
    if target >= len(mesh.faces):
        return mesh

    verts = np.asarray(mesh.vertices, dtype=np.float32)
    faces = np.asarray(mesh.faces, dtype=np.uint32)

    new_verts, new_faces, collapses = fast_simplification.simplify(
        verts, faces, target_count=target, return_collapses=True,
    )

    # Forward vertex colors through the collapse map. LiDAR scenes are
    # textureless and rely on per-vertex RGB, so preserve when we can — but
    # don't fail the whole optimization if replay_simplification chokes
    # on the topology (it can with high-genus / weird-manifold meshes).
    visual = None
    vc = getattr(mesh.visual, "vertex_colors", None)
    if vc is not None and len(vc) == len(mesh.vertices):
        try:
            rgb = np.asarray(vc, dtype=np.float32)[:, :3]
            new_rgb, _, _ = fast_simplification.replay_simplification(rgb, faces, collapses)
            new_colors = np.empty((new_rgb.shape[0], 4), dtype=np.uint8)
            new_colors[:, :3] = np.nan_to_num(np.clip(new_rgb, 0, 255), nan=128.0).astype(np.uint8)
            new_colors[:, 3] = 255
            visual = trimesh.visual.ColorVisuals(vertex_colors=new_colors)
        except Exception as exc:
            print(f"    [warn] color replay failed ({exc}); falling back to nearest-neighbor lookup")
            from scipy.spatial import cKDTree
            tree = cKDTree(verts)
            _, idx = tree.query(new_verts, k=1)
            picked = np.asarray(vc, dtype=np.uint8)[idx]
            if picked.shape[1] == 3:
                picked = np.concatenate([picked, np.full((len(picked), 1), 255, dtype=np.uint8)], axis=1)
            visual = trimesh.visual.ColorVisuals(vertex_colors=picked)

    return trimesh.Trimesh(vertices=new_verts, faces=new_faces, visual=visual, process=False)


def _decimate_scene(scene: trimesh.Scene, ratio: float) -> trimesh.Scene:
    """Apply decimation to every mesh in the scene graph in place."""
    out = trimesh.Scene()
    for name, geom in scene.geometry.items():
        if isinstance(geom, trimesh.Trimesh):
            print(f"  [{name}] {len(geom.faces):>10,} -> ", end="", flush=True)
            simplified = _decimate_geometry(geom, ratio)
            print(f"{len(simplified.faces):>10,} faces")
            out.add_geometry(simplified, geom_name=name)
        else:
            out.add_geometry(geom, geom_name=name)
    return out


def _meshopt_compress(input_glb: Path, output_glb: Path) -> bool:
    """Run `npx gltfpack -cc` on a (smaller) GLB. Returns True on success."""
    npx = shutil.which("npx")
    if not npx:
        return False
    cmd = [npx, "--yes", "-p", "gltfpack@^0.20", "gltfpack",
           "-i", str(input_glb), "-o", str(output_glb), "-cc", "-kn"]
    print(f"[optimize_mesh] {' '.join(cmd)}")
    try:
        subprocess.run(cmd, check=True)
        return True
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        print(f"[optimize_mesh] gltfpack compression skipped: {exc}", file=sys.stderr)
        return False


def optimize(
    input_path: Path,
    output_path: Path,
    *,
    simplify_ratio: float = 0.2,
    force: bool = False,
    skip_compress: bool = False,
) -> Path:
    if not input_path.exists():
        raise FileNotFoundError(input_path)

    if (
        not force
        and output_path.exists()
        and output_path.stat().st_mtime >= input_path.stat().st_mtime
    ):
        print(f"[optimize_mesh] up to date: {output_path}")
        return output_path

    output_path.parent.mkdir(parents=True, exist_ok=True)

    print(f"[optimize_mesh] loading {input_path} ...")
    loaded = trimesh.load(str(input_path), force="scene")
    if isinstance(loaded, trimesh.Trimesh):
        scene = trimesh.Scene(loaded)
    else:
        scene = loaded

    print(f"[optimize_mesh] decimating to ratio={simplify_ratio} ...")
    decimated = _decimate_scene(scene, simplify_ratio)

    with tempfile.NamedTemporaryFile(suffix=".glb", delete=False) as tmp:
        intermediate = Path(tmp.name)
    try:
        print(f"[optimize_mesh] writing intermediate GLB ...")
        decimated.export(str(intermediate), file_type="glb")

        compressed = False
        if not skip_compress:
            compressed = _meshopt_compress(intermediate, output_path)
        if not compressed:
            shutil.move(str(intermediate), str(output_path))
    finally:
        if intermediate.exists():
            intermediate.unlink()

    in_mb = input_path.stat().st_size / (1024 * 1024)
    out_mb = output_path.stat().st_size / (1024 * 1024)
    print(f"[optimize_mesh] {in_mb:.1f} MB -> {out_mb:.1f} MB ({out_mb / in_mb * 100:.1f}%)")
    return output_path


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("input", type=Path)
    p.add_argument("output", type=Path, nargs="?", default=None)
    p.add_argument(
        "--ratio", type=float, default=0.2, dest="simplify_ratio",
        help="Target triangle ratio (default: 0.2 = keep 20%% of triangles).",
    )
    p.add_argument("--force", action="store_true", help="Re-run even if output is newer.")
    p.add_argument("--skip-compress", action="store_true",
                   help="Skip the gltfpack -cc post-pass (Meshopt compression).")
    args = p.parse_args()

    output = args.output or default_output(args.input)
    optimize(
        args.input, output,
        simplify_ratio=args.simplify_ratio,
        force=args.force,
        skip_compress=args.skip_compress,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
