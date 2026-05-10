"""Headless route rendering: drive a plan in a Playwright-controlled browser
and stitch the captured frames into an MP4.

Usage::

    python -m scripts.render_route --plan A,B,C --out route.mp4

Assumes the dev server is already running at ``--server`` (default
``http://localhost:5000``). Requires ``playwright`` (Chromium) and ``ffmpeg``
on PATH.
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path
from urllib.parse import urlencode


def render(
    plan: list[str],
    out_path: Path,
    *,
    server: str = "http://localhost:5000",
    graph: str | None = None,
    view: str = "potree",
    speed: float = 2.0,
    fps: int = 15,
    width: int = 1280,
    height: int = 720,
    max_seconds: float = 600.0,
    show_browser: bool = False,
    keep_frames: bool = False,
) -> Path:
    try:
        from playwright.sync_api import sync_playwright
    except ImportError as exc:  # pragma: no cover
        raise SystemExit(
            "playwright is not installed. Install with:\n"
            "  pip install playwright && playwright install chromium"
        ) from exc

    if shutil.which("ffmpeg") is None:
        raise SystemExit("ffmpeg not found on PATH")

    qs = urlencode(
        {
            "headless": "1",
            "plan": ",".join(plan),
            "view": view,
            "speed": f"{speed}",
            **({"graph": graph} if graph else {}),
        }
    )
    url = f"{server}/?{qs}"

    frames_dir = Path(tempfile.mkdtemp(prefix="route-frames-"))
    print(f"[render_route] frames dir: {frames_dir}")
    print(f"[render_route] url: {url}")

    with sync_playwright() as pw:
        browser = pw.chromium.launch(headless=not show_browser)
        try:
            context = browser.new_context(viewport={"width": width, "height": height})
            page = context.new_page()
            page.goto(url)

            print("[render_route] waiting for headless ready…")
            page.wait_for_function("window.__headlessReady === true", timeout=120_000)
            print("[render_route] capturing frames…")

            frame_period = 1.0 / fps
            start = time.monotonic()
            frame_idx = 0
            while True:
                t0 = time.monotonic()
                page.screenshot(path=str(frames_dir / f"f{frame_idx:06d}.png"))
                frame_idx += 1

                done = page.evaluate("window.__planDone === true")
                if done:
                    # Capture one last frame at the final pose
                    page.screenshot(path=str(frames_dir / f"f{frame_idx:06d}.png"))
                    frame_idx += 1
                    break

                elapsed = time.monotonic() - start
                if elapsed > max_seconds:
                    print(f"[render_route] hit max_seconds={max_seconds}; stopping early")
                    break

                # Pace to target FPS
                lag = frame_period - (time.monotonic() - t0)
                if lag > 0:
                    time.sleep(lag)

            print(f"[render_route] captured {frame_idx} frames in {time.monotonic()-start:.1f}s")
        finally:
            browser.close()

    out_path = out_path.resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "ffmpeg", "-y",
        "-framerate", str(fps),
        "-i", str(frames_dir / "f%06d.png"),
        "-c:v", "libx264",
        "-pix_fmt", "yuv420p",
        "-preset", "medium",
        "-crf", "20",
        str(out_path),
    ]
    print(f"[render_route] ffmpeg → {out_path}")
    subprocess.run(cmd, check=True)

    if not keep_frames:
        shutil.rmtree(frames_dir, ignore_errors=True)
    else:
        print(f"[render_route] kept frames at {frames_dir}")

    return out_path


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--plan", required=True, help="Comma-separated node IDs (route order)")
    p.add_argument("--out", default="route.mp4", help="Output MP4 path")
    p.add_argument("--server", default="http://localhost:5000")
    p.add_argument("--graph", default=None, help="Saved graph name to load (optional)")
    p.add_argument("--view", choices=["potree", "mesh"], default="potree")
    p.add_argument("--speed", type=float, default=2.0)
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--max-seconds", type=float, default=600.0)
    p.add_argument("--show-browser", action="store_true", help="Show the browser window")
    p.add_argument("--keep-frames", action="store_true")
    args = p.parse_args()

    plan = [s.strip() for s in args.plan.split(",") if s.strip()]
    if len(plan) < 1:
        print("error: --plan must have at least one node id", file=sys.stderr)
        return 1

    out = render(
        plan,
        Path(args.out),
        server=args.server,
        graph=args.graph,
        view=args.view,
        speed=args.speed,
        fps=args.fps,
        width=args.width,
        height=args.height,
        max_seconds=args.max_seconds,
        show_browser=args.show_browser,
        keep_frames=args.keep_frames,
    )
    print(f"[render_route] wrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
