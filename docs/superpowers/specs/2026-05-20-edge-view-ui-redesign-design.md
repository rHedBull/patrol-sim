# Edge View UI Redesign

**Date:** 2026-05-20
**Status:** Design, pending implementation
**Related:** `docs/graph-editor.md`, PR #7 (`feat/edge-aux-views`)

## Context

PR #7 introduced per-edge auxiliary view captures and a render-skip toggle. The current Edge Panel (`static/index.html:504`) exposes a checkbox + `+ Add view` button. Views are modeled as `{side: "left"|"right", tilt: -90..90}` in `navigation/graph.py:22-31`. Edges with views render solid cyan; render-skipped edges render as faded gray tubes; selection is by clicking the tube (`EDGE_RADIUS = 0.02`).

Three usability issues drive this redesign:

1. Edges are hard to click at `0.02` radius.
2. The dichotomy "solid cyan = has views / faded gray = skipped" relies on color saturation alone — easy to miss at a glance.
3. View angles are abstract (`side` + `tilt` numbers) with no spatial preview in the 3D view; users can't see *where* a view will point until they render.

## Goals

- Thicker, easier-to-pick edges.
- Visually unambiguous distinction between render-on, render-off, and views-attached.
- Direct manipulation of view direction in the 3D scene via a draggable arrow handle.
- Single intuitive parameter per view: a rotation angle around the edge axis.

## Non-goals

- Backward-compat for existing graph files. The feature is one PR old; we hard-break the schema.
- Variable sample position along an edge (`t`). Views remain anchored at the edge midpoint for both capture and the on-edge arrow glyph.
- Looking along the direction of travel (forward/back). The new control surface is roll-only; views always point radially out from the edge axis.

## Design

### 1. Edge thickness

`EDGE_RADIUS` in `static/index.html:1136` increases from `0.02` to `0.05`. Visual edges become 2.5× thicker; the existing raycast picker (`graphEdgeLines` array, tube meshes) is unchanged so no other code is affected. No change to graph layout or geometry math.

### 2. Render-skipped edge styling

Skipped edges (`render: false`) render as a **dashed line** instead of a faded tube. Implementation: replace the cylinder mesh with a `THREE.Line` using `LineDashedMaterial`:

- Color `0x666666`, opacity `0.5`
- `dashSize: 0.15`, `gapSize: 0.10`
- Must call `line.computeLineDistances()` after creation for dashes to render
- Line width: default (Three.js can't reliably set line width across platforms; that's acceptable for the de-emphasized state)

Solid tube vs. dashed line is a strong, unambiguous visual signal: "this edge will not be rendered."

If the edge is both selected and skipped: dashed line tinted orange (`edgeMaterialSelected` color) — selection still takes precedence visually, but the dashing remains.

### 3. Data model change (hard break)

`View` in `navigation/graph.py:22-31` becomes:

```python
@dataclass(frozen=True)
class View:
    roll_deg: float  # 0..360, angle around edge axis; 0° = world-up direction

    def __post_init__(self) -> None:
        if not (0.0 <= float(self.roll_deg) < 360.0):
            raise ValueError(f"View.roll_deg must be in [0, 360), got {self.roll_deg}")
```

Consequences:

- `Side` type alias is removed.
- `view_canonical_key(v)` returns `int(round(v.roll_deg)) % 360` (single integer for dedupe + filenames).
- `views_in_traversal_direction(views, reversed_)`: when `reversed_=True`, `roll_deg` mirrors as `(360 - roll_deg) % 360` (the "right side" of forward travel is the "left side" of reverse travel; reflecting across the up-axis flips the ring angle).
- `set_edge_views` keeps the `len(views) <= 3` and uniqueness checks.
- JSON serialization (`graph.py:215`) writes `{"roll_deg": v.roll_deg}`; loader (`graph.py:253`) reads the new field. Files written by PR #7 fail validation with a clear error — acceptable per the hard-break decision.

### 4. Camera math (edge-local frame)

A view's look direction is computed from `roll_deg` plus the edge's local frame:

```
forward = normalize(to_pos - from_pos)        # traversal direction
world_up = (0, 1, 0)
up_perp  = normalize(world_up - (world_up·forward)·forward)  # world-up projected onto plane ⟂ forward
right_perp = normalize(forward × up_perp)
look_dir = cos(roll)·up_perp + sin(roll)·right_perp
```

- `roll = 0°` → look straight up
- `roll = 90°` → look right relative to traversal (matches old `side: "right", tilt: 0`)
- `roll = 180°` → look down
- `roll = 270°` → look left

Degenerate case: a near-vertical edge makes `up_perp` ill-defined. Detection: `|world_up·forward| > 0.99`. Fallback: `up_perp = (1, 0, 0)`. Patrol graphs are essentially horizontal, so this branch is defensive but rarely hit.

Render pipeline (`navigation/render.py` or equivalent — file to be confirmed during implementation) replaces the existing `side`+`tilt` → camera-direction code with the formula above. Camera position is the sample point on the edge (mid-edge); the existing capture logic that places the camera on the path is unchanged.

### 5. View indicator glyph (3D scene)

For each view on each edge, render a cone glyph in the graph group:

- Geometry: `THREE.ConeGeometry(0.04, 0.25, 12)` — radius 0.04 m, length 0.25 m
- Material: cyan (`0x55ddee`), unlit (`MeshBasicMaterial`) so it stays readable from any angle
- Anchor: edge midpoint
- Orientation: cone axis aligned with `look_dir(roll_deg)`, base at midpoint, tip pointing radially outward
- `userData`: `{ kind: "edgeView", edgeFrom, edgeTo, viewIndex }` for picking

When 2–3 views share an edge, all cones anchor at the same midpoint but radiate out at distinct angles — no overlap because their tips point in different directions.

Edges with `views.length > 0` keep a cyan tube tint (existing `edgeMaterialWithViews`) so the "this edge has views" signal is visible even when zoomed out where individual cones are tiny.

### 6. Drag interaction

When an edge is selected (matches `selectedEdgeKey`), its view-cones become drag handles:

- **Hover** a cone → cursor changes to grab, cone scales 1.15×
- **Mouse-down** on a cone → enter rotate-view mode (set a state flag, capture `viewIndex`)
- **Mouse-move** while rotating:
  - Cast mouse ray into scene
  - Find the intersection of the ray with the plane through the edge midpoint perpendicular to `forward`
  - `v = intersection - midpoint`
  - New `roll_deg = atan2(v·right_perp, v·up_perp)` (mod 360)
  - Update cone orientation live; mirror value into edge-panel slider
- **Mouse-up** → exit rotate mode, persist via existing `set_edge_views` flow

While dragging, `OrbitControls` (or equivalent) is disabled to avoid camera rotation conflicting with cone rotation. A single `isRotatingView` flag gates both pointer handlers and orbit input.

### 7. Edge panel changes

Edge panel keeps its current location (toolbar strip, `static/index.html:504`) and overall layout. Per-view rows in `#edge-views` change:

- Before: `[side ▼] [tilt ##] [×]`
- After: `[roll: ###°] [slider 0–359] [×]`

The slider and number input are bound to `roll_deg`. Changes from the 3D drag push into both controls; changes from the panel push into the cone orientation. Both paths call the same `setEdgeViewRoll(edgeKey, viewIndex, roll_deg)` helper.

`+ Add view` button creates a new view with `roll_deg = 90` (right-of-travel default). Disabled when `views.length >= 3`, as today.

### 8. File touchpoints

- `navigation/graph.py` — `View` dataclass, `view_canonical_key`, `views_in_traversal_direction`, JSON IO at `:212-254`
- `navigation/render.py` (or wherever view → camera-pose conversion lives) — replace `side`+`tilt` math with `roll_deg` math
- `server.py` — request/response payload field rename if the API surfaces `side`/`tilt`
- `static/index.html`:
  - `EDGE_RADIUS` (`:1136`)
  - Edge tube material selection (`:1224-1232`) — dashed-line branch
  - Cone glyph creation in the edge-render loop (`:1205-1243`)
  - Drag handlers (new) wired into the existing pointer-event pipeline
  - Edge panel view-row template (`:1783`)
- `tests/` — replace existing `side`/`tilt` test fixtures with `roll_deg` equivalents; add tests for `views_in_traversal_direction` mirror semantics
- `docs/graph-editor.md` — document ring-angle model, dashed-edge styling, drag interaction

### 9. Test plan

- `tests/test_graph.py`: `View(roll_deg=...)` accepts `[0, 360)`, rejects out-of-range; serialization round-trips `roll_deg`; reverse traversal yields `(360 - roll) % 360`; canonical-key dedupe by integer roll.
- `tests/test_render.py` (or render path tests): camera look direction for `roll_deg ∈ {0, 90, 180, 270}` on a horizontal edge along +X yields the expected world-space vectors (±Y, ±Z).
- Manual UI verification (browser-verification skill): select edge → drag cone → cone rotates with mouse, panel slider updates; toggle render off → tube becomes dashed; create 3 views at rolls 30/120/240 → three cones radiate from midpoint without overlap.

## Risks

- **Three.js dashed line width** — `LineDashedMaterial` ignores `linewidth` on most WebGL platforms. Mitigation: skipped edges are de-emphasized by design, so a thin dashed line is acceptable; if visibility suffers, swap to a custom shader or segmented-cylinder approach.
- **Drag accuracy on grazing edges** — when the camera nearly aligns with the edge's forward axis, the perpendicular plane projects to a near-line on screen and roll becomes hard to control. Mitigation: clamp dragging to a minimum delta; this is a known limitation of axis-aligned gizmos, not a blocker.
- **Schema break impact** — anyone with locally saved graphs from PR #7 will need to recreate views. Acceptable given the feature is one PR old; the loader emits a clear error on the missing `roll_deg` field.

## Open questions

None — all design decisions confirmed during brainstorming.
