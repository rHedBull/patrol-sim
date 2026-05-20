# Per-edge auxiliary view angles + render skip

**Date:** 2026-05-20
**Status:** Design

## Problem

Today, route rendering captures one forward-facing frame per arc-length sample along the chained A* path. Many inspection use-cases want **side-looking** frames at the same sample points (e.g. equipment walls, ceilings, floor seams), and some path segments are only there for connectivity and shouldn't produce frames at all.

This adds two per-edge controls:

1. **Auxiliary views** — up to 3 extra side-looking captures per sample on the edge, each defined by a side (left/right) and a tilt angle.
2. **Render skip** — a boolean to disable frame capture entirely on selected edges while still traversing them.

## Scope

In:
- Graph JSON schema extension (backward compatible).
- `NavGraph` round-trip and helpers for edge metadata.
- Graph-edit UI: per-edge panel for `render` toggle and `views` list.
- Visual styling: dashed/grey for skipped edges; tinted for edges with views.
- Render loop changes: per-segment edge lookup, multi-capture per sample, skip-segment short-circuit, suffixed filenames, manifest `view` field.

Out (YAGNI):
- Roll axis, translational camera offsets, per-view quality presets, per-view labels.
- Side-views on the orbit renderer (orbit mode stays as-is).

## Data model

`NavGraph` becomes mildly metadata-aware on edges. Internal storage:

```python
@dataclass
class EdgeMeta:
    render: bool = True
    views: list[View] = field(default_factory=list)  # max 3

@dataclass
class View:
    side: Literal["left", "right"]
    tilt: float  # degrees, [-90, 90]; 0 = horizontal, +up, -down
```

The `_edges: dict[str, set[str]]` adjacency stays for cheap neighbor lookup; metadata lives in a parallel `_edge_meta: dict[frozenset[str, str], EdgeMeta]` keyed by the unordered endpoint pair. New helpers:

- `get_edge_meta(a, b) -> EdgeMeta` (returns default if absent)
- `set_edge_meta(a, b, meta)`
- `set_edge_render(a, b, render: bool)`
- `set_edge_views(a, b, views: list[View])`

### JSON schema

Edge entries gain two optional fields:

```json
{
  "from": "node_A",
  "to": "node_B",
  "render": false,
  "views": [
    { "side": "left",  "tilt": 10 },
    { "side": "right", "tilt": -45 }
  ]
}
```

Defaults when missing: `render: true`, `views: []`. Old graphs deserialize unchanged. `to_dict` only emits the fields when they differ from default, keeping diffs minimal.

### Direction handling

Edges are bidirectional and stored once (`from < to`). Views are defined in the canonical `from → to` orientation. When a render sample traverses the edge in reverse (`to → from`), `side` is mirrored (left↔right) so the camera continues to look at the same physical wall regardless of walk direction. `tilt` is preserved (gravity is direction-independent).

The `render` flag is symmetric — skipping an edge skips it in both directions.

## Render-loop semantics

`renderPlanFrames` in `static/index.html` currently:

1. Resolves chained path → `positions[]`.
2. `arcLengthSample(positions, spacing)` → `samples[]` of `{pos, yaw, arcLen}`.
3. For each sample: drive robot pose, capture once via Potree, POST `/api/render_frame`.

Changes:

- **Per-segment edge tracking.** `arcLengthSample` already knows the current segment index `seg` while walking `cum[]`. Extend its output to include `seg` per sample. `/api/plan/path` already returns `[{id, position}, ...]` per node; today `renderPlanFrames` discards ids via `path.map(n => ({x,y,z}))`. Change: also capture `ids = path.map(n => n.id)`. For sample `seg = k`, the traversed edge endpoints are `(ids[k], ids[k+1])`. The frontend resolves edge metadata from a `graphEdgesByPair` map (keyed by sorted `[a,b]` join) built at graph-load time.
- **Direction is per-segment, not per-edge.** A chained path can revisit the same edge (loops/backtracks). For each segment, compare `(ids[k], ids[k+1])` against the canonical edge ordering (`from < to`) stored in the metadata: if forward, apply views as-is; if reversed, mirror `side`. Re-evaluated independently per occurrence.
- **Skip.** If the edge for `samples[i]` has `render === false`, skip the entire sample (no pose change, no capture, no POST), regardless of any `views` defined on it. Index `i` advances; the manifest becomes sparse, which downstream tools tolerate (they iterate `frames[]`). Validated by the explicit test in §Testing.
- **Multi-view capture.** For a non-skipped sample, capture forward as today (`yaw = s.yaw`, `pitch = 0`). Then for each view in the edge's `views` (after direction-mirror), set:
  - `robotYaw = s.yaw + (sideMirrored === "left" ? -π/2 : +π/2)`
  - `robotPitch = tilt_radians` — confirmed against existing code (`robotCamera.rotation.order = 'YXZ'`, `rotation.x = robotPitch`); in Three.js this convention makes **+pitch = look up**, matching `+tilt = up`.
  - capture, POST with view-suffixed metadata.
  Restore `robotPitch = 0` between samples.
- **POST payload.** New optional field `view`. The server uses `(index, view)` as the dedupe key when replacing manifest entries.
- **Mid-render graph edits.** Edit-mode UI is already gated by the live/edit toggle; we additionally disable the Edge Panel inputs and **Save Graph** while `planActive === true`. The render loop also snapshots `graphEdgesByPair` at start so concurrent mutations (if any escape the gate) cannot corrupt an in-flight run.

### Filenames and manifest

- Forward frame keeps the existing name: `frame_NNNN.png`.
- View frames append `__<viewcode>` where viewcode is `L<sign><tilt>` or `R<sign><tilt>` (sign always emitted, tilt is `|tilt|` rounded to nearest integer, no zero-padding): `frame_0042__L+10.png`, `frame_0042__R-45.png`, `frame_0042__L+0.png`. The sign of `0` is canonicalized to `+`.
- **Duplicate views are rejected.** `set_edge_views` and `from_dict` validate that the canonicalized key `(side, signed_round(tilt))` — with `-0` normalized to `0` — is unique within an edge's views list; collisions error rather than silently overwriting frames. The same canonicalization is used for the filename viewcode, so dedupe and on-disk uniqueness agree by construction.
- Manifest entries include:
  ```json
  { "index": 42, "file": "frame_0042__L+10.png",
    "view": "L+10", "side": "left", "tilt": 10,
    "yaw_offset_deg": -90, "pitch_offset_deg": 10,
    "position": [...], "yaw": ..., "arc_length": ... }
  ```
  Forward frames keep `view: "forward"` for consistency.

## UI (graph-edit mode)

State today: edges are click-to-delete in connect mode. We add an **edit-edge selection** distinct from the delete gesture.

- New UX: in edit mode, clicking an edge **selects** it and opens an **Edge Panel** beneath the existing graph-edit controls. Clicking empty space deselects. The previous click-to-delete-on-any-edge gesture is removed — delete is always via the panel's button. This unifies behavior so users don't have to know whether an edge has metadata; the migration cost is one extra click for the (rare) delete operation.
- Edge Panel contents:
  - Header: `edge: <nodeA short id> ↔ <nodeB short id>` + **Delete edge** button.
  - `[x] Render this edge` checkbox.
  - **Views** list (up to 3 rows):
    - Each row: `[ Left | Right ]` toggle, `tilt °` number input (step 5, range −90..90), delete row.
    - `+ Add view` button (disabled at 3).
    - Preset chips that prefill a new row: `side` (tilt 0), `side ↑30°`, `side ↓30°`.
- Persistence: edits are local to the in-memory graph until **Save Graph** (matches current node/edge behavior).

### Visual styling

Edge-line material varies by metadata:

- Default edge: current style.
- Has views (`views.length > 0`): tinted (e.g. cyan).
- Render skipped (`render === false`): dashed grey, lower opacity. Takes precedence over the "has views" tint.
- Selected: highlight outline regardless of state.

## Server-side changes

`/api/render_frame` accepts an optional `view` (string, default `"forward"`). Manifest dedupe key changes from `index` to `(index, view)`. **Migration:** on each write, entries in the existing manifest that lack a `view` field are normalized to `view: "forward"` in-place before applying the dedupe filter, so reruns over a pre-existing manifest don't leave orphan untagged entries.

No new endpoints. The graph save/load endpoints don't change — they already round-trip whatever `NavGraph.to_dict` produces.

## Backward compatibility

- Old graph JSON (no `render`, no `views`) loads with defaults; saving rewrites without those fields when defaults apply, so diffs stay clean for legacy graphs.
- Old render manifests don't have a `view` field; readers should treat missing `view` as `"forward"`.
- `/api/render_frame` calls without `view` continue to write `frame_NNNN.png` with `view: "forward"`.

## Testing

Unit tests (`tests/` mirrors `navigation/graph.py`):

- Round-trip a graph with views + skip flag through `to_dict` / `from_dict`.
- `set_edge_views` enforces `len(views) <= 3`, `tilt ∈ [-90, 90]`, and unique `(side, round(tilt))` within the list.
- `from_dict` applies the same validation; malformed/out-of-range JSON raises (no silent clamping).
- Direction-mirror helper: given `views` defined `A→B`, traversal `B→A` returns side-mirrored copies with tilt preserved.
- Legacy graph JSON (no metadata fields) loads with defaults; saving back is byte-identical for legacy graphs with default metadata.
- Render-skip wins over views: an edge with `render=false` AND `views=[...]` produces zero frames for samples on that segment (asserted via a render-loop unit-level helper, since the full capture loop is browser-side).

Frontend / integration verified manually in the browser per `browser-verification` skill: place a graph, mark one edge with two views and one with `render=false`, run a render, confirm filename suffixes, sparse manifest indices, and correct camera poses in the captured PNGs.

## Risks / open questions

- **Segment-to-edge mapping precision.** `arcLengthSample` interpolates across segment boundaries; samples landing exactly on a node belong to both adjoining edges. Convention: a sample at arc-length `s` belongs to the segment whose `cum[seg] <= s < cum[seg+1]`. The very last sample (at `s = total`) is assigned to the final segment. Documented in code; deterministic and matches how `seg` is already tracked.
- **Edge-click ergonomics.** Today clicking an edge deletes it. New behavior unifies all edge interactions behind selection: a click selects, and delete happens via the Edge Panel button. This costs one extra click for the previously-instant delete gesture, but eliminates a mode-dependent affordance.
- **Manifest sparseness.** Downstream tooling that assumes contiguous `index` would break. The existing manifest already supports gaps in principle (entries are sorted by index); we'll grep the repo for any consumer that assumes contiguity and adjust or flag.
