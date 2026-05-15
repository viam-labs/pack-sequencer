# pack-sequencer

## What this is

Viam module that registers a single service: **`viam:pack-sequencer:sequencer`** under `rdk:service:world_state_store`.

The service owns the *pack-order math + cursor* for a palletizing workcell:

- Computes which box goes in which slot at which layer/orientation given pallet geometry + box dimensions. Supports column-fill and 2:1 interlock-brick patterns.
- Maintains a cursor (`next_seq`) and a done/failed/skipped set, advanced by `report_placement` / `skip_box` / `reset_cursor`.
- Returns pre-composed world-frame `place_start` and `place_end` poses per cycle via `next_box`, so the palletizer doesn't need its own pallet-origin compose step.
- Exposes the active set of placed boxes + any caller-supplied dynamic Transforms via the WorldStateStore API (`ListUUIDs`, `GetTransform`, `StreamTransformChanges`) so the 3D scene viewer renders the live state of the pallet.

This is one of the four sibling modules in the workcell ecosystem.

| Module | Role |
|---|---|
| `viam:workcell-components` | Owns pallet + pick-station pose/dims via their `frame:` blocks. |
| `viam:cell-configure-webapp` | Apps-only module shipping the operator UI. |
| `shrews-testing:palletizing-module` | The palletizer state machine. Calls this service every cycle for `next_box`, `report_placement`. |

## Dependency on workcell-components

Pack-sequencer reads the pallet's pose and dimensions via DoCommand to `viam:workcell-components:pallet` on **every** pack-order computation — there is no cache. workcell-components 0.2.0+ supports live `set_dimensions` / `set_color` DoCommand mutation, and pack-sequencer was caching at construction, so dim updates required bouncing pack-sequencer (dryrun pain point). Live-fetching costs ~1ms per call (in-process gRPC over UNIX socket) and is a wash next to the actual pack-order arithmetic.

This is intentional rather than reading the frame system directly — DoCommand gives a stable contract independent of frame-system internals. See README for the rationale.

## DoCommand surface

| Verb | Args | Returns | Used by |
|---|---|---|---|
| `next_box` | none | placement + pre-composed world poses + counters | palletizer every cycle |
| `report_placement` | `{seq, success, error?}` | counters + complete flag | palletizer at end of cycle |
| `get_box_dims` | none | `{box_length_mm, box_width_mm, box_height_mm}` | palletizer at construction |
| `get_pallet_home` | none | `pallet_home_local` + `pallet_home_world` | palletizer's `resolvePalletHomePose` |
| `get_pack_order` | none | full placement list + pallet pose/dims | webapp 3D preview, verify_pallet |
| `get_progress` | none | done/failed/skipped seqs + counts | webapp polling for live UI |
| `set_box_transform` / `clear_box_transform` | `{seq, ...}` | ack | palletizer's `emitAttachVisual` / `emitDropoffVisual` |
| `reset_cursor` | none | `{reset, next_seq}` | palletizer's reset path |
| `skip_box` | `{seq, reason?}` | ack | operator UI |
| `get_attributes` / `set_attributes` | none / partial Config | full Config | operator UI for live edits |

Typed Go structs for `next_box`, `report_placement`, and `get_box_dims` live in `github.com/viam-labs/viamkit/contracts`. Both producer (here) and consumer (palletizer) import them so wire keys can't drift.

## Conventions

- **Cursor survives reconfigure.** A pallet edit (box dimensions, layer count) cascades through AlwaysRebuild but the cursor preserves through. Only `reset_cursor` (or a Config that invalidates the pack order) clears it.
- **Pack-order math is recomputed per call, not cached.** `packColumn` / `packInterlock` are pure functions of Config + pallet dims. Cheap (<1ms for 100-box pallets) and avoids cache-invalidation bugs.
- **Pallet pose + dims are live-fetched per call, not cached.** Lets operators update pallet `set_dimensions` / drag the pallet without a pack-sequencer bounce. `palletInfo()` does the DoCommand; callers MUST invoke it before locking p.mu (the DoCommand round-trips through gRPC and can't hold our mutex). See doNextBox / doReportPlacement / GetTransform call patterns.
- **Strict attribute validation at construction.** `rejectUnknownAttributes` round-trips the raw attribute map through `json.DisallowUnknownFields` so typos like `box_width` (vs `box_width_mm`) error at config-load instead of silently becoming 0 and reporting `is_complete=true` on cycle 1.
- **Default box color is cardboard brown** (`#b08850`, see `defaultBoxColor`). The WSS renderer's default is red — without the default, placed boxes and in-flight box transforms render red. Override via `box_color: {r, g, b, opacity?}` in Config.
- **Inline emit of WorldStateStore changes.** `set_box_transform` / `clear_box_transform` / `report_placement` push to a buffered `changeChan` so the 3D scene reflects state immediately. Buffer cap 128; overflow logs at warn.

## Dependencies

- `github.com/viam-labs/viamkit/geom` — `Pose6D` (type alias).
- `github.com/viam-labs/viamkit/contracts` — verb constants + typed structs for `next_box`, `report_placement`, `get_box_dims` (others still use raw `map[string]any`).

## Layout

```
pack-sequencer/
├── go.mod
├── meta.json
├── Makefile
├── VERSION
├── README.md
├── CLAUDE.md         (this file)
├── module.go         (single big file — Config, pack-order math, DoCommand dispatch, WorldStateStore impl)
└── cmd/module/main.go
```

## Build + publish

```
make publish        # builds + uploads at $(cat VERSION)
```

Bump `VERSION` first.

## What to watch when editing

- **Wire format compatibility.** Adding fields to a verb's response is safe; renaming or removing them breaks palletizer pinned to older versions. When adding typed structs to `viamkit/contracts`, keep the JSON tags stable.
- **Pack-order determinism.** `packColumn` and `packInterlock` must be deterministic — the palletizer and the webapp both rely on running them and getting the same answer. The webapp currently has its own JS implementations (drift risk noted in the cross-module audit — fix is `preview_pack_order` DoCommand, planned).
- **WorldStateStore broadcast under load.** The mutex covers `dynamicBoxes` + `cursor` reads. Don't hold it during the emit — the channel send is async but the broadcaster could block briefly under heavy load. Inline emit is set up to release the lock before sending.

## Repo + registry

- GitHub: [`viam-labs/pack-sequencer`](https://github.com/viam-labs/pack-sequencer)
- Registry: `viam:pack-sequencer`
- Latest published: `0.3.0` (`set_box_transform` honors user-supplied pose [nested or flat]; per-call color override; viamkit v0.11.0)
