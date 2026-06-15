# pack-sequencer

## What this is

Viam module that registers a single service: **`viam:pack-sequencer:sequencer`** under `rdk:service:world_state_store`.

The service owns the *pack-order math + cursor* for a palletizing workcell:

- Computes which box goes in which slot at which layer/orientation given pallet geometry + box dimensions. Supports column-fill and 2:1 interlock-brick patterns.
- Maintains a cursor (`next_seq`) and a done/failed/skipped set, advanced by `report_placement` / `skip_box` / `reset_progress`.
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
| `next_box` | none | placement + pre-composed world poses + `is_complete` (NO counters) | palletizer every cycle |
| `report_placement` | `{seq, success, error?}` | counters + `complete` flag (`next_box_index`) | palletizer at end of cycle |
| `get_box_dims` | none | `{box_length_mm, box_width_mm, box_height_mm}` | palletizer at construction |
| `get_pallet_home` | none | `pallet_home_local` + `pallet_home_world` | palletizer's `resolvePalletHomePose` |
| `get_pack_order` | none | full placement list + pallet pose/dims | webapp 3D preview, verify_pallet |
| `get_status` | none | `next_box_index`, done/failed/skipped seqs + bare counts (`placed`/`failed`/`skipped`/`remaining`/`total`) + `complete` | palletizer obstacle cache, webapp polling |
| `set_box_transform` / `clear_box_transform` | `{seq, ...}` | ack | palletizer's `emitAttachVisual` / `emitDropoffVisual` |
| `reset_progress` | none | `{reset, next_box_index}` | palletizer's reset path |
| `skip_box` | `{seq, reason?}` | `{skipped, next_box_index, placed, remaining}` | operator UI |
| `get_attributes` / `set_attributes` | none / partial Config | full Config | operator UI for live edits |

The wire contract is the in-repo nested module `github.com/viam-labs/pack-sequencer/contracts` (stdlib-only — no rdk). The producer here marshals every response through its typed structs (`contracts.MustToMap(contracts.XResponse{...})`); the consumer (palletizer) imports the same module and uses its typed client (`contracts.NextBox(ctx, svc)` etc.). A renamed JSON tag is a compile error on both ends. The flat verb keys above are what those structs serialize to.

**Verb-rename note (0.4.0):** `get_progress`→`get_status`, `reset_cursor`→`reset_progress`; `next_box` no longer returns progress counters (use `get_status`); `next_seq`→`next_box_index`; `get_status` counters dropped the `_count` suffix. Breaking — ships in lockstep with the palletizer.

**Pose-field note (0.4.0-rc1 / contracts v0.2.0):** `next_box`'s pallet-frame fields are now symmetric with the world-frame pair — `pose_in_pallet`/`approach_offset_in_pallet` → `place_start_in_pallet`/`place_end_in_pallet` (so the response is `place_{start,end}_in_{world,pallet}`). The place move is the two-pose trajectory PlaceStart → PlaceEnd (angled descent). `PackOrderPlacement` (get_pack_order) keeps its `pose_in_*` naming for now.

## Conventions

- **Cursor survives reconfigure.** A pallet edit (box dimensions, layer count) cascades through AlwaysRebuild but the cursor preserves through. Only `reset_progress` (or a Config that invalidates the pack order) clears it.
- **Pack-order math is recomputed per call, not cached.** `packColumn` / `packInterlock` are pure functions of Config + pallet dims. Cheap (<1ms for 100-box pallets) and avoids cache-invalidation bugs.
- **Pallet pose + dims are live-fetched per call, not cached.** Lets operators update pallet `set_dimensions` / drag the pallet without a pack-sequencer bounce. `palletInfo()` does the DoCommand; callers MUST invoke it before locking p.mu (the DoCommand round-trips through gRPC and can't hold our mutex). See doNextBox / doReportPlacement / GetTransform call patterns.
- **Strict attribute validation at construction.** `rejectUnknownAttributes` round-trips the raw attribute map through `json.DisallowUnknownFields` so typos like `box_width` (vs `box_width_mm`) error at config-load instead of silently becoming 0 and reporting `is_complete=true` on cycle 1.
- **Default box color is cardboard brown** (`#b08850`, see `defaultBoxColor`). The WSS renderer's default is red — without the default, placed boxes and in-flight box transforms render red. Override via `box_color: {r, g, b, opacity?}` in Config.
- **Inline emit of WorldStateStore changes.** `set_box_transform` / `clear_box_transform` / `report_placement` push to a buffered `changeChan` so the 3D scene reflects state immediately. Buffer cap 128; overflow logs at warn.

## Dependencies

- `github.com/viam-labs/viamkit/geom` — `Pose6D` (the producer's internal, rdk-backed pose type).
- `github.com/viam-labs/viamkit/viz` — WorldStateStore Transform builders.
- `github.com/viam-labs/pack-sequencer/contracts` — the in-repo nested wire-contract module (stdlib-only). Consumed via `replace ./contracts`. The producer marshals all DoCommand responses through it; `module.go` converts its internal `geom.Pose6D` to the contract's `contracts.Pose6D` at the wire boundary via `toContractsPose`.

## Layout

```
pack-sequencer/
├── go.mod                  (require + replace ./contracts)
├── meta.json
├── Makefile
├── VERSION
├── README.md
├── CLAUDE.md               (this file)
├── module.go               (single big file — Config, pack-order math, DoCommand dispatch, WorldStateStore impl)
├── contracts/              (NESTED MODULE: github.com/viam-labs/pack-sequencer/contracts, stdlib-only)
│   ├── go.mod              (no requires — keep it dependency-free; deps_test.go enforces)
│   ├── codec.go            (ToMap / FromMap[T] / MustToMap)
│   ├── pose.go             (Pose6D, Vec3 — plain JSON, NO spatialmath converter)
│   ├── colors.go           (Color)
│   ├── packsequencer.go    (verb constants + request/response structs)
│   ├── client.go           (DoCommander + typed verb functions)
│   └── *_test.go           (codec, wire-shape, deps guardrail)
└── cmd/module/main.go
```

**Nested-module tag rule:** the contracts module is tagged `contracts/vX.Y.Z` (path-prefixed — a bare `vX.Y.Z` will NOT publish it). Consumers `require github.com/viam-labs/pack-sequencer/contracts vX.Y.Z`; during local dev they use a `replace` to a local checkout.

## Build + publish

```
make publish        # builds + uploads at $(cat VERSION)
```

Bump `VERSION` first.

## What to watch when editing

- **Wire format compatibility.** Adding fields to a verb's response is safe; renaming or removing them breaks palletizer pinned to older versions. When adding typed structs to the in-repo `contracts` module, keep the JSON tags stable.
- **Pack-order determinism.** `packColumn` and `packInterlock` must be deterministic — the palletizer and the webapp both rely on running them and getting the same answer. The webapp currently has its own JS implementations (drift risk noted in the cross-module audit — fix is `preview_pack_order` DoCommand, planned).
- **WorldStateStore broadcast under load.** The mutex covers `dynamicBoxes` + `cursor` reads. Don't hold it during the emit — the channel send is async but the broadcaster could block briefly under heavy load. Inline emit is set up to release the lock before sending.

## Repo + registry

- GitHub: [`viam-labs/pack-sequencer`](https://github.com/viam-labs/pack-sequencer)
- Registry: `viam:pack-sequencer`
- Latest published: `0.4.0-rc1` (symmetric `place_{start,end}_in_{world,pallet}` on `next_box`; contracts tagged `contracts/v0.2.0`)
- `0.4.0-rc0` (in-repo `contracts` nested module + producer marshals through it; `next_box` drops counters; `get_progress`→`get_status`, `reset_cursor`→`reset_progress`, `next_seq`→`next_box_index`. Contracts `contracts/v0.1.0`.)
- Prior: `0.3.0` (`set_box_transform` honors user-supplied pose [nested or flat]; per-call color override; viamkit v0.11.0)
