# pack-sequencer

WorldStateStore service that owns the runtime state for a palletizing
workcell: pack-order math, placement cursor, placed-set, and a live
TransformChange stream of placed-box geometries.

## Model

| Model | API | Purpose |
|---|---|---|
| `viam:pack-sequencer:sequencer` | `rdk:service:world_state_store` | Computes pack order against a `viam:workcell-components:pallet`, tracks placement progress, and serves the WorldStateStore stream consumed by the 3D viewer and the planner-via-geometry path. |

## Responsibilities

1. **Pack-order computation** — given pallet dimensions (from a sibling `pallet` component), box dimensions, gap, pattern, and layer count, produces the ordered list of slot poses.
2. **Cursor + placed-set state** — tracks which seq is next and which seqs have been confirmed placed. Survives palletizer restarts.
3. **WorldStateStore implementation** — `ListUUIDs`, `GetTransform`, `StreamTransformChanges` for the 3D viewer + (future) motion-service consumption.
4. **Dynamic Transform broker** — accepts `set_box_transform` publishes for in-flight boxes (at pickup, on gripper, at place) and fans them out to subscribers.
5. **Pallet/box config aggregator** — single source of truth for box dimensions; the palletizer pulls these at construction.

## DoCommand surface

| Command | Purpose |
|---|---|
| `get_attributes` / `set_attributes` | Read or update the service's pack-order parameters. |
| `get_pack_order` | Return the full enriched placement list (poses in pallet + world frames, approach offsets). |
| `get_box_dims` / `get_pallet_home` | Single-source-of-truth queries for dimensions + the pre-place transit pose. |
| `next_box` / `report_placement` / `skip_box` / `reset_cursor` / `get_progress` | Cursor control. |
| `set_box_transform` / `clear_box_transform` | Dynamic in-flight Transform publish/clear. |

## Build

```
make module.tar.gz
viam module upload --version <X.Y.Z-rcN> --platform linux/amd64 module.tar.gz
```
