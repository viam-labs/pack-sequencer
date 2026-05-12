// Package palletwebappconfiguretest implements a generic component that
// serves two audiences:
//
//  1. A companion single_machine web application (see apps/) uses
//     get_attributes / set_attributes / get_pack_order to view, edit, and
//     visualize a pallet configuration.
//
//  2. The palletizer module consumes this component as the authoritative
//     source of pack-order state via next_box / report_placement /
//     skip_box / reset_cursor. The palletizer asks for the next box to
//     place, executes it, and reports success or failure; this component
//     owns the sequence cursor and pallet-frame geometry. All poses are
//     returned in the *pallet frame* — the palletizer composes its own
//     pallet_origin to get world-frame poses.
package packsequencer

import (
	"bytes"
	"context"
	"encoding/json"
	"fmt"
	"math"
	"sort"
	"sync"

	commonpb "go.viam.com/api/common/v1"
	wsspb "go.viam.com/api/service/worldstatestore/v1"
	"github.com/golang/geo/r3"
	wcsh "github.com/viam-labs/viamkit/geom"
	"go.viam.com/rdk/components/generic"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/worldstatestore"
	"go.viam.com/rdk/spatialmath"
)

var Model = resource.NewModel("viam", "pack-sequencer", "sequencer")

// Standard GMA wooden-pallet dimensions (48" × 40" × ~6"). Used as
// fallbacks when no pallet component is configured or when the configured
// component's frame.geometry is missing. Duplicated from
// viam:workcell-components rather than imported to keep the two modules
// independent at the Go level.
const (
	DefaultPalletWidthMM     = 1219.2 // 48 inches — pallet X
	DefaultPalletLengthMM    = 1016.0 // 40 inches — pallet Y
	DefaultPalletThicknessMM = 152.4  // 6 inches  — wooden base height (Z)
)

func init() {
	resource.RegisterService(worldstatestore.API, Model,
		resource.Registration[worldstatestore.Service, *Config]{
			Constructor: newPalletConfig,
		},
	)
}

// Config is the persisted attribute shape. Any of these may be zero / empty
// when the component starts; the webapp fills them in.
//
// Pallet width/length/thickness now come from the bound pallet
// component's frame.geometry — pack-sequencer is purely about the pack
// order on top of that pallet.
type Config struct {
	// Pallet is the resource name of a sibling `pallet` component. The
	// pallet's world pose and L/W/thickness come from its `frame.translation`
	// / `frame.orientation` / `frame.geometry`, which the user can edit
	// (and drag) directly in the Viam 3D viewer. This service queries the
	// pallet via DoCommand instead of holding its own pallet_origin or
	// pallet area dimensions, so there's a single source of truth.
	Pallet string `json:"pallet"`

	// PalletAreaHeightMM is the stacking ceiling — the maximum z (in
	// pallet-local frame) up to which boxes may be stacked. Independent
	// of the pallet's physical thickness; the pallet component owns
	// width/length/thickness via frame.geometry.
	PalletAreaHeightMM float64 `json:"pallet_area_height_mm"`

	BoxLengthMM float64 `json:"box_length_mm"`
	BoxWidthMM  float64 `json:"box_width_mm"`
	BoxHeightMM float64 `json:"box_height_mm"`

	// CenterOnPallet, when true, shifts the filled region so its centroid
	// lines up with the pallet-area centroid. Leftover space (pallet minus
	// used footprint) is split evenly on both sides instead of all ending
	// up on the +X / +Y edges.
	CenterOnPallet bool `json:"center_on_pallet,omitempty"`

	// BoxOffsetXMM / BoxOffsetYMM are extra gaps (mm) inserted between
	// adjacent slots in pallet-X and pallet-Y respectively. The pack
	// math packs slots flush by default (boxes touching edge-to-edge);
	// adding even a few mm of gap keeps the planner's collision check
	// from flagging neighbouring boxes as overlapping when they meet at
	// zero distance, and gives the descending held box room to settle
	// without grazing already-placed neighbours. Defaults to 0.
	BoxOffsetXMM float64 `json:"box_offset_x_mm,omitempty"`
	BoxOffsetYMM float64 `json:"box_offset_y_mm,omitempty"`

	// RotateAlternateLayers enables 2:1 brickwork interlocking. It is only
	// physically stable when the box's long side is exactly twice its short
	// side, because that ratio lets two boxes tile an L×L square in two
	// distinct orientations. Each odd layer uses the rotated orientation,
	// so every upper box spans the joint between two lower boxes and is
	// fully supported. For any other box ratio we fall back to column
	// stacking (direct stack, no rotation) and emit a warning; rotating
	// arbitrary rectangles would leave upper boxes floating over gaps.
	RotateAlternateLayers bool `json:"rotate_alternate_layers,omitempty"`

	Quantity int    `json:"quantity"`
	Label    string `json:"label,omitempty"`

	// Place approach parameters — consumed by next_box when it builds the
	// per-box approach_offset. Defaults mirror the palletizer module's own
	// defaults so swapping it in behaves the same as the legacy self-computed
	// poses.
	// PlaceApproachAngleDeg tilts the approach along the pallet's +X axis
	// (default 15°). The gripper descends from height down to the slot,
	// offset horizontally by height·tan(angle).
	PlaceApproachAngleDeg float64 `json:"place_approach_angle_deg,omitempty"` // default 15
	// PlaceApproachAngleYDeg adds an independent tilt along the pallet's
	// +Y axis (default 0°). Set it together with the X angle to approach
	// from a corner rather than an edge — makes the descent less likely
	// to slide against an already-placed neighbor on either axis.
	PlaceApproachAngleYDeg float64 `json:"place_approach_angle_y_deg,omitempty"`
	PlaceApproachHeightMM  float64 `json:"place_approach_height_mm,omitempty"` // default 100
	PlaceOrientation      *Pose6D `json:"place_orientation,omitempty"`         // orientation part only; default (0,0,-1,0)

	// PalletHome is the arm's pre-place transit waypoint, expressed in
	// pallet-local coordinates (mm + degrees). Theta rotates the gripper
	// yaw around world Z. Only X/Y/Z/Theta are consumed — orientation is
	// always gripper-straight-down (0,0,-1). Composed with the pallet
	// component's frame at read-time to produce the world-frame pose
	// the palletizer moves to. When unset, the webapp default is the
	// pallet's far corner at Z=200mm with theta=0.
	PalletHome *Pose6D `json:"pallet_home,omitempty"`

	// ObserverFrame is the parent frame for emitted world-state-store
	// transforms. Defaults to "world".
	ObserverFrame string `json:"observer_frame,omitempty"`
}

// Pose6D is re-exported from github.com/viam-labs/viamkit/geom so all
// workcell modules share the same JSON shape. Only the orientation fields
// of PlaceOrientation are consumed — position is always taken from the
// computed slot.
type Pose6D = wcsh.Pose6D

// cursor tracks which seq to place next and records outcomes. It lives in
// RAM and resets on reconfigure (AlwaysRebuild); reset_cursor is the only
// runtime control. "done" captures the final disposition per seq — success,
// failed (palletizer will retry on next next_box unless skipped), or
// skipped (counted as not-remaining).
type cursor struct {
	next int
	done map[int]string // seq → "success" | "failed" | "skipped"
	err  map[int]string // seq → error message from last failed report
}

func newCursor() cursor {
	return cursor{next: 1, done: map[int]string{}, err: map[int]string{}}
}

// Count outcomes by category.
func (c *cursor) counts() (placed, failed, skipped int) {
	for _, s := range c.done {
		switch s {
		case "success":
			placed++
		case "failed":
			failed++
		case "skipped":
			skipped++
		}
	}
	return
}

// Validate declares the pallet component as a required dependency when
// set, but otherwise accepts any shape — this service exists to be
// scribbled on by the webapp, so we don't reject partial configs at
// startup. Returning Pallet under the implicit-deps slot lets the Viam
// resource manager construct the pallet first and pass it in.
func (c *Config) Validate(_ string) ([]string, []string, error) {
	if c.Pallet == "" {
		return nil, nil, nil
	}
	return []string{c.Pallet}, nil, nil
}

type palletSequencer struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	logger logging.Logger

	mu     sync.Mutex
	cfg    Config
	cursor cursor

	// pallet is the sibling pallet component (when cfg.Pallet is set).
	// Source of truth for pallet pose and L/W/thickness. Resolved once
	// at construction; AlwaysRebuild on the dependency cascades so a
	// frame edit on the pallet (drag-and-save in the 3D viewer) rebuilds
	// this service with refreshed values.
	pallet resource.Resource

	// Cached pallet info — set at construction. Avoids per-call DoCommand
	// overhead and handles the no-pallet fallback uniformly.
	palletPose        spatialmath.Pose
	palletWidthMM     float64
	palletLengthMM    float64
	palletThicknessMM float64

	// World-state-store surface. observerFrame defaults to "world" when
	// unset. changeChan delivers ADDED/REMOVED events to any
	// StreamTransformChanges subscriber — the cursor mutators emit inline,
	// so there's no polling loop.
	observerFrame string
	changeChan    chan worldstatestore.TransformChange

	// dynamicBoxes holds caller-controlled Transforms (keyed by seq) that
	// override the default "placed box on pallet" rendering for boxes
	// mid-cycle: sitting at pickup before grasp, attached to the gripper
	// during transit, etc. A placement report clears the entry for that
	// seq, switching it back to the canonical world-pose-on-pallet
	// rendering derived from the pack order.
	//
	// Each dynamic transform gets a fresh, versioned UUID
	// ("box-<seq>-v<N>") on every set_box_transform call, because the
	// Viam 3D scene's stream handler ignores ADDED events for UUIDs it
	// already knows — and dropping the box via REMOVED then re-adding
	// the same UUID doesn't bring it back. A brand-new UUID sidesteps
	// both of those: REMOVE the prior UUID (box disappears), ADD the
	// new one (renderer sees it for the first time and draws it).
	dynamicBoxes   map[int]*commonpb.Transform
	dynamicVersion map[int]int
}

func newPalletConfig(
	_ context.Context,
	deps resource.Dependencies,
	conf resource.Config,
	logger logging.Logger,
) (worldstatestore.Service, error) {
	cfg, err := resource.NativeConfig[*Config](conf)
	if err != nil {
		return nil, err
	}

	observer := cfg.ObserverFrame
	if observer == "" {
		observer = "world"
	}

	var palletRes resource.Resource
	palletPose := spatialmath.NewZeroPose()
	pw, pl, pt := DefaultPalletWidthMM, DefaultPalletLengthMM, DefaultPalletThicknessMM
	if cfg.Pallet != "" {
		palletRes, err = resource.FromDependencies[resource.Resource](deps, generic.Named(cfg.Pallet))
		if err != nil {
			return nil, fmt.Errorf("pallet dependency %q not available: %w", cfg.Pallet, err)
		}
		centerPose, w, l, t, err := queryPalletAttributes(palletRes)
		if err != nil {
			return nil, fmt.Errorf("query pallet %q attributes: %w", cfg.Pallet, err)
		}
		// The Pallet component's frame is at the wood's centroid (Viam
		// convention: dragging the frame in the 3D viewer drags the
		// visible box). Pallet-local placements are measured from one
		// corner with z=0 at the top of the wood, so we offset by
		// (-w/2, -l/2, +t/2) in frame-local terms to get the corner pose
		// used for composition. Doing this once at construction means
		// downstream code just composes palletPose × pose_in_pallet.
		cornerOffset := spatialmath.NewPoseFromPoint(r3.Vector{X: -w / 2, Y: -l / 2, Z: t / 2})
		palletPose = spatialmath.Compose(centerPose, cornerOffset)
		pw, pl, pt = w, l, t
		logger.Infow("pallet component wired",
			"name", cfg.Pallet,
			"width_mm", pw, "length_mm", pl, "thickness_mm", pt,
			"corner_x", palletPose.Point().X, "corner_y", palletPose.Point().Y, "corner_z", palletPose.Point().Z,
		)
	}

	return &palletSequencer{
		name:              conf.ResourceName(),
		logger:            logger,
		cfg:               *cfg,
		cursor:            newCursor(),
		pallet:            palletRes,
		palletPose:        palletPose,
		palletWidthMM:     pw,
		palletLengthMM:    pl,
		palletThicknessMM: pt,
		observerFrame:     observer,
		changeChan:        make(chan worldstatestore.TransformChange, 128),
		dynamicBoxes:      map[int]*commonpb.Transform{},
		dynamicVersion:    map[int]int{},
	}, nil
}

// queryPalletAttributes asks the pallet component for its world pose and box
// dimensions in one DoCommand. Errors propagate so a misconfigured pallet
// surfaces at construction time rather than silently falling back to
// defaults.
func queryPalletAttributes(p resource.Resource) (spatialmath.Pose, float64, float64, float64, error) {
	resp, err := p.DoCommand(context.Background(), map[string]interface{}{"get_attributes": true})
	if err != nil {
		return nil, 0, 0, 0, err
	}
	width := asFloat(resp["width_mm"])
	length := asFloat(resp["length_mm"])
	thickness := asFloat(resp["thickness_mm"])
	poseRaw, _ := resp["pose"].(map[string]interface{})
	pose := spatialmath.NewPose(
		r3.Vector{X: asFloat(poseRaw["x"]), Y: asFloat(poseRaw["y"]), Z: asFloat(poseRaw["z"])},
		&spatialmath.OrientationVectorDegrees{
			OX: asFloat(poseRaw["o_x"]), OY: asFloat(poseRaw["o_y"]),
			OZ: asFloat(poseRaw["o_z"]), Theta: asFloat(poseRaw["theta"]),
		},
	)
	return pose, width, length, thickness, nil
}

func asFloat(v interface{}) float64 {
	switch n := v.(type) {
	case float64:
		return n
	case float32:
		return float64(n)
	case int:
		return float64(n)
	case int64:
		return float64(n)
	}
	return 0
}

func (p *palletSequencer) Name() resource.Name { return p.name }

// Placement describes one box slot in the computed pack order.
type Placement struct {
	Seq    int     `json:"seq"`
	Col    int     `json:"col"`
	Row    int     `json:"row"`
	Layer  int     `json:"layer"`
	XMM    float64 `json:"x_mm"`
	YMM    float64 `json:"y_mm"`
	ZMM    float64 `json:"z_mm"`
	Label  string  `json:"label,omitempty"`
	Length float64 `json:"length_mm"`
	Width  float64 `json:"width_mm"`
	Height float64 `json:"height_mm"`
}

// packOrder computes the fill sequence given the current cfg.
//
// Two modes:
//
//   - column (default): every layer is identical. Each upper box sits directly
//     on the box below it — fully supported for any box dimensions.
//
//   - interlock (c.RotateAlternateLayers == true, only when box L = 2·W or
//     W = 2·L within tolerance): boxes are grouped into L×L tiles holding 2
//     boxes each. Even layers orient boxes one way inside the tile, odd layers
//     rotate them 90°. Every upper box spans the seam between two lower boxes,
//     so the upper layer is fully supported by the lower one. Any other ratio
//     would leave upper boxes floating over gaps, so we reject those and fall
//     back to column mode with a warning.
//
// Returns top-level cols/rows that describe the *layer-0* layout for a summary
// display; the per-placement width_mm/length_mm fields carry the per-box
// orientation so the UI can render rotated boxes correctly.
func packOrder(c Config, palletWidthMM, palletLengthMM float64) (placements []Placement, cols, rows, layers, capacity int, mode string, warnings []string) {
	mode = "column"
	if c.BoxLengthMM <= 0 || c.BoxWidthMM <= 0 || c.BoxHeightMM <= 0 {
		return nil, 0, 0, 0, 0, mode, nil
	}
	layers = int(c.PalletAreaHeightMM / c.BoxHeightMM)
	if layers < 1 {
		return nil, 0, 0, 0, 0, mode, nil
	}

	if c.RotateAlternateLayers {
		long := math.Max(c.BoxLengthMM, c.BoxWidthMM)
		short := math.Min(c.BoxLengthMM, c.BoxWidthMM)
		if short > 0 && math.Abs(long-2*short) <= 0.5 {
			return packInterlock(c, palletWidthMM, palletLengthMM, layers, long, short)
		}
		warnings = append(warnings, fmt.Sprintf(
			"rotate_alternate_layers requires a 2:1 box (got %.1f × %.1f, ratio %.3f); "+
				"using column stack instead — rotating non-2:1 boxes would leave upper boxes unsupported",
			c.BoxLengthMM, c.BoxWidthMM, long/short))
	}
	placements, cols, rows, capacity = packColumn(c, palletWidthMM, palletLengthMM, layers)
	return placements, cols, rows, layers, capacity, mode, warnings
}

// packColumn: every layer identical, every box directly above the one below.
// Boxes are packed tight (no gaps) — pad box_length/box_width if real-world
// clearance is needed.
func packColumn(c Config, palletWidthMM, palletLengthMM float64, layers int) (placements []Placement, cols, rows, capacity int) {
	if c.BoxWidthMM <= 0 || c.BoxLengthMM <= 0 {
		return nil, 0, 0, 0
	}
	// Pitch = box dim + inter-slot gap. Negative offsets are clamped
	// to zero so a misconfigured negative gap can't make slots overlap.
	gapX := math.Max(0, c.BoxOffsetXMM)
	gapY := math.Max(0, c.BoxOffsetYMM)
	pitchX := c.BoxWidthMM + gapX
	pitchY := c.BoxLengthMM + gapY
	// Column count is based on full pitch except for the last column,
	// which only needs the box width to fit (the trailing gap doesn't
	// have to land on the pallet).
	cols = int((palletWidthMM-c.BoxWidthMM)/pitchX) + 1
	rows = int((palletLengthMM-c.BoxLengthMM)/pitchY) + 1
	if cols < 1 || rows < 1 {
		return nil, cols, rows, 0
	}
	capacity = cols * rows * layers
	n := c.Quantity
	if n <= 0 || n > capacity {
		n = capacity
	}
	// Span actually consumed by the box footprints + interior gaps.
	spanX := float64(cols)*c.BoxWidthMM + float64(cols-1)*gapX
	spanY := float64(rows)*c.BoxLengthMM + float64(rows-1)*gapY
	offX, offY := 0.0, 0.0
	if c.CenterOnPallet {
		offX = (palletWidthMM - spanX) / 2
		offY = (palletLengthMM - spanY) / 2
	}
	placements = make([]Placement, 0, n)
	seq := 1
	for layer := 0; layer < layers && seq <= n; layer++ {
		for row := 0; row < rows && seq <= n; row++ {
			for col := 0; col < cols && seq <= n; col++ {
				placements = append(placements, Placement{
					Seq:    seq,
					Col:    col,
					Row:    row,
					Layer:  layer,
					XMM:    offX + float64(col)*pitchX + c.BoxWidthMM/2,
					YMM:    offY + float64(row)*pitchY + c.BoxLengthMM/2,
					ZMM:    float64(layer)*c.BoxHeightMM + c.BoxHeightMM,
					Label:  c.Label,
					Width:  c.BoxWidthMM,
					Length: c.BoxLengthMM,
					Height: c.BoxHeightMM,
				})
				seq++
			}
		}
	}
	return placements, cols, rows, capacity
}

// packInterlock: 2:1 brickwork. Tile side = long. Two boxes per tile.
//
// Layer 0 pattern A: both boxes oriented long-along-Y, placed side-by-side
// along X. Each box footprint is short × long.
//
// Layer 1 pattern B: both boxes oriented long-along-X, placed stacked along Y.
// Each box footprint is long × short.
//
// Both patterns fill the same long × long tile footprint, so stacks align.
// A layer-1 box spans the X-seam of two layer-0 boxes (50/50 overlap on each).
func packInterlock(c Config, palletWidthMM, palletLengthMM float64, layers int, long, short float64) (placements []Placement, cols, rows, allLayers, capacity int, mode string, warnings []string) {
	mode = "interlock_2x1"
	tile := long // = 2 * short within tolerance
	allLayers = layers

	tileCols := int(palletWidthMM / tile)
	tileRows := int(palletLengthMM / tile)
	if tileCols < 1 || tileRows < 1 {
		return nil, 0, 0, allLayers, 0, mode, []string{fmt.Sprintf(
			"pallet %.1f × %.1f too small for 2:1 interlock tile %.1f × %.1f; need at least one full tile",
			palletWidthMM, palletLengthMM, tile, tile)}
	}

	perLayer := tileCols * tileRows * 2
	capacity = perLayer * layers
	// Report cols/rows describing pattern A (layer 0): 2·tileCols boxes wide, tileRows boxes deep.
	cols = tileCols * 2
	rows = tileRows

	n := c.Quantity
	if n <= 0 || n > capacity {
		n = capacity
	}

	offX, offY := 0.0, 0.0
	if c.CenterOnPallet {
		offX = (palletWidthMM - float64(tileCols)*tile) / 2
		offY = (palletLengthMM - float64(tileRows)*tile) / 2
	}

	placements = make([]Placement, 0, n)
	seq := 1
	for layer := 0; layer < layers && seq <= n; layer++ {
		patternA := layer%2 == 0
		for tr := 0; tr < tileRows && seq <= n; tr++ {
			for tc := 0; tc < tileCols && seq <= n; tc++ {
				tileX0 := offX + float64(tc)*tile
				tileY0 := offY + float64(tr)*tile
				for k := 0; k < 2 && seq <= n; k++ {
					var cx, cy, bw, bl float64
					var col, row int
					if patternA {
						// 2 boxes side-by-side along X inside the tile.
						cx = tileX0 + float64(k)*short + short/2
						cy = tileY0 + long/2
						bw, bl = short, long
						col, row = tc*2+k, tr
					} else {
						// 2 boxes stacked along Y inside the tile.
						cx = tileX0 + long/2
						cy = tileY0 + float64(k)*short + short/2
						bw, bl = long, short
						col, row = tc, tr*2+k
					}
					placements = append(placements, Placement{
						Seq:    seq,
						Col:    col,
						Row:    row,
						Layer:  layer,
						XMM:    cx,
						YMM:    cy,
						ZMM:    float64(layer)*c.BoxHeightMM + c.BoxHeightMM,
						Label:  c.Label,
						Width:  bw,
						Length: bl,
						Height: c.BoxHeightMM,
					})
					seq++
				}
			}
		}
	}
	return placements, cols, rows, allLayers, capacity, mode, nil
}

func (p *palletSequencer) snapshot() Config {
	p.mu.Lock()
	defer p.mu.Unlock()
	return p.cfg
}

func (p *palletSequencer) apply(attrs map[string]interface{}) (Config, error) {
	raw, err := json.Marshal(attrs)
	if err != nil {
		return Config{}, fmt.Errorf("marshal attributes: %w", err)
	}
	// Decode onto a copy of the current cfg rather than a zero-value
	// struct, so a partial attrs payload (e.g. webapp form Save that
	// only emits a subset of fields) preserves anything not present in
	// the payload — pallet_home, place_orientation, place_approach_*,
	// etc. Without this merge, every Save wipes those to defaults.
	p.mu.Lock()
	next := p.cfg
	p.mu.Unlock()

	dec := json.NewDecoder(bytes.NewReader(raw))
	dec.DisallowUnknownFields()
	if err := dec.Decode(&next); err != nil {
		return Config{}, fmt.Errorf("unknown or malformed attribute: %w", err)
	}
	p.mu.Lock()
	p.cfg = next
	p.mu.Unlock()
	return next, nil
}

// DoCommand surface:
//
//	Configuration (used by the webapp):
//	{"get_attributes": true}                      → {"attributes": {...}}
//	{"set_attributes": {...}}                     → {"attributes": {...applied...}}
//	{"get_pack_order": true}                      → {"placements": [...], ...}
//
//	Palletizer-facing sequencing:
//	{"next_box": true}                            → {seq, col, row, layer,
//	                                                 pose_in_pallet, approach_offset_in_pallet,
//	                                                 box_dimensions_mm, is_complete,
//	                                                 total, placed, failed, skipped, remaining}
//	{"report_placement": {"seq":N, "success":bool, "error"?:"…"}}
//	                                              → {acknowledged, next_seq, placed,
//	                                                 failed, skipped, remaining, complete}
//	{"skip_box": {"seq":N, "reason"?:"…"}}        → {skipped, next_seq, placed, remaining}
//	{"reset_cursor": true}                        → {reset:true, next_seq:1}
//	{"get_progress": true}                        → {next_seq, done_seqs, skipped_seqs,
//	                                                 failed_seqs, placed_count, failed_count,
//	                                                 skipped_count, total, complete}
// Defaults applied when the operator hasn't set explicit values in the
// pack-sequencer service attributes. Named so doGetPackOrder reads
// top-down without buried magic numbers.
const (
	// defaultPlaceApproachAngleDeg is the X-axis tilt of the diagonal
	// approach when place_approach_angle_deg is unset. 15° empirically
	// clears flush-packed neighbours without making place_start far
	// enough out that workspace reach becomes a problem.
	defaultPlaceApproachAngleDeg = 15

	// defaultPlaceApproachHeightMM is the vertical component of the
	// approach offset when place_approach_height_mm is unset. 100 mm
	// is enough to clear most boxes; the safety floor below clamps it
	// up further when box_height_mm makes 100 inadequate.
	defaultPlaceApproachHeightMM = 100

	// placeApproachSafetyPadMM is the minimum gap between the descending
	// held-box bottom at place_start and a same-layer neighbour top.
	// Without this, an aggressive operator-set approach height plus a
	// tall box would let the diagonal approach plow through neighbours.
	placeApproachSafetyPadMM = 10
)

// cmdHandler is the signature every DoCommand verb implements. Each
// handler receives the full cmd map so verbs that take a value
// (set_attributes, report_placement, skip_box, set_box_transform,
// clear_box_transform) can pull their own argument.
type cmdHandler func(p *palletSequencer, cmd map[string]interface{}) (map[string]interface{}, error)

// cmdHandlers is the canonical inventory of supported DoCommand verbs.
// Order is the dispatch order when a caller sends multiple keys at
// once — first matching key wins, mirroring the prior if-chain
// semantics. Adding a verb is one append.
var cmdHandlers = []struct {
	key     string
	handler cmdHandler
}{
	{"get_attributes", func(p *palletSequencer, _ map[string]interface{}) (map[string]interface{}, error) {
		return p.attrsResponse(p.snapshot()), nil
	}},
	{"set_attributes", func(p *palletSequencer, cmd map[string]interface{}) (map[string]interface{}, error) {
		attrs, err := coerceAttrs(cmd["set_attributes"])
		if err != nil {
			return nil, err
		}
		applied, err := p.apply(attrs)
		if err != nil {
			return nil, err
		}
		return p.attrsResponse(applied), nil
	}},
	{"get_pallet_home", func(p *palletSequencer, _ map[string]interface{}) (map[string]interface{}, error) {
		c := p.snapshot()
		local, world := p.palletHomeLocalAndWorld(c)
		return map[string]interface{}{
			"pallet_home_local": pose6DToMap(local),
			"pallet_home_world": pose6DToMap(world),
		}, nil
	}},
	{"get_box_dims", func(p *palletSequencer, _ map[string]interface{}) (map[string]interface{}, error) {
		// Single source of truth for box dimensions. The palletizer
		// pulls these at construction so it doesn't carry duplicate
		// box_length/width/height_mm fields that can drift.
		c := p.snapshot()
		return map[string]interface{}{
			"box_length_mm": c.BoxLengthMM,
			"box_width_mm":  c.BoxWidthMM,
			"box_height_mm": c.BoxHeightMM,
		}, nil
	}},
	{"get_pack_order", func(p *palletSequencer, _ map[string]interface{}) (map[string]interface{}, error) {
		return p.doGetPackOrder(), nil
	}},
	{"next_box", func(p *palletSequencer, _ map[string]interface{}) (map[string]interface{}, error) {
		return p.doNextBox()
	}},
	{"report_placement", func(p *palletSequencer, cmd map[string]interface{}) (map[string]interface{}, error) {
		return p.doReportPlacement(cmd["report_placement"])
	}},
	{"skip_box", func(p *palletSequencer, cmd map[string]interface{}) (map[string]interface{}, error) {
		return p.doSkipBox(cmd["skip_box"])
	}},
	{"reset_cursor", func(p *palletSequencer, _ map[string]interface{}) (map[string]interface{}, error) {
		return p.doResetCursor()
	}},
	{"get_progress", func(p *palletSequencer, _ map[string]interface{}) (map[string]interface{}, error) {
		return p.doGetProgress(), nil
	}},
	{"set_box_transform", func(p *palletSequencer, cmd map[string]interface{}) (map[string]interface{}, error) {
		return p.doSetBoxTransform(cmd["set_box_transform"])
	}},
	{"clear_box_transform", func(p *palletSequencer, cmd map[string]interface{}) (map[string]interface{}, error) {
		return p.doClearBoxTransform(cmd["clear_box_transform"])
	}},
}

func (p *palletSequencer) DoCommand(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	p.logger.Debugf("DoCommand received: %v", cmd)
	for _, e := range cmdHandlers {
		if _, ok := cmd[e.key]; ok {
			return e.handler(p, cmd)
		}
	}
	return nil, fmt.Errorf("unknown command: keys=%v raw=%v", keysOf(cmd), cmd)
}

// doGetPackOrder builds the enriched pack-order response: the full
// placement list with each entry's pose_in_pallet, pose_in_world, and
// approach_offset_in_pallet pre-computed. Without these, the
// palletizer's doVerifyPallet falls back to zero approach offset and
// reports place_start == place_end.
func (p *palletSequencer) doGetPackOrder() map[string]interface{} {
	c := p.snapshot()
	placements, cols, rows, layers, capacity, mode, warnings := packOrder(c, p.palletWidthMM, p.palletLengthMM)
	overflow := 0
	if c.Quantity > capacity {
		overflow = c.Quantity - capacity
	}

	angleXDeg := c.PlaceApproachAngleDeg
	if angleXDeg <= 0 {
		angleXDeg = defaultPlaceApproachAngleDeg
	}
	angleYDeg := c.PlaceApproachAngleYDeg
	height := c.PlaceApproachHeightMM
	if height <= 0 {
		height = defaultPlaceApproachHeightMM
	}
	// Floor the approach height so the descending held box can't plow
	// through a same-layer neighbour: held-box bottom at place_start =
	// layer*box_h + height, neighbour top = layer*box_h + box_h. Need
	// height ≥ box_h + safety pad.
	if minHeight := c.BoxHeightMM + placeApproachSafetyPadMM; height < minHeight {
		height = minHeight
	}
	offsetX := height * math.Tan(angleXDeg*math.Pi/180)
	offsetY := height * math.Tan(angleYDeg*math.Pi/180)

	enriched := make([]map[string]interface{}, 0, len(placements))
	for _, pl := range placements {
		ori := orientationForPlacement(c, pl)
		localPose := spatialmath.NewPose(
			r3.Vector{X: pl.XMM, Y: pl.YMM, Z: pl.ZMM},
			&spatialmath.OrientationVectorDegrees{OX: ori.OX, OY: ori.OY, OZ: ori.OZ, Theta: ori.Theta},
		)
		worldPose := poseToPose6D(spatialmath.Compose(p.palletPose, localPose))
		enriched = append(enriched, map[string]interface{}{
			"seq":       pl.Seq,
			"col":       pl.Col,
			"row":       pl.Row,
			"layer":     pl.Layer,
			"x_mm":      pl.XMM,
			"y_mm":      pl.YMM,
			"z_mm":      pl.ZMM,
			"label":     pl.Label,
			"length_mm": pl.Length,
			"width_mm":  pl.Width,
			"height_mm": pl.Height,
			"pose_in_pallet": map[string]interface{}{
				"x":     pl.XMM,
				"y":     pl.YMM,
				"z":     pl.ZMM,
				"o_x":   ori.OX,
				"o_y":   ori.OY,
				"o_z":   ori.OZ,
				"theta": ori.Theta,
			},
			// Pre-composed world-frame pose so consumers don't need to
			// know pallet_origin themselves. Single source of truth —
			// eliminates the duplicated pallet_origin field the
			// palletizer used to carry.
			"pose_in_world": pose6DToMap(worldPose),
			"approach_offset_in_pallet": map[string]interface{}{
				"x": offsetX,
				"y": offsetY,
				"z": height,
			},
		})
	}

	return map[string]interface{}{
		"placements":          enriched,
		"cols":                cols,
		"rows":                rows,
		"layers":              layers,
		"capacity":            capacity,
		"quantity":            c.Quantity,
		"overflow":            overflow,
		"mode":                mode,
		"warnings":            warnings,
		"pallet_thickness_mm": p.palletThicknessMM,
		"pallet_width_mm":     p.palletWidthMM,
		"pallet_length_mm":    p.palletLengthMM,
		"pallet_pose":         pose6DToMap(poseToPose6D(p.palletPose)),
	}
}

// doResetCursor zeroes the placement cursor + dynamic-transform map
// and emits REMOVED events for every UUID the renderer might have
// cached (both confirmed-placed seqs and in-flight dynamic transforms).
// dynamicVersion is intentionally NOT reset so future set_box_transform
// calls mint UUIDs that differ from any prior ones the renderer
// remembers.
func (p *palletSequencer) doResetCursor() (map[string]interface{}, error) {
	p.mu.Lock()
	removedUUIDs := make([][]byte, 0, len(p.cursor.done)+len(p.dynamicBoxes))
	for _, tf := range p.dynamicBoxes {
		removedUUIDs = append(removedUUIDs, append([]byte(nil), tf.Uuid...))
	}
	dynamicSeqs := map[int]bool{}
	for seq := range p.dynamicBoxes {
		dynamicSeqs[seq] = true
	}
	for seq, status := range p.cursor.done {
		if status != "success" || dynamicSeqs[seq] {
			continue
		}
		removedUUIDs = append(removedUUIDs, []byte(fmt.Sprintf("box-%d", seq)))
	}
	p.cursor = newCursor()
	p.dynamicBoxes = map[int]*commonpb.Transform{}
	p.mu.Unlock()

	for _, u := range removedUUIDs {
		p.emit(worldstatestore.TransformChange{
			ChangeType: wsspb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED,
			Transform:  &commonpb.Transform{Uuid: u},
		})
	}
	return map[string]interface{}{"reset": true, "next_seq": 1}, nil
}

// doSetBoxTransform inserts or updates a caller-supplied Transform for
// one box seq. Used to render a box at the pickup location, attach it
// to the gripper during transit, etc. The seq doesn't need to be in the
// cursor yet — anything the palletizer wants to visualize.
//
//	{"set_box_transform": {
//	    "seq":    7,
//	    "parent": "fake-gripper",            // any frame name; "world" by default
//	    "x": 0, "y": 0, "z": 30,             // pose in parent frame, mm
//	    "o_x": 0, "o_y": 0, "o_z": 1, "theta": 0
//	}}
//
// Box dimensions come from cfg.box_*_mm.
func (p *palletSequencer) doSetBoxTransform(raw interface{}) (map[string]interface{}, error) {
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("set_box_transform: expected object, got %T", raw)
	}
	seqF, ok := args["seq"].(float64)
	if !ok {
		return nil, fmt.Errorf("set_box_transform: missing or non-numeric 'seq'")
	}
	seq := int(seqF)
	parent, _ := args["parent"].(string)
	if parent == "" {
		parent = p.observerFrame
	}
	x, _ := args["x"].(float64)
	y, _ := args["y"].(float64)
	z, _ := args["z"].(float64)
	ox, _ := args["o_x"].(float64)
	oy, _ := args["o_y"].(float64)
	oz, _ := args["o_z"].(float64)
	if ox == 0 && oy == 0 && oz == 0 {
		oz = 1 // default OV: straight up
	}
	theta, _ := args["theta"].(float64)

	p.mu.Lock()
	p.dynamicVersion[seq]++
	version := p.dynamicVersion[seq]
	prev := p.dynamicBoxes[seq]
	uuid := fmt.Sprintf("box-%d-v%d", seq, version)
	tf := &commonpb.Transform{
		Uuid:           []byte(uuid),
		ReferenceFrame: uuid,
		PoseInObserverFrame: &commonpb.PoseInFrame{
			ReferenceFrame: parent,
			Pose: &commonpb.Pose{
				X: x, Y: y, Z: z,
				OX: ox, OY: oy, OZ: oz, Theta: theta,
			},
		},
		PhysicalObject: &commonpb.Geometry{
			GeometryType: &commonpb.Geometry_Box{
				Box: &commonpb.RectangularPrism{
					DimsMm: &commonpb.Vector3{
						X: p.cfg.BoxWidthMM,
						Y: p.cfg.BoxLengthMM,
						Z: p.cfg.BoxHeightMM,
					},
				},
			},
			Label: uuid,
		},
	}
	p.dynamicBoxes[seq] = tf
	p.mu.Unlock()

	// Mint a fresh UUID per call and emit REMOVED for the prior UUID +
	// ADDED for the new one. Same UUID would be silently dropped by the
	// renderer (ADDED for an existing UUID is a no-op); a brand-new
	// UUID after a REMOVED is treated as a fresh transform and rendered.
	if prev != nil {
		p.emit(worldstatestore.TransformChange{
			ChangeType: wsspb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED,
			Transform:  &commonpb.Transform{Uuid: prev.Uuid},
		})
	}
	p.emit(worldstatestore.TransformChange{
		ChangeType: wsspb.TransformChangeType_TRANSFORM_CHANGE_TYPE_ADDED,
		Transform:  tf,
	})
	return map[string]interface{}{"acknowledged": true, "seq": seq, "uuid": uuid, "parent": parent}, nil
}

// doClearBoxTransform removes a dynamic Transform for one seq. If the
// seq is also a successful placement, the canonical world-pose-on-pallet
// rendering takes over and we emit UPDATED. Otherwise it disappears
// (REMOVED).
func (p *palletSequencer) doClearBoxTransform(raw interface{}) (map[string]interface{}, error) {
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("clear_box_transform: expected object, got %T", raw)
	}
	seqF, ok := args["seq"].(float64)
	if !ok {
		return nil, fmt.Errorf("clear_box_transform: missing or non-numeric 'seq'")
	}
	seq := int(seqF)

	p.mu.Lock()
	prev := p.dynamicBoxes[seq]
	delete(p.dynamicBoxes, seq)
	stillPlaced := p.cursor.done[seq] == "success"
	var placedTf *commonpb.Transform
	if stillPlaced {
		placedTf = p.buildTransformForSeqLocked(seq)
	}
	p.mu.Unlock()

	if prev == nil {
		return map[string]interface{}{"acknowledged": true, "seq": seq, "noop": true}, nil
	}
	p.emit(worldstatestore.TransformChange{
		ChangeType: wsspb.TransformChangeType_TRANSFORM_CHANGE_TYPE_REMOVED,
		Transform:  &commonpb.Transform{Uuid: prev.Uuid},
	})
	if stillPlaced && placedTf != nil {
		p.emit(worldstatestore.TransformChange{
			ChangeType: wsspb.TransformChangeType_TRANSFORM_CHANGE_TYPE_ADDED,
			Transform:  placedTf,
		})
	}
	return map[string]interface{}{"acknowledged": true, "seq": seq}, nil
}

// doGetProgress returns the current cursor state: which seqs have been
// placed (success), skipped, failed (pending retry), and the total.
// Intended for the webapp to filter the 3D view so confirmed-placed boxes
// render solid while pending ones render as ghost outlines.
func (p *palletSequencer) doGetProgress() map[string]interface{} {
	p.mu.Lock()
	cur := p.cursor
	c := p.cfg
	p.mu.Unlock()

	placed, failed, skipped := cur.counts()

	successSeqs := make([]int, 0, placed)
	skippedSeqs := make([]int, 0, skipped)
	for seq, status := range cur.done {
		switch status {
		case "success":
			successSeqs = append(successSeqs, seq)
		case "skipped":
			skippedSeqs = append(skippedSeqs, seq)
		}
	}
	failedSeqs := make([]int, 0, len(cur.err))
	for seq := range cur.err {
		// failed = in err but not yet successfully placed or skipped
		if _, isDone := cur.done[seq]; !isDone {
			failedSeqs = append(failedSeqs, seq)
		}
	}
	sort.Ints(successSeqs)
	sort.Ints(skippedSeqs)
	sort.Ints(failedSeqs)

	placements, _, _, _, _, _, _ := packOrder(c, p.palletWidthMM, p.palletLengthMM)
	total := len(placements)

	return map[string]interface{}{
		"next_seq":      cur.next,
		"done_seqs":     successSeqs,
		"skipped_seqs":  skippedSeqs,
		"failed_seqs":   failedSeqs,
		"placed_count":  placed,
		"failed_count":  failed,
		"skipped_count": skipped,
		"total":         total,
		"complete":      placed+skipped == total && total > 0,
	}
}

// doNextBox returns the next place-pose for the palletizer to execute, or
// is_complete=true when the cursor has walked past every placement. The
// returned pose is in the pallet frame (origin at the bottom-left corner of
// the pallet area, +X right, +Y receding into the scene, +Z up); the
// palletizer composes its own pallet_origin to get a world-frame pose.
func (p *palletSequencer) doNextBox() (map[string]interface{}, error) {
	p.mu.Lock()
	c := p.cfg
	cur := p.cursor
	p.mu.Unlock()

	placements, _, _, _, _, _, _ := packOrder(c, p.palletWidthMM, p.palletLengthMM)
	total := len(placements)

	// Find first placement at-or-after cursor.next that isn't already done.
	var next *Placement
	for i := range placements {
		if placements[i].Seq < cur.next {
			continue
		}
		if _, isDone := cur.done[placements[i].Seq]; isDone {
			continue
		}
		next = &placements[i]
		break
	}
	placed, failed, skipped := cur.counts()
	remaining := total - placed - skipped

	if next == nil {
		return map[string]interface{}{
			"is_complete": true,
			"total":       total,
			"placed":      placed,
			"failed":      failed,
			"skipped":     skipped,
			"remaining":   remaining,
		}, nil
	}

	// Place orientation derives from pallet_home when set — pallet_home
	// is the single source of truth for the gripper's pose at place time.
	// Place_orientation kept as a back-compat fallback for configs that
	// don't define pallet_home yet. orientationForPlacement also adds a
	// 90° yaw when the placement's footprint is rotated relative to the
	// box's default (e.g. interlock alternate layers).
	ori := orientationForPlacement(c, *next)

	// Approach offset in pallet frame: h above the slot, tilted by the
	// X and Y angles so the descent happens diagonally rather than
	// straight down one edge. X defaults to 15°, Y defaults to 0°
	// (back-compat with the single-axis approach). Setting both gives
	// a corner approach, which slides less against already-placed
	// neighbors.
	angleXDeg := c.PlaceApproachAngleDeg
	if angleXDeg <= 0 {
		angleXDeg = 15
	}
	angleYDeg := c.PlaceApproachAngleYDeg
	height := c.PlaceApproachHeightMM
	if height <= 0 {
		height = 100
	}
	// Same floor as packOrder above: held box bottom at place_start =
	// layer*box_h + height, while same-layer neighbour top = layer*box_h
	// + box_h. height < box_h means the descending box plows through the
	// neighbour. Treat the user value as a minimum and clamp up to
	// box_height + 10 mm of safety pad.
	if minHeight := c.BoxHeightMM + 10; height < minHeight {
		height = minHeight
	}
	offsetX := height * math.Tan(angleXDeg*math.Pi/180)
	offsetY := height * math.Tan(angleYDeg*math.Pi/180)

	// Pre-composed world-frame poses so the palletizer doesn't need a
	// duplicate pallet_origin. start_in_world includes the approach
	// offset so the palletizer can move directly to the world poses.
	endLocal := spatialmath.NewPose(
		r3.Vector{X: next.XMM, Y: next.YMM, Z: next.ZMM},
		&spatialmath.OrientationVectorDegrees{OX: ori.OX, OY: ori.OY, OZ: ori.OZ, Theta: ori.Theta},
	)
	startLocal := spatialmath.NewPose(
		r3.Vector{X: next.XMM + offsetX, Y: next.YMM + offsetY, Z: next.ZMM + height},
		&spatialmath.OrientationVectorDegrees{OX: ori.OX, OY: ori.OY, OZ: ori.OZ, Theta: ori.Theta},
	)
	endWorld := poseToPose6D(spatialmath.Compose(p.palletPose, endLocal))
	startWorld := poseToPose6D(spatialmath.Compose(p.palletPose, startLocal))

	return map[string]interface{}{
		"seq":   next.Seq,
		"col":   next.Col,
		"row":   next.Row,
		"layer": next.Layer,
		"pose_in_pallet": map[string]interface{}{
			"x":     next.XMM,
			"y":     next.YMM,
			"z":     next.ZMM,
			"o_x":   ori.OX,
			"o_y":   ori.OY,
			"o_z":   ori.OZ,
			"theta": ori.Theta,
		},
		"approach_offset_in_pallet": map[string]interface{}{
			"x": offsetX, "y": offsetY, "z": height,
		},
		"place_end_in_world":   pose6DToMap(endWorld),
		"place_start_in_world": pose6DToMap(startWorld),
		"box_dimensions_mm": map[string]interface{}{
			"width":  next.Width,
			"length": next.Length,
			"height": next.Height,
		},
		"is_complete": false,
		"total":       total,
		"placed":      placed,
		"failed":      failed,
		"skipped":     skipped,
		"remaining":   remaining,
	}, nil
}

// doReportPlacement records the palletizer's outcome for a seq. success
// advances the cursor (and clears any prior failure for that seq). failure
// leaves the cursor put so next_box returns the same seq for retry; the
// error message is stored for later inspection.
func (p *palletSequencer) doReportPlacement(raw interface{}) (map[string]interface{}, error) {
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("report_placement: expected object, got %T", raw)
	}
	seqF, ok := args["seq"].(float64)
	if !ok {
		return nil, fmt.Errorf("report_placement: missing or non-numeric 'seq'")
	}
	seq := int(seqF)
	success, _ := args["success"].(bool)
	errMsg, _ := args["error"].(string)

	p.mu.Lock()
	var pendingEmits []worldstatestore.TransformChange
	// Push emits before unlock so LIFO-ordered defers fire them *after*
	// the mutex is released — keeps the lock scope narrow even though
	// emit() is already non-blocking.
	defer func() {
		for _, c := range pendingEmits {
			p.emit(c)
		}
	}()
	defer p.mu.Unlock()

	if success {
		_, alreadyDone := p.cursor.done[seq]
		hadDynamic := p.dynamicBoxes[seq] != nil
		p.cursor.done[seq] = "success"
		delete(p.cursor.err, seq)
		if seq == p.cursor.next {
			// Walk forward past any already-done seqs.
			for {
				_, isDone := p.cursor.done[p.cursor.next]
				if !isDone {
					break
				}
				p.cursor.next++
			}
		}
		// If a caller-set dynamic Transform is already in place, leave
		// it alone — emitDropoffVisual (or whoever updated it last) put
		// the box where it belongs, and re-emitting under the canonical
		// "box-N" UUID would just cause a brief flicker. We only need
		// to materialize the canonical placement when there's no
		// dynamic Transform to inherit, e.g. a caller using the API
		// without the lifecycle helpers.
		if !alreadyDone && !hadDynamic {
			placedTf := p.buildTransformForSeqLocked(seq)
			if placedTf != nil {
				pendingEmits = append(pendingEmits, worldstatestore.TransformChange{
					ChangeType: wsspb.TransformChangeType_TRANSFORM_CHANGE_TYPE_ADDED,
					Transform:  placedTf,
				})
			}
		}
	} else {
		p.cursor.err[seq] = errMsg
		// Cursor stays put so next_box will return the same seq; don't add to
		// done so it isn't counted as final.
	}

	placed, failed, skipped := p.cursor.counts()
	c := p.cfg
	placements, _, _, _, _, _, _ := packOrder(c, p.palletWidthMM, p.palletLengthMM)
	total := len(placements)
	remaining := total - placed - skipped
	complete := remaining == 0 && total > 0

	return map[string]interface{}{
		"acknowledged": true,
		"next_seq":     p.cursor.next,
		"placed":       placed,
		"failed":       failed,
		"skipped":      skipped,
		"remaining":    remaining,
		"complete":     complete,
		"last_error":   errMsg,
	}, nil
}

// doSkipBox marks a seq as skipped (not counted toward placed, but also not
// retried). Useful for manual recovery after repeated failures.
func (p *palletSequencer) doSkipBox(raw interface{}) (map[string]interface{}, error) {
	args, ok := raw.(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("skip_box: expected object, got %T", raw)
	}
	seqF, ok := args["seq"].(float64)
	if !ok {
		return nil, fmt.Errorf("skip_box: missing or non-numeric 'seq'")
	}
	seq := int(seqF)
	reason, _ := args["reason"].(string)

	p.mu.Lock()
	defer p.mu.Unlock()

	p.cursor.done[seq] = "skipped"
	if reason != "" {
		p.cursor.err[seq] = reason
	}
	if seq == p.cursor.next {
		for {
			_, isDone := p.cursor.done[p.cursor.next]
			if !isDone {
				break
			}
			p.cursor.next++
		}
	}
	placed, _, skipped := p.cursor.counts()
	c := p.cfg
	placements, _, _, _, _, _, _ := packOrder(c, p.palletWidthMM, p.palletLengthMM)
	total := len(placements)
	remaining := total - placed - skipped

	return map[string]interface{}{
		"skipped":   seq,
		"next_seq":  p.cursor.next,
		"placed":    placed,
		"remaining": remaining,
	}, nil
}

// coerceAttrs accepts either a nested object (preferred) or a JSON-encoded
// string (fallback when the client can't ship nested Structs).
func coerceAttrs(raw interface{}) (map[string]interface{}, error) {
	switch v := raw.(type) {
	case map[string]interface{}:
		return v, nil
	case string:
		var out map[string]interface{}
		if err := json.Unmarshal([]byte(v), &out); err != nil {
			return nil, fmt.Errorf("set_attributes: JSON string parse failed: %w", err)
		}
		return out, nil
	default:
		return nil, fmt.Errorf("set_attributes expects an object or JSON string, got %T", raw)
	}
}

func (p *palletSequencer) attrsResponse(c Config) map[string]interface{} {
	raw, _ := json.Marshal(c)
	var m map[string]interface{}
	_ = json.Unmarshal(raw, &m)
	return map[string]interface{}{"attributes": m}
}

// placeOrientation returns the gripper orientation (in pallet-local frame)
// the palletizer should use at place_start and place_end. Pallet_home is
// the canonical source — using its orientation guarantees place_start/end
// share the same world yaw as pallet_home, so the wrist doesn't spin
// during the place. Place_orientation is kept as a fallback for older
// configs that haven't migrated to pallet_home; default is gripper
// straight down.
func placeOrientation(c Config) Pose6D {
	if c.PalletHome != nil {
		out := Pose6D{
			OX: c.PalletHome.OX, OY: c.PalletHome.OY,
			OZ: c.PalletHome.OZ, Theta: c.PalletHome.Theta,
		}
		// Pose6D zero-value has all-zero OV which is invalid; if the
		// caller didn't set OZ (partial config), fall back to
		// straight-down so the gripper has a sane orientation.
		if out.OX == 0 && out.OY == 0 && out.OZ == 0 {
			out.OZ = -1
		}
		return out
	}
	if c.PlaceOrientation != nil {
		return *c.PlaceOrientation
	}
	return Pose6D{OX: 0, OY: 0, OZ: -1, Theta: 0}
}

// orientationForPlacement returns the gripper orientation for one
// placement, applying a 90° yaw rotation when the placement's footprint
// is rotated relative to the box's default (width/length swapped). The
// interlock pack mode rotates alternate layers — without this rotation
// the gripper grips the box edge-on instead of along its long axis.
func orientationForPlacement(c Config, pl Placement) Pose6D {
	ori := placeOrientation(c)
	// Tolerance handles float comparison; box dims are typically mm
	// integers so 0.5 is generous.
	rotated := math.Abs(pl.Width-c.BoxLengthMM) <= 0.5 && math.Abs(pl.Length-c.BoxWidthMM) <= 0.5
	if rotated {
		ori.Theta += 90
	}
	return ori
}

// palletHomeLocalAndWorld resolves pallet_home to concrete Pose6D values
// in both pallet-local and world frames. Local fills in defaults (far
// corner at Z=200, theta=0, straight-down gripper) for any missing
// pieces — width/length default from the pallet component's dimensions.
// World composes local with the pallet component's world pose.
func (p *palletSequencer) palletHomeLocalAndWorld(c Config) (Pose6D, Pose6D) {
	var local Pose6D
	if c.PalletHome != nil {
		local = *c.PalletHome
	}
	if local.X == 0 && local.Y == 0 && local.Z == 0 &&
		(c.PalletHome == nil || (c.PalletHome.X == 0 && c.PalletHome.Y == 0 && c.PalletHome.Z == 0)) {
		local.X = p.palletWidthMM
		local.Y = p.palletLengthMM
		local.Z = 200
	}
	local.OX, local.OY, local.OZ = 0, 0, -1

	localPose := spatialmath.NewPose(
		r3.Vector{X: local.X, Y: local.Y, Z: local.Z},
		&spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: -1, Theta: local.Theta},
	)
	worldPose := spatialmath.Compose(p.palletPose, localPose)
	pt := worldPose.Point()
	ori := worldPose.Orientation().OrientationVectorDegrees()
	world := Pose6D{
		X: pt.X, Y: pt.Y, Z: pt.Z,
		OX: ori.OX, OY: ori.OY, OZ: ori.OZ, Theta: ori.Theta,
	}
	return local, world
}

// Thin wrappers around the shared package. Kept as locally-named
// shorthands so call sites stay terse.
func pose6DToMap(p Pose6D) map[string]interface{} { return p.ToMap() }
func poseToPose6D(pose spatialmath.Pose) Pose6D   { return wcsh.PoseFrom(pose) }

func keysOf(m map[string]interface{}) []string {
	ks := make([]string, 0, len(m))
	for k := range m {
		ks = append(ks, k)
	}
	return ks
}

// ---------------------------------------------------------------------------
// world-state-store surface on palletSequencer
//
// Every confirmed-placed seq is exposed as a Transform with stable UUID
// "box-<seq>". The cursor mutators (doReportPlacement on success,
// reset_cursor) emit ADDED/REMOVED events inline, so subscribers see
// placements in real time without polling. pallet_origin composes with
// the pallet-frame center to produce the world-frame pose the renderer
// needs.
// ---------------------------------------------------------------------------

// ListUUIDs returns one UUID per successfully-placed seq AND every seq
// with a caller-set dynamic Transform (in-flight boxes — at pickup,
// attached to the gripper, etc.). Dynamic and placed UUIDs deduplicate
// via the seq. The pallet itself is rendered by the pallet component's
// own frame.geometry, not via a world-state-store transform.
func (p *palletSequencer) ListUUIDs(_ context.Context, _ map[string]any) ([][]byte, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	// The pallet component renders its own wooden-pallet visual via
	// frame.geometry — no need to emit a pallet-wood transform here.
	uuids := make([][]byte, 0, len(p.cursor.done)+len(p.dynamicBoxes))
	dynamicSeqs := map[int]bool{}
	for seq, tf := range p.dynamicBoxes {
		dynamicSeqs[seq] = true
		uuids = append(uuids, append([]byte(nil), tf.Uuid...))
	}
	for seq, status := range p.cursor.done {
		if status != "success" || dynamicSeqs[seq] {
			continue
		}
		uuids = append(uuids, []byte(fmt.Sprintf("box-%d", seq)))
	}
	return uuids, nil
}

// GetTransform returns the full Transform for one UUID. Dynamic UUIDs
// look like "box-<seq>-v<N>" and are stored verbatim; the canonical
// "box-<seq>" comes from the pack order for placed seqs.
func (p *palletSequencer) GetTransform(_ context.Context, uuid []byte, _ map[string]any) (*commonpb.Transform, error) {
	p.mu.Lock()
	defer p.mu.Unlock()
	for _, tf := range p.dynamicBoxes {
		if string(tf.Uuid) == string(uuid) {
			return tf, nil
		}
	}
	var seq int
	if _, err := fmt.Sscanf(string(uuid), "box-%d", &seq); err != nil {
		return nil, fmt.Errorf("unknown uuid %q", string(uuid))
	}
	if p.cursor.done[seq] != "success" {
		return nil, fmt.Errorf("seq %d not placed", seq)
	}
	tf := p.buildTransformForSeqLocked(seq)
	if tf == nil {
		return nil, fmt.Errorf("seq %d has no placement", seq)
	}
	return tf, nil
}

// StreamTransformChanges returns a live stream of ADDED/REMOVED events.
// Events are emitted inline from the cursor mutators; this just wraps
// the internal channel.
func (p *palletSequencer) StreamTransformChanges(ctx context.Context, _ map[string]any) (*worldstatestore.TransformChangeStream, error) {
	return worldstatestore.NewTransformChangeStreamFromChannel(ctx, p.changeChan), nil
}

// buildTransformForSeqLocked builds a placed-box Transform in world
// space, composing pallet_origin with the box's pallet-frame center.
// placement.z_mm is the gripper pose at the top face; subtract half the
// box height to get the center.
func (p *palletSequencer) buildTransformForSeqLocked(seq int) *commonpb.Transform {
	placements, _, _, _, _, _, _ := packOrder(p.cfg, p.palletWidthMM, p.palletLengthMM)
	var placement *Placement
	for i := range placements {
		if placements[i].Seq == seq {
			placement = &placements[i]
			break
		}
	}
	if placement == nil {
		return nil
	}

	centerInPallet := spatialmath.NewPoseFromPoint(r3.Vector{
		X: placement.XMM, Y: placement.YMM, Z: placement.ZMM - placement.Height/2,
	})
	uuid := fmt.Sprintf("box-%d", seq)
	return &commonpb.Transform{
		Uuid:           []byte(uuid),
		ReferenceFrame: uuid,
		PoseInObserverFrame: &commonpb.PoseInFrame{
			ReferenceFrame: p.observerFrame,
			Pose:           p.worldPose(centerInPallet),
		},
		PhysicalObject: &commonpb.Geometry{
			GeometryType: &commonpb.Geometry_Box{
				Box: &commonpb.RectangularPrism{
					DimsMm: &commonpb.Vector3{X: placement.Width, Y: placement.Length, Z: placement.Height},
				},
			},
			Label: uuid,
		},
	}
}

// worldPose composes a pallet-local pose with the bound pallet
// component's world pose, returning a protobuf Pose in world space.
// p.palletPose is set at construction from the pallet's frame; an
// AlwaysRebuild cascade refreshes it when the pallet's frame changes.
func (p *palletSequencer) worldPose(inPallet spatialmath.Pose) *commonpb.Pose {
	world := spatialmath.Compose(p.palletPose, inPallet)
	pt := world.Point()
	ov := world.Orientation().OrientationVectorDegrees()
	return &commonpb.Pose{
		X: pt.X, Y: pt.Y, Z: pt.Z,
		OX: ov.OX, OY: ov.OY, OZ: ov.OZ, Theta: ov.Theta,
	}
}

// emit drops events rather than blocking the caller — the channel is
// generously buffered and subscribers can always resync via ListUUIDs.
func (p *palletSequencer) emit(c worldstatestore.TransformChange) {
	select {
	case p.changeChan <- c:
	default:
		p.logger.Warnw("change channel full, dropping event", "uuid", string(c.Transform.GetUuid()))
	}
}

