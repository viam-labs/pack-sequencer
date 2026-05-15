package packsequencer

import (
	"math"
	"testing"

	"github.com/golang/geo/r3"
	"github.com/viam-labs/viamkit/viz"
	"go.viam.com/rdk/spatialmath"
)

const eps = 1e-6

func TestPackOrderEmptyDims(t *testing.T) {
	c := Config{PalletAreaHeightMM: 100}
	placements, _, _, _, capacity, _, _ := packOrder(c, 1000, 1000)
	if len(placements) != 0 {
		t.Errorf("zero box dims: got %d placements, want 0", len(placements))
	}
	if capacity != 0 {
		t.Errorf("zero box dims: capacity %d, want 0", capacity)
	}
}

func TestPackOrderCapacity(t *testing.T) {
	c := Config{
		BoxLengthMM:        200,
		BoxWidthMM:         100,
		BoxHeightMM:        80,
		PalletAreaHeightMM: 160, // 2 layers
	}
	placements, cols, rows, layers, capacity, _, _ := packOrder(c, 600, 400)
	if layers != 2 {
		t.Errorf("layers: got %d, want 2", layers)
	}
	if capacity != cols*rows*layers {
		t.Errorf("capacity %d != cols*rows*layers (%d * %d * %d)", capacity, cols, rows, layers)
	}
	if len(placements) != capacity {
		t.Errorf("len(placements) %d != capacity %d", len(placements), capacity)
	}
	// Seq numbering: every placement gets a unique seq starting at 1
	seen := map[int]bool{}
	for _, pl := range placements {
		if seen[pl.Seq] {
			t.Errorf("duplicate seq %d", pl.Seq)
		}
		seen[pl.Seq] = true
	}
}

func TestCoerceAttrsMapPassthrough(t *testing.T) {
	in := map[string]interface{}{"foo": 1.0, "bar": "x"}
	out, err := coerceAttrs(in)
	if err != nil {
		t.Fatal(err)
	}
	if out["foo"] != 1.0 || out["bar"] != "x" {
		t.Errorf("map passthrough: got %v", out)
	}
}

func TestCoerceAttrsJSONString(t *testing.T) {
	out, err := coerceAttrs(`{"foo": 1}`)
	if err != nil {
		t.Fatal(err)
	}
	if out["foo"].(float64) != 1.0 {
		t.Errorf("JSON string: got %v, want {foo:1}", out)
	}
}

func TestCoerceAttrsBadInput(t *testing.T) {
	if _, err := coerceAttrs(42); err == nil {
		t.Error("expected error for int input")
	}
	if _, err := coerceAttrs(`not json`); err == nil {
		t.Error("expected error for invalid JSON string")
	}
}

func TestPoseRoundTrip(t *testing.T) {
	original := spatialmath.NewPose(
		r3.Vector{X: 100, Y: 200, Z: 300},
		&spatialmath.OrientationVectorDegrees{OX: 0, OY: 0, OZ: -1, Theta: 45},
	)
	p6 := poseToPose6D(original)
	if math.Abs(p6.X-100) > eps || math.Abs(p6.Y-200) > eps || math.Abs(p6.Z-300) > eps {
		t.Errorf("poseToPose6D translation: %+v", p6)
	}
	if math.Abs(p6.OZ-(-1)) > eps || math.Abs(p6.Theta-45) > eps {
		t.Errorf("poseToPose6D orientation: OZ=%v Theta=%v", p6.OZ, p6.Theta)
	}

	m := pose6DToMap(p6)
	if math.Abs(m["x"].(float64)-100) > eps {
		t.Errorf("pose6DToMap x: %v", m["x"])
	}
	if math.Abs(m["o_z"].(float64)-(-1)) > eps {
		t.Errorf("pose6DToMap o_z: %v", m["o_z"])
	}
	if math.Abs(m["theta"].(float64)-45) > eps {
		t.Errorf("pose6DToMap theta: %v", m["theta"])
	}
}

func TestKeysOf(t *testing.T) {
	keys := keysOf(map[string]interface{}{"a": 1, "b": 2, "c": 3})
	if len(keys) != 3 {
		t.Errorf("got %d keys, want 3", len(keys))
	}
	seen := map[string]bool{}
	for _, k := range keys {
		seen[k] = true
	}
	for _, k := range []string{"a", "b", "c"} {
		if !seen[k] {
			t.Errorf("missing key %q in %v", k, keys)
		}
	}
}

func TestOrientationForPlacementUnrotated(t *testing.T) {
	c := Config{
		BoxLengthMM: 200, BoxWidthMM: 100, BoxHeightMM: 80,
		PalletHome: &Pose6D{OX: 0, OY: 0, OZ: -1, Theta: 0},
	}
	pl := Placement{Length: 200, Width: 100, Height: 80}
	ori := orientationForPlacement(c, pl)
	if math.Abs(ori.Theta) > eps {
		t.Errorf("unrotated theta: got %v, want 0", ori.Theta)
	}
}

func TestOrientationForPlacementRotated(t *testing.T) {
	// Box dims swapped (interlock alternate-layer): theta += 90
	c := Config{
		BoxLengthMM: 200, BoxWidthMM: 100, BoxHeightMM: 80,
		PalletHome: &Pose6D{OX: 0, OY: 0, OZ: -1, Theta: 0},
	}
	pl := Placement{Length: 100, Width: 200, Height: 80}
	ori := orientationForPlacement(c, pl)
	if math.Abs(ori.Theta-90) > eps {
		t.Errorf("rotated theta: got %v, want 90", ori.Theta)
	}
}

func TestRejectUnknownAttributes(t *testing.T) {
	// Canonical key — accepted.
	if err := rejectUnknownAttributes(map[string]interface{}{
		"box_width_mm": 100.0,
	}); err != nil {
		t.Errorf("canonical key rejected: %v", err)
	}
	// Typo (no _mm suffix) — rejected. This is the exact case the
	// dryrun hit silently before the strict check landed.
	if err := rejectUnknownAttributes(map[string]interface{}{
		"box_width": 100.0,
	}); err == nil {
		t.Errorf("typo'd key 'box_width' should have been rejected, got nil")
	}
	// Nested-object shape (matches next_box response) — rejected as
	// unknown. Another exact dryrun bug.
	if err := rejectUnknownAttributes(map[string]interface{}{
		"box_dimensions_mm": map[string]interface{}{"width": 100, "length": 200},
	}); err == nil {
		t.Errorf("'box_dimensions_mm' nested shape should have been rejected, got nil")
	}
}

func TestVizColorDefaultIsCardboardBrown(t *testing.T) {
	p := &palletSequencer{cfg: Config{}}
	c := p.vizColor()
	if c.R != 176 || c.G != 136 || c.B != 80 {
		t.Errorf("default box color: got (%d,%d,%d), want cardboard brown (176,136,80)",
			c.R, c.G, c.B)
	}
	if math.Abs(c.Opacity-1) > eps {
		t.Errorf("default opacity: got %v, want 1", c.Opacity)
	}
}

func TestVizColorConfigOverride(t *testing.T) {
	p := &palletSequencer{cfg: Config{
		BoxColor: &BoxColor{R: 10, G: 20, B: 30, Opacity: 0.5},
	}}
	c := p.vizColor()
	if c.R != 10 || c.G != 20 || c.B != 30 || math.Abs(c.Opacity-0.5) > eps {
		t.Errorf("override: got (%d,%d,%d,opacity=%v), want (10,20,30,0.5)",
			c.R, c.G, c.B, c.Opacity)
	}
}

// parsePoseArgs is the silent-zero fix from the 2026-05-15 dryrun.
// The dryrun-claude sent {"pose": {x,y,z,...}} (nested) and
// pack-sequencer 0.2.0 read flat-only — every set_box_transform pose
// landed at world origin instead of the supplied coords.

func TestParsePoseArgs_NestedForm(t *testing.T) {
	args := map[string]interface{}{
		"seq":    1.0,
		"parent": "world",
		"pose": map[string]interface{}{
			"x": 100.0, "y": 200.0, "z": 300.0,
			"o_z": -1.0, "theta": 90.0,
		},
	}
	x, y, z, ox, oy, oz, theta := parsePoseArgs(args)
	if x != 100 || y != 200 || z != 300 || ox != 0 || oy != 0 || oz != -1 || theta != 90 {
		t.Errorf("nested: got (%v,%v,%v) ov=(%v,%v,%v) theta=%v, want (100,200,300) ov=(0,0,-1) theta=90",
			x, y, z, ox, oy, oz, theta)
	}
}

func TestParsePoseArgs_FlatForm(t *testing.T) {
	args := map[string]interface{}{
		"seq":    1.0,
		"x":      111.0, "y": 222.0, "z": 333.0,
		"o_z":   -1.0, "theta": 45.0,
	}
	x, y, z, _, _, oz, theta := parsePoseArgs(args)
	if x != 111 || y != 222 || z != 333 || oz != -1 || theta != 45 {
		t.Errorf("flat: got (%v,%v,%v) oz=%v theta=%v, want (111,222,333) oz=-1 theta=45",
			x, y, z, oz, theta)
	}
}

func TestParsePoseArgs_NestedWinsOverFlat(t *testing.T) {
	args := map[string]interface{}{
		"x": 999.0, "y": 999.0, "z": 999.0,
		"pose": map[string]interface{}{"x": 1.0, "y": 2.0, "z": 3.0, "o_z": 1.0},
	}
	x, y, z, _, _, _, _ := parsePoseArgs(args)
	if x != 1 || y != 2 || z != 3 {
		t.Errorf("nested+flat: got (%v,%v,%v), want (1,2,3) (nested wins)", x, y, z)
	}
}

func TestParsePoseArgs_Empty(t *testing.T) {
	x, y, z, ox, oy, oz, theta := parsePoseArgs(map[string]interface{}{"seq": 1.0})
	if x != 0 || y != 0 || z != 0 || ox != 0 || oy != 0 || oz != 0 || theta != 0 {
		t.Errorf("empty: got (%v,%v,%v) ov=(%v,%v,%v) theta=%v, want all zeros", x, y, z, ox, oy, oz, theta)
	}
}

func TestParseColorArg_EmptyFallsBack(t *testing.T) {
	fallback := viz.Color{R: 1, G: 2, B: 3, Opacity: 0.5}
	got := parseColorArg(map[string]interface{}{}, fallback)
	if got != fallback {
		t.Errorf("empty: got %+v, want fallback %+v", got, fallback)
	}
}

func TestParseColorArg_OutOfRangeFallsBack(t *testing.T) {
	fallback := viz.Color{R: 1, G: 2, B: 3, Opacity: 0.5}
	got := parseColorArg(map[string]interface{}{"r": float64(300), "g": float64(0), "b": float64(0)}, fallback)
	if got != fallback {
		t.Errorf("out-of-range r=300: got %+v, want fallback %+v", got, fallback)
	}
}

func TestParseColorArg_ValidApplies(t *testing.T) {
	fallback := viz.Color{R: 1, G: 2, B: 3, Opacity: 0.5}
	got := parseColorArg(map[string]interface{}{
		"r": float64(180), "g": float64(140), "b": float64(80), "opacity": float64(0.9),
	}, fallback)
	want := viz.Color{R: 180, G: 140, B: 80, Opacity: 0.9}
	if got != want {
		t.Errorf("valid: got %+v, want %+v", got, want)
	}
}
