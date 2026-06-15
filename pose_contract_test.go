package packsequencer

import (
	"testing"

	"github.com/viam-labs/pack-sequencer/contracts"
)

// TestToContractsPose guards the one place the producer crosses from its
// rdk-backed geom.Pose6D to the dependency-free contracts.Pose6D: a
// swapped field here would silently mis-place every box, so pin the
// field mapping.
func TestToContractsPose(t *testing.T) {
	in := Pose6D{X: 1, Y: 2, Z: 3, OX: 0.4, OY: 0.5, OZ: 0.6, Theta: 90}
	got := toContractsPose(in)
	want := contracts.Pose6D{X: 1, Y: 2, Z: 3, OX: 0.4, OY: 0.5, OZ: 0.6, Theta: 90}
	if got != want {
		t.Errorf("toContractsPose mismapped a field: got %+v want %+v", got, want)
	}
}
