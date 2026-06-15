package contracts

// Pose6D is a position-and-orientation record in the JSON shape the
// pack-sequencer carries on the wire. Orientation is an orientation
// vector (OX, OY, OZ) plus a rotation Theta in degrees.
//
// This is intentionally a plain data struct with no conversion to the
// SDK's spatialmath.Pose — that conversion needs the rdk dependency
// this package avoids. A consumer that wants a spatialmath.Pose builds
// one from these fields at its own edge.
type Pose6D struct {
	X     float64 `json:"x"`
	Y     float64 `json:"y"`
	Z     float64 `json:"z"`
	OX    float64 `json:"o_x"`
	OY    float64 `json:"o_y"`
	OZ    float64 `json:"o_z"`
	Theta float64 `json:"theta"`
}

// Vec3 is a 3D vector / point in millimeters.
type Vec3 struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}
