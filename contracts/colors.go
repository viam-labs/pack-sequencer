package contracts

// Color is an RGB+opacity color. RGB is 0..255, opacity is 0..1.
// Opacity == 0 means "unspecified" — the service's default applies.
//
// Use a pointer when "unset" must be distinguishable from "set to
// (0, 0, 0)" — e.g. on a request where omitting the field means "keep
// the current color."
type Color struct {
	R       int     `json:"r"`
	G       int     `json:"g"`
	B       int     `json:"b"`
	Opacity float64 `json:"opacity,omitempty"`
}
