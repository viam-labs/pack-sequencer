// Package contracts is the wire-format contract for the
// viam:pack-sequencer service's DoCommand verbs.
//
// At the protocol level a Viam DoCommand is just a
// `map[string]interface{}` carried over gRPC. This package defines the
// typed Go structs that describe that map for each pack-sequencer
// verb, plus a small typed client, so the producer (the pack-sequencer
// module) and its consumers (a palletizer module, an operator webapp)
// share one definition. A renamed JSON field becomes a compile error
// on both ends instead of a silent zero value.
//
// The package is deliberately dependency-light: it imports nothing
// outside the Go standard library. That lets a producer and a consumer
// pinned to different rdk versions share it without a version conflict
// — which is why the pose type here (Pose6D) is a plain struct with no
// spatialmath converter. Consumers convert at their own edge.
//
// Producer side, a verb handler builds its reply from a typed struct:
//
//	return contracts.MustToMap(contracts.GetBoxDimsResponse{
//	    BoxLengthMM: c.BoxLengthMM, BoxWidthMM: c.BoxWidthMM, BoxHeightMM: c.BoxHeightMM,
//	}), nil
//
// Consumer side, prefer the typed client over a raw DoCommand:
//
//	dims, err := contracts.GetBoxDims(ctx, svc)
package contracts

import "encoding/json"

// ToMap converts a struct to the map[string]interface{} shape Viam
// DoCommand carries on the wire. JSON struct tags drive the field
// names. Returns an error only on JSON marshal failure (e.g.
// unsupported field types).
func ToMap(v any) (map[string]interface{}, error) {
	b, err := json.Marshal(v)
	if err != nil {
		return nil, err
	}
	out := map[string]interface{}{}
	if err := json.Unmarshal(b, &out); err != nil {
		return nil, err
	}
	return out, nil
}

// MustToMap is ToMap that panics on error. Useful when the input is a
// well-known struct and a marshal failure is a programmer bug.
func MustToMap(v any) map[string]interface{} {
	m, err := ToMap(v)
	if err != nil {
		panic("contracts.MustToMap: " + err.Error())
	}
	return m
}

// FromMap parses a DoCommand wire map into a typed struct. The generic
// type parameter selects which struct shape to decode into.
func FromMap[T any](m map[string]interface{}) (T, error) {
	var zero T
	b, err := json.Marshal(m)
	if err != nil {
		return zero, err
	}
	var out T
	if err := json.Unmarshal(b, &out); err != nil {
		return zero, err
	}
	return out, nil
}
