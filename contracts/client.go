package contracts

import (
	"context"
	"fmt"
)

// DoCommander is the slice of a Viam resource these client helpers
// need: just DoCommand. Any rdk resource — including a
// worldstatestore.Service — satisfies it, so this package stays free
// of an rdk dependency.
type DoCommander interface {
	DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error)
}

// NextBox asks the pack-sequencer for the placement of the box at the
// current cursor. It does not advance the cursor — report the outcome
// with ReportPlacement for that.
func NextBox(ctx context.Context, svc DoCommander) (NextBoxResponse, error) {
	m, err := svc.DoCommand(ctx, map[string]interface{}{VerbNextBox: true})
	if err != nil {
		return NextBoxResponse{}, fmt.Errorf("next_box: %w", err)
	}
	return FromMap[NextBoxResponse](m)
}

// GetStatus returns the pack-sequencer's full progress snapshot.
func GetStatus(ctx context.Context, svc DoCommander) (StatusResponse, error) {
	m, err := svc.DoCommand(ctx, map[string]interface{}{VerbGetStatus: true})
	if err != nil {
		return StatusResponse{}, fmt.Errorf("get_status: %w", err)
	}
	return FromMap[StatusResponse](m)
}

// ReportPlacement reports how a placement went so the cursor advances
// (success) or holds for a retry (failure).
func ReportPlacement(ctx context.Context, svc DoCommander, req ReportPlacementRequest) (ReportPlacementResponse, error) {
	body, err := ToMap(req)
	if err != nil {
		return ReportPlacementResponse{}, fmt.Errorf("report_placement encode: %w", err)
	}
	m, err := svc.DoCommand(ctx, map[string]interface{}{VerbReportPlacement: body})
	if err != nil {
		return ReportPlacementResponse{}, fmt.Errorf("report_placement: %w", err)
	}
	return FromMap[ReportPlacementResponse](m)
}

// ReportSuccess reports that the box currently being placed — the one at
// the cursor — went down cleanly, advancing the pack to the next box. It's
// shorthand for ReportPlacement with no seq and Success=true, so a consumer
// doesn't have to track the box index just to report it.
func ReportSuccess(ctx context.Context, svc DoCommander) (ReportPlacementResponse, error) {
	return ReportPlacement(ctx, svc, ReportPlacementRequest{Success: true})
}

// ReportFailure reports that the box currently being placed could not be
// placed, recording reason; the cursor stays put so the next NextBox returns
// the same box for a retry. Shorthand for ReportPlacement with no seq,
// Success=false, and Error=reason.
func ReportFailure(ctx context.Context, svc DoCommander, reason string) (ReportPlacementResponse, error) {
	return ReportPlacement(ctx, svc, ReportPlacementRequest{Success: false, Error: reason})
}

// GetBoxDims returns the pack's box dimensions — the single source of
// truth a consumer pulls at construction.
func GetBoxDims(ctx context.Context, svc DoCommander) (GetBoxDimsResponse, error) {
	m, err := svc.DoCommand(ctx, map[string]interface{}{VerbGetBoxDims: true})
	if err != nil {
		return GetBoxDimsResponse{}, fmt.Errorf("get_box_dims: %w", err)
	}
	return FromMap[GetBoxDimsResponse](m)
}

// GetPalletHome returns the pallet-home pose in pallet-local and world
// frames.
func GetPalletHome(ctx context.Context, svc DoCommander) (GetPalletHomeResponse, error) {
	m, err := svc.DoCommand(ctx, map[string]interface{}{VerbGetPalletHome: true})
	if err != nil {
		return GetPalletHomeResponse{}, fmt.Errorf("get_pallet_home: %w", err)
	}
	return FromMap[GetPalletHomeResponse](m)
}

// GetPackOrder returns the full computed pack order plus pallet info.
func GetPackOrder(ctx context.Context, svc DoCommander) (GetPackOrderResponse, error) {
	m, err := svc.DoCommand(ctx, map[string]interface{}{VerbGetPackOrder: true})
	if err != nil {
		return GetPackOrderResponse{}, fmt.Errorf("get_pack_order: %w", err)
	}
	return FromMap[GetPackOrderResponse](m)
}

// ResetProgress clears the placed/failed/skipped sets back to an empty
// pallet. The service's reply isn't typed for the caller; callers that
// need it can fall back to a raw DoCommand.
func ResetProgress(ctx context.Context, svc DoCommander) error {
	if _, err := svc.DoCommand(ctx, map[string]interface{}{VerbResetProgress: true}); err != nil {
		return fmt.Errorf("reset_progress: %w", err)
	}
	return nil
}

// SkipBox marks a box as skipped without placing it (an operator
// action).
func SkipBox(ctx context.Context, svc DoCommander, req SkipBoxRequest) error {
	body, err := ToMap(req)
	if err != nil {
		return fmt.Errorf("skip_box encode: %w", err)
	}
	if _, err := svc.DoCommand(ctx, map[string]interface{}{VerbSkipBox: body}); err != nil {
		return fmt.Errorf("skip_box: %w", err)
	}
	return nil
}

// SetBoxVisual adds or moves a named box visual in the 3D scene.
func SetBoxVisual(ctx context.Context, svc DoCommander, req SetBoxVisualRequest) (SetBoxVisualResponse, error) {
	body, err := ToMap(req)
	if err != nil {
		return SetBoxVisualResponse{}, fmt.Errorf("set_box_visual encode: %w", err)
	}
	m, err := svc.DoCommand(ctx, map[string]interface{}{VerbSetBoxVisual: body})
	if err != nil {
		return SetBoxVisualResponse{}, fmt.Errorf("set_box_visual: %w", err)
	}
	return FromMap[SetBoxVisualResponse](m)
}

// ClearBoxVisual removes a named box visual from the 3D scene.
func ClearBoxVisual(ctx context.Context, svc DoCommander, req ClearBoxVisualRequest) error {
	body, err := ToMap(req)
	if err != nil {
		return fmt.Errorf("clear_box_visual encode: %w", err)
	}
	if _, err := svc.DoCommand(ctx, map[string]interface{}{VerbClearBoxVisual: body}); err != nil {
		return fmt.Errorf("clear_box_visual: %w", err)
	}
	return nil
}
