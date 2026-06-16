package contracts

// Pack-sequencer DoCommand verb names. Use the typed client functions
// below rather than these constants directly where you can; they are
// exported for callers that build a raw DoCommand map.
const (
	VerbNextBox           = "next_box"
	VerbReportPlacement   = "report_placement"
	VerbGetBoxDims        = "get_box_dims"
	VerbGetPalletHome     = "get_pallet_home"
	VerbGetPackOrder      = "get_pack_order"
	VerbGetStatus         = "get_status"
	VerbResetProgress     = "reset_progress"
	VerbSkipBox           = "skip_box"
	VerbSetBoxVisual   = "set_box_visual"
	VerbClearBoxVisual = "clear_box_visual"
	VerbGetAttributes     = "get_attributes"
	VerbSetAttributes     = "set_attributes"
)

// NextBoxResponse is the placement of the box at the current box index
// — where the next box goes. It carries geometry only; ask GetStatus
// for progress counters. When the pack is finished, IsComplete is true
// and the placement fields are zero.
//
// The place move is a two-pose trajectory, not a single drop:
//
//   - PlaceEnd is the final slot — where the box is set down.
//   - PlaceStart is offset up and over from PlaceEnd by the angled
//     approach, so the box descends diagonally into the slot and clears
//     already-placed neighbours instead of dragging across them.
//
// The arm moves to PlaceStart, then descends to PlaceEnd. Each pose is
// given in two frames: the *InWorld pair is pre-composed with the
// pallet's world pose — hand those straight to the motion service. The
// *InPallet pair is the same two poses in the pallet's local frame,
// for visualization or pallet-frame reasoning (not needed for motion).
type NextBoxResponse struct {
	Seq                int           `json:"seq"`
	Col                int           `json:"col"`
	Row                int           `json:"row"`
	Layer              int           `json:"layer"`
	PlaceStartInWorld  Pose6D        `json:"place_start_in_world"`
	PlaceEndInWorld    Pose6D        `json:"place_end_in_world"`
	PlaceStartInPallet Pose6D        `json:"place_start_in_pallet"`
	PlaceEndInPallet   Pose6D        `json:"place_end_in_pallet"`
	BoxDimensionsMM    BoxDimensions `json:"box_dimensions_mm"`
	IsComplete         bool          `json:"is_complete"`
}

// ApproachOffset is the per-box angled-approach delta in pallet-local
// frame, applied to the place-end slot to produce place-start.
type ApproachOffset struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
	Z float64 `json:"z"`
}

// BoxDimensions is a box footprint in millimeters. Length is the
// pack-order Y dimension, which may be the box's long or short side
// depending on its rotation in the pack.
type BoxDimensions struct {
	Width  float64 `json:"width"`
	Length float64 `json:"length"`
	Height float64 `json:"height"`
}

// ReportPlacementRequest reports the outcome of a placement. Success
// advances the cursor to the next box; failure leaves it for a retry
// and records Error.
type ReportPlacementRequest struct {
	Seq     int    `json:"seq"`
	Success bool   `json:"success"`
	Error   string `json:"error,omitempty"`
}

// ReportPlacementResponse is the progress snapshot returned after a
// placement report. Complete is true once every box is placed.
type ReportPlacementResponse struct {
	Acknowledged bool   `json:"acknowledged"`
	NextBoxIndex int    `json:"next_box_index"`
	Placed       int    `json:"placed"`
	Failed       int    `json:"failed"`
	Skipped      int    `json:"skipped"`
	Remaining    int    `json:"remaining"`
	Complete     bool   `json:"complete"`
	LastError    string `json:"last_error,omitempty"`
}

// StatusResponse is the pack-sequencer's full progress snapshot — the
// single place to ask "how far along is the pack?". The *Seqs slices
// list which box indices have been placed, skipped, or failed; the
// counters are their sizes plus Remaining and Total.
type StatusResponse struct {
	NextBoxIndex int   `json:"next_box_index"`
	DoneSeqs     []int `json:"done_seqs"`
	SkippedSeqs  []int `json:"skipped_seqs"`
	FailedSeqs   []int `json:"failed_seqs"`
	Placed       int   `json:"placed"`
	Failed       int   `json:"failed"`
	Skipped      int   `json:"skipped"`
	Remaining    int   `json:"remaining"`
	Total        int   `json:"total"`
	Complete     bool  `json:"complete"`
}

// GetBoxDimsResponse is the pack's box dimensions. The box_* prefix is
// part of the wire contract — do not shorten it to width_mm etc.
type GetBoxDimsResponse struct {
	BoxLengthMM float64 `json:"box_length_mm"`
	BoxWidthMM  float64 `json:"box_width_mm"`
	BoxHeightMM float64 `json:"box_height_mm"`
}

// GetPalletHomeResponse is the pallet-home pose. Local is in
// pallet-local frame; World is the same pose composed with the
// pallet's world pose.
type GetPalletHomeResponse struct {
	PalletHomeLocal Pose6D `json:"pallet_home_local"`
	PalletHomeWorld Pose6D `json:"pallet_home_world"`
}

// SetBoxVisualRequest adds or moves a named box visual in the 3D
// scene (a held box following the gripper, a dropoff preview, …).
// Color is an optional per-call override of the service's box_color
// Config attr; a nil Color uses the service default.
type SetBoxVisualRequest struct {
	Seq    int    `json:"seq"`
	Parent string `json:"parent,omitempty"`
	Pose   Pose6D `json:"pose,omitempty"`
	Color  *Color `json:"color,omitempty"`
}

// SetBoxVisualResponse is the ack returned after set_box_visual.
type SetBoxVisualResponse struct {
	Acknowledged bool   `json:"acknowledged"`
	Seq          int    `json:"seq"`
	UUID         string `json:"uuid"`
	Parent       string `json:"parent,omitempty"`
}

// ClearBoxVisualRequest removes a named box visual from the 3D
// scene.
type ClearBoxVisualRequest struct {
	Seq int `json:"seq"`
}

// SkipBoxRequest marks a box as skipped without placing it. Reason is
// recorded for later inspection.
type SkipBoxRequest struct {
	Seq    int    `json:"seq"`
	Reason string `json:"reason,omitempty"`
}

// SkipBoxResponse is the progress snapshot returned after a skip.
type SkipBoxResponse struct {
	Skipped      int `json:"skipped"`
	NextBoxIndex int `json:"next_box_index"`
	Placed       int `json:"placed"`
	Remaining    int `json:"remaining"`
}

// ResetProgressResponse is returned after reset_progress clears the
// placed/failed/skipped sets back to an empty pallet.
type ResetProgressResponse struct {
	Reset        bool `json:"reset"`
	NextBoxIndex int  `json:"next_box_index"`
}

// PackOrderPlacement is one entry in GetPackOrderResponse.Placements —
// a single box's pose, footprint, and slot. Dimensions here use bare
// width_mm / length_mm / height_mm (not the box_* prefix that
// get_box_dims uses); poses are given in both pallet-local and world
// frames.
type PackOrderPlacement struct {
	Seq                    int            `json:"seq"`
	Col                    int            `json:"col"`
	Row                    int            `json:"row"`
	Layer                  int            `json:"layer"`
	XMM                    float64        `json:"x_mm"`
	YMM                    float64        `json:"y_mm"`
	ZMM                    float64        `json:"z_mm"`
	Label                  string         `json:"label,omitempty"`
	LengthMM               float64        `json:"length_mm"`
	WidthMM                float64        `json:"width_mm"`
	HeightMM               float64        `json:"height_mm"`
	PoseInPallet           Pose6D         `json:"pose_in_pallet"`
	PoseInWorld            Pose6D         `json:"pose_in_world"`
	ApproachOffsetInPallet ApproachOffset `json:"approach_offset_in_pallet"`
}

// GetPackOrderResponse is the full computed pack order plus pallet
// geometry — every placement up front, without stepping the cursor.
type GetPackOrderResponse struct {
	Placements        []PackOrderPlacement `json:"placements"`
	Cols              int                  `json:"cols"`
	Rows              int                  `json:"rows"`
	Layers            int                  `json:"layers"`
	Capacity          int                  `json:"capacity"`
	Quantity          int                  `json:"quantity"`
	Overflow          int                  `json:"overflow"`
	Mode              string               `json:"mode"`
	Warnings          []string             `json:"warnings,omitempty"`
	PalletThicknessMM float64              `json:"pallet_thickness_mm"`
	PalletWidthMM     float64              `json:"pallet_width_mm"`
	PalletLengthMM    float64              `json:"pallet_length_mm"`
	PalletPose        Pose6D               `json:"pallet_pose"`
}
