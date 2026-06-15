package contracts

import (
	"context"
	"errors"
	"testing"
)

// fakeDoCommander records the request and returns a canned reply/error.
type fakeDoCommander struct {
	gotCmd map[string]interface{}
	reply  map[string]interface{}
	err    error
}

func (f *fakeDoCommander) DoCommand(_ context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	f.gotCmd = cmd
	if f.err != nil {
		return nil, f.err
	}
	return f.reply, nil
}

// TestVerbStrings pins the renamed verbs so a typo can't drift them.
func TestVerbStrings(t *testing.T) {
	cases := map[string]string{
		VerbNextBox:         "next_box",
		VerbGetStatus:       "get_status",
		VerbResetProgress:   "reset_progress",
		VerbReportPlacement: "report_placement",
		VerbSkipBox:         "skip_box",
		VerbGetBoxDims:      "get_box_dims",
	}
	for got, want := range cases {
		if got != want {
			t.Errorf("verb constant = %q, want %q", got, want)
		}
	}
}

// TestNextBoxHasNoCounters asserts the placement response carries no
// progress counters — those moved to StatusResponse.
func TestNextBoxHasNoCounters(t *testing.T) {
	m, err := ToMap(NextBoxResponse{Seq: 3, IsComplete: false})
	if err != nil {
		t.Fatal(err)
	}
	for _, banned := range []string{"placed", "failed", "skipped", "remaining", "total"} {
		if _, ok := m[banned]; ok {
			t.Errorf("NextBoxResponse should not carry %q (moved to get_status)", banned)
		}
	}
	if _, ok := m["is_complete"]; !ok {
		t.Error("NextBoxResponse missing is_complete")
	}
}

// TestStatusCounterKeys asserts get_status uses bare counter keys and
// next_box_index (not the old *_count / next_seq spellings).
func TestStatusCounterKeys(t *testing.T) {
	m, err := ToMap(StatusResponse{Placed: 1, Total: 8, NextBoxIndex: 2})
	if err != nil {
		t.Fatal(err)
	}
	for _, want := range []string{"placed", "failed", "skipped", "remaining", "total", "next_box_index", "done_seqs"} {
		if _, ok := m[want]; !ok {
			t.Errorf("StatusResponse missing %q", want)
		}
	}
	for _, banned := range []string{"placed_count", "failed_count", "skipped_count", "next_seq"} {
		if _, ok := m[banned]; ok {
			t.Errorf("StatusResponse should not carry %q", banned)
		}
	}
}

func TestNextBoxClient(t *testing.T) {
	f := &fakeDoCommander{reply: map[string]interface{}{"seq": 3, "is_complete": false}}
	resp, err := NextBox(context.Background(), f)
	if err != nil {
		t.Fatalf("NextBox: %v", err)
	}
	if _, ok := f.gotCmd[VerbNextBox]; !ok {
		t.Fatalf("request missing verb %q: %v", VerbNextBox, f.gotCmd)
	}
	if resp.Seq != 3 || resp.IsComplete {
		t.Errorf("decoded wrong: %+v", resp)
	}
}

func TestReportPlacementNestsRequestUnderVerb(t *testing.T) {
	f := &fakeDoCommander{reply: map[string]interface{}{"acknowledged": true, "next_box_index": 4, "complete": true}}
	resp, err := ReportPlacement(context.Background(), f, ReportPlacementRequest{Seq: 3, Success: true})
	if err != nil {
		t.Fatalf("ReportPlacement: %v", err)
	}
	body, ok := f.gotCmd[VerbReportPlacement].(map[string]interface{})
	if !ok {
		t.Fatalf("request body not nested under %q: %v", VerbReportPlacement, f.gotCmd)
	}
	if body["seq"].(float64) != 3 || body["success"].(bool) != true {
		t.Errorf("encoded request wrong: %v", body)
	}
	if !resp.Complete || resp.NextBoxIndex != 4 {
		t.Errorf("decoded response wrong: %+v", resp)
	}
}

func TestResetProgressPropagatesError(t *testing.T) {
	f := &fakeDoCommander{err: errors.New("boom")}
	if err := ResetProgress(context.Background(), f); err == nil {
		t.Fatal("expected error to propagate")
	}
	if _, ok := f.gotCmd[VerbResetProgress]; !ok {
		t.Fatalf("request missing verb %q: %v", VerbResetProgress, f.gotCmd)
	}
}
