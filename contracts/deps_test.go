package contracts

import (
	"go/parser"
	"go/token"
	"os"
	"strings"
	"testing"
)

// TestNoExternalDeps asserts the contracts package imports only the Go
// standard library. This is load-bearing: the package is shared by the
// pack-sequencer producer and consumers pinned to different rdk
// versions, so any non-stdlib import (especially a geom -> rdk
// spatialmath dependency) would reintroduce the version-skew coupling
// this package exists to avoid. Standard-library import paths never
// have a dot in their first segment; third-party ones (github.com/…,
// go.viam.com/…) do.
func TestNoExternalDeps(t *testing.T) {
	entries, err := os.ReadDir(".")
	if err != nil {
		t.Fatal(err)
	}
	fset := token.NewFileSet()
	for _, e := range entries {
		name := e.Name()
		if e.IsDir() || !strings.HasSuffix(name, ".go") || strings.HasSuffix(name, "_test.go") {
			continue
		}
		f, err := parser.ParseFile(fset, name, nil, parser.ImportsOnly)
		if err != nil {
			t.Fatalf("parse %s: %v", name, err)
		}
		for _, imp := range f.Imports {
			path := strings.Trim(imp.Path.Value, `"`)
			first := path
			if i := strings.IndexByte(path, '/'); i >= 0 {
				first = path[:i]
			}
			if strings.Contains(first, ".") {
				t.Errorf("%s imports non-stdlib package %q; contracts must stay stdlib-only", name, path)
			}
		}
	}
}
