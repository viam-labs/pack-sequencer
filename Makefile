GO_BUILD_ENV :=
GO_BUILD_FLAGS :=
MODULE_BINARY := bin/pack-sequencer
VERSION := $(shell cat VERSION 2>/dev/null)
PLATFORM ?= linux/amd64

$(MODULE_BINARY): Makefile go.mod *.go cmd/module/*.go
	$(GO_BUILD_ENV) go build $(GO_BUILD_FLAGS) -o $(MODULE_BINARY) cmd/module/main.go

lint:
	gofmt -s -w .

update:
	go get go.viam.com/rdk@latest
	go mod tidy

test:
	go test ./...

module.tar.gz: meta.json $(MODULE_BINARY) VERSION
	strip $(MODULE_BINARY)
	tar czf $@ meta.json $(MODULE_BINARY)

module: test module.tar.gz

all: test module.tar.gz

publish: module.tar.gz
	viam module upload --version $(VERSION) --platform $(PLATFORM) module.tar.gz

setup:
	go mod tidy
