package main

import (
	ps "packsequencer"

	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/worldstatestore"
)

func main() {
	module.ModularMain(
		resource.APIModel{API: worldstatestore.API, Model: ps.Model},
	)
}
