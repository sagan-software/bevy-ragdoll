# bevy_ragdoll_conformance

Backend conformance checks and a mock backend for
[bevy-ragdoll](https://github.com/sagan-software/bevy-ragdoll).

- `contract`: component and message checks that every physics backend must
  pass.
- `physics`: physical behavior checks on a generated humanoid.
- `mock`: an approximate backend for development and tests.

A backend crate calls these functions from its integration tests with its own
plugin setup.
