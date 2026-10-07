# bevy_ragdoll_rapier3d

Rapier 3D physics backend for
[bevy-ragdoll](https://github.com/sagan-software/bevy-ragdoll).

Add `RagdollPlugin`, then `RapierPhysicsPlugin` in the `RagdollPlugin` fixed
schedule with `RapierRagdollHooks` as its hooks, then `RapierRagdollPlugin`.
The application owns the Rapier world. The adapter maps ragdoll bodies,
joints, contacts, and raycasts to Rapier and reports them back through the
backend-neutral components and messages.
