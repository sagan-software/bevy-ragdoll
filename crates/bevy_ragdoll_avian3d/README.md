# bevy_ragdoll_avian3d

Avian 3D physics backend for [bevy-ragdoll](https://github.com/sagan-software/bevy-ragdoll).

```rust,ignore
use avian3d::prelude::PhysicsPlugins;
use bevy::prelude::{App, FixedUpdate};
use bevy_ragdoll::RagdollPlugin;
use bevy_ragdoll_avian3d::{AvianRagdollHooks, AvianRagdollPlugin};

App::new().add_plugins((
    RagdollPlugin::default(),
    PhysicsPlugins::new(FixedUpdate).with_collision_hooks::<AvianRagdollHooks<'static, 'static>>(),
    AvianRagdollPlugin,
));
```

Avian must run in the same schedule as the `RagdollPlugin` fixed schedule.
Avian 0.7 has no spherical joint motors and one symmetric swing cone, so the
core drives every joint with fallback torque. The crate documentation lists
the joint mapping and the conformance cases that do not pass yet.
