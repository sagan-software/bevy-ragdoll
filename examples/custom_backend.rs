//! Shows the core runtime and mock backend in a runnable example.
//!
//! The mock backend ignores joints and contacts, so the body's parts fall
//! apart. That behavior is expected for this contract-test backend.

/// Runs the shared mock-backend example with its window or headless options.
fn main() {
    if let Err(error) = bevy_ragdoll_examples::run_custom_backend() {
        eprintln!("custom backend example failed: {error}");
        std::process::exit(1);
    }
}
