//! Measure headless or visible bevy-ragdoll stress scenarios.

use bevy_ragdoll_examples::stress::run_stress;
use std::process::ExitCode;

/// Parses stress options and runs one scenario, comparison, or child-process sweep.
fn main() -> ExitCode {
    match run_stress() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            let message = error.to_string();
            eprintln!("{message}");
            if message.contains("needs Phase") || message.contains("needs phase") {
                ExitCode::from(2)
            } else {
                ExitCode::FAILURE
            }
        }
    }
}
