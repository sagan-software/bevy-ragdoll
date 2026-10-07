//! Measures ragdoll body, joint, and fixed-step costs across stress scenarios.
//!
//! Each run spawns a population of TGF human ragdolls, activates them on a
//! schedule, and records frame, physics-step, and ragdoll-core timings. The run
//! ends with a schema-1 JSON report that `--compare` can check against a
//! baseline such as `benches/baselines/*.json`.
//!
//! Run one headless scenario and write its report:
//!
//! ```sh
//! cargo run --example ragdoll_stress -- --headless --scenario pile --count 4 \
//!   --duration 2 --warmup 0 --report stress-smoke.json
//! ```
//!
//! Run the default sweep, where every row runs in a fresh headless child process:
//!
//! ```sh
//! cargo run --release --example ragdoll_stress -- --headless --sweep default
//! ```
//!
//! Omit `--headless` to open a window. In the window, Space triggers activation
//! and F toggles a global freeze. `--screenshot path.png` saves the final frame.
//! Run with `--help` for every option.

use std::collections::HashMap;
use std::error::Error;
use std::fmt::{Display, Formatter};
use std::num::NonZeroU32;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};
use std::str::FromStr;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use bevy::app::ScheduleRunnerPlugin;
use bevy::diagnostic::{DiagnosticsStore, FrameTimeDiagnosticsPlugin};
use bevy::math::Isometry3d;
use bevy::prelude::*;
use bevy::render::renderer::RenderAdapterInfo;
use bevy::render::view::screenshot::{Screenshot, save_to_disk};
use bevy::time::TimeUpdateStrategy;
use bevy::transform::TransformSystems;
use bevy::window::PresentMode;
use bevy::winit::WinitSettings;
use bevy_ragdoll::runtime::body::{
    BodyAtRest, BodyKind, BodyPhysicsPose, BodyShape, BodyVelocity, JointToParent,
};
use bevy_ragdoll::runtime::budget::RagdollBudget;
use bevy_ragdoll::runtime::components::{Ragdoll, RagdollBodyOf, RagdollDrive, RagdollMode};
use bevy_ragdoll::runtime::hit::{HitProfile, HitSettings, LastHit};
use bevy_ragdoll::runtime::messages::{HitKind, RagdollHit};
use bevy_ragdoll::runtime::sets::{RagdollFixedSystems, RagdollSystems};
use bevy_ragdoll::{BodyIndex, RagdollPlugin, RagdollProfile, ShapeSpec};
use bevy_ragdoll_rapier3d::{RapierRagdollHooks, RapierRagdollPlugin};
use bevy_rapier3d::plugin::{PhysicsSet, RapierPhysicsPlugin, TimestepMode};
use bevy_rapier3d::prelude::{Collider, RigidBody};
use clap::Parser;
use serde::{Deserialize, Serialize};

/// Parses stress options and runs one scenario, comparison, or child-process sweep.
///
/// Options that a later phase implements exit with status 2.
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

/// Runs the stress app, sweep, or baseline comparison selected on the command line.
fn run_stress() -> Result<(), Box<dyn Error>> {
    #[cfg(not(target_arch = "wasm32"))]
    let cli = StressCli::parse();
    #[cfg(target_arch = "wasm32")]
    let cli = StressCli::parse_from(["ragdoll_stress"]);

    if let Some(sweep) = cli.sweep.as_ref() {
        return run_sweep(&cli, sweep);
    }

    let config = validate_run(&cli)?;
    let profile = load_profile()?;
    let (report_path, temporary_report) = report_path(&cli)?;
    let result = run_app(&cli, config, profile, &report_path)
        .and_then(|report| write_and_compare(&cli, &report));
    if temporary_report {
        let _ = std::fs::remove_file(report_path);
    }
    result
}

/// Generates the human profile from the reference humanoid skeleton.
fn load_profile() -> Result<RagdollProfile, Box<dyn Error>> {
    Ok(RagdollProfile::from_skeleton(&bevy_ragdoll::Skeleton::humanoid())?)
}

// ---------------------------------------------------------------------------
// Command line
// ---------------------------------------------------------------------------

/// Command-line options for one stress run or a sweep.
#[derive(Clone, Debug, Parser)]
#[command(
    name = "ragdoll_stress",
    about = "Measure bevy-ragdoll stress scenarios"
)]
struct StressCli {
    /// Select one compiled physics backend.
    #[arg(long, default_value = "rapier3d")]
    backend: Backend,
    /// Select the simulation population and activation scenario.
    #[arg(long, default_value = "grid")]
    scenario: Scenario,
    /// Character count for pile and shooting scenarios.
    #[arg(long, default_value = "64")]
    count: CharacterCount,
    /// Grid width and depth written as `columns x rows`.
    #[arg(long, default_value = "16x16")]
    grid: GridSize,
    /// Distance between grid characters in metres.
    #[arg(long, default_value = "1.5")]
    spacing: Meters,
    /// Select the character rig profile.
    #[arg(long, default_value = "human")]
    creature: Creature,
    /// Select the drive policy after activation.
    #[arg(long, default_value = "limp")]
    mode: DriveMode,
    /// Seconds before the first activation trigger.
    #[arg(long, default_value = "2")]
    trigger_at: NonnegativeSeconds,
    /// Seconds measured after activation.
    #[arg(long, default_value = "10")]
    duration: PositiveSeconds,
    /// Initial seconds excluded from measurements.
    #[arg(long, default_value = "1")]
    warmup: NonnegativeSeconds,
    /// Fixed simulation frequency in hertz.
    #[arg(long, default_value = "60")]
    fixed_hz: PositiveHertz,
    /// Rapier substeps per fixed interval; the backend default is used when absent.
    #[arg(long)]
    substeps: Option<PositiveCount>,
    /// Maximum dynamic-character budget; absence means unlimited.
    #[arg(long)]
    budget: Option<Budget>,
    /// Seed used for all deterministic stress inputs.
    #[arg(long, default_value = "42")]
    seed: Seed,
    /// Enable the reserved deterministic backend path.
    #[arg(long, num_args = 0..=1, default_missing_value = "enabled")]
    deterministic: Option<DeterministicMode>,
    /// Run without a window or renderer.
    #[arg(long, num_args = 0..=1, default_missing_value = "enabled")]
    headless: Option<HeadlessMode>,
    /// Write one run report to this JSON path.
    #[arg(long)]
    report: Option<PathBuf>,
    /// Save a visible-run screenshot to this PNG path.
    #[arg(long)]
    screenshot: Option<PathBuf>,
    /// Run the named default sweep or read a RON sweep file.
    #[arg(long)]
    sweep: Option<SweepSelection>,
    /// Compare this run or sweep with a prior JSON report.
    #[arg(long)]
    compare: Option<PathBuf>,
}

impl StressCli {
    /// Converts parsed boundary values into the serializable run description.
    fn run_config(&self) -> Result<RunConfig, &'static str> {
        let [columns, rows] = self.grid.as_array();
        let grid_count = usize::try_from(columns.get())
            .ok()
            .and_then(|columns| {
                usize::try_from(rows.get())
                    .ok()
                    .and_then(|rows| columns.checked_mul(rows))
            })
            .ok_or("grid character count exceeds the supported range")?;
        let count = match self.scenario {
            Scenario::Pile | Scenario::Shooting => self.count.get(),
            Scenario::Grid
            | Scenario::Wave
            | Scenario::Powered
            | Scenario::Balance
            | Scenario::Mixed => grid_count,
        };
        let default_substeps = NonZeroU32::new(1).expect("one is a nonzero substep count");
        Ok(RunConfig {
            backend: self.backend,
            scenario: self.scenario,
            grid: [columns.get(), rows.get()],
            count,
            spacing_m: self.spacing.get(),
            creature: self.creature,
            mode: self.mode,
            trigger_at_s: self.trigger_at.get().as_secs_f64(),
            duration_s: self.duration.get().as_secs_f64(),
            warmup_s: self.warmup.get().as_secs_f64(),
            fixed_hz: self.fixed_hz.get(),
            substeps: Some(
                self.substeps
                    .map_or(default_substeps, PositiveCount::get)
                    .get(),
            ),
            budget: self.budget.map(Budget::get),
            seed: self.seed.get(),
            headless: self.headless.is_some(),
            deterministic: self.deterministic.is_some(),
        })
    }

    /// Builds headless child-process arguments from validated values.
    ///
    /// Values other than the overridden ones are copied from this parent run.
    fn child_arguments(
        &self,
        backend: Backend,
        scenario: Scenario,
        grid: GridSize,
        count: CharacterCount,
        report: &Path,
    ) -> Vec<String> {
        let mut arguments = vec![
            "--backend".to_owned(),
            backend.to_string(),
            "--scenario".to_owned(),
            scenario.to_string(),
            "--count".to_owned(),
            count.to_string(),
            "--grid".to_owned(),
            grid.to_string(),
            "--spacing".to_owned(),
            self.spacing.to_string(),
            "--creature".to_owned(),
            self.creature.to_string(),
            "--mode".to_owned(),
            self.mode.to_string(),
            "--trigger-at".to_owned(),
            self.trigger_at.to_string(),
            "--duration".to_owned(),
            self.duration.to_string(),
            "--warmup".to_owned(),
            self.warmup.to_string(),
            "--fixed-hz".to_owned(),
            self.fixed_hz.to_string(),
            "--seed".to_owned(),
            self.seed.to_string(),
        ];
        if let Some(substeps) = self.substeps {
            arguments.extend(["--substeps".to_owned(), substeps.to_string()]);
        }
        if let Some(budget) = self.budget {
            arguments.extend(["--budget".to_owned(), budget.to_string()]);
        }
        arguments.push("--headless".to_owned());
        arguments.extend(["--report".to_owned(), report.display().to_string()]);
        arguments
    }
}

/// Positive character count parsed before allocating a population.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CharacterCount(
    /// Nonzero number of characters allocated by this run.
    usize,
);

impl CharacterCount {
    /// Returns the validated nonzero count.
    const fn get(self) -> usize {
        self.0
    }

    /// Creates a positive character count from a RON sweep row.
    fn try_new(value: usize) -> Result<Self, &'static str> {
        if value == 0 {
            Err("sweep character count must be positive")
        } else {
            Ok(Self(value))
        }
    }
}

impl FromStr for CharacterCount {
    type Err = &'static str;

    /// Parses a nonzero character count.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        match input.parse::<usize>() {
            Ok(count) if count > 0 => Ok(Self(count)),
            _ => Err("count must be a positive integer"),
        }
    }
}

impl Display for CharacterCount {
    /// Writes the canonical positive decimal character count.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        let count = self.0;
        write!(formatter, "{count}")
    }
}

/// Positive grid dimensions stored as nonzero unsigned integers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct GridSize {
    /// Number of columns in the horizontal grid.
    columns: NonZeroU32,
    /// Number of rows in the horizontal grid.
    rows: NonZeroU32,
}

impl GridSize {
    /// Returns checked grid dimensions in column-then-row order.
    const fn as_array(self) -> [NonZeroU32; 2] {
        [self.columns, self.rows]
    }

    /// Creates positive grid dimensions from a RON sweep row.
    fn try_new(columns: u32, rows: u32) -> Result<Self, &'static str> {
        let columns = NonZeroU32::new(columns).ok_or("sweep grid columns must be positive")?;
        let rows = NonZeroU32::new(rows).ok_or("sweep grid rows must be positive")?;
        Ok(Self { columns, rows })
    }
}

impl FromStr for GridSize {
    type Err = &'static str;

    /// Parses positive dimensions separated by a lowercase `x`.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let (columns, rows) = input
            .split_once('x')
            .ok_or("grid must use the form COLUMNSxROWS")?;
        let dimension = |value: &str| {
            value
                .parse::<u32>()
                .ok()
                .and_then(NonZeroU32::new)
                .ok_or("grid dimensions must be positive 32-bit integers")
        };
        Ok(Self {
            columns: dimension(columns)?,
            rows: dimension(rows)?,
        })
    }
}

impl Display for GridSize {
    /// Writes the canonical column-then-row spelling.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        let columns = self.columns;
        let rows = self.rows;
        write!(formatter, "{columns}x{rows}")
    }
}

/// Positive finite grid distance measured in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Meters(
    /// Positive finite distance in metres.
    f32,
);

impl Meters {
    /// Returns the validated distance in metres.
    const fn get(self) -> f32 {
        self.0
    }
}

impl FromStr for Meters {
    type Err = &'static str;

    /// Parses a finite positive floating-point distance.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let error = "spacing must be positive and finite";
        let value = input.parse::<f64>().map_err(|_| error)?;
        if !value.is_finite() || value <= 0.0 || value > f64::from(f32::MAX) {
            return Err(error);
        }
        let meters = value as f32;
        if meters.is_finite() && meters > 0.0 {
            Ok(Self(meters))
        } else {
            Err(error)
        }
    }
}

impl Display for Meters {
    /// Writes the canonical decimal distance in metres.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        let meters = self.0;
        write!(formatter, "{meters}")
    }
}

/// Positive duration parsed from seconds without retaining a raw float.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PositiveSeconds(
    /// Nonzero duration.
    Duration,
);

impl PositiveSeconds {
    /// Returns the validated positive duration.
    const fn get(self) -> Duration {
        self.0
    }
}

impl FromStr for PositiveSeconds {
    type Err = &'static str;

    /// Converts positive finite seconds into `Duration`.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let seconds = parse_seconds(input)?;
        if seconds <= 0.0 {
            return Err("seconds must be positive and finite");
        }
        let duration = Duration::try_from_secs_f64(seconds)
            .map_err(|_| "seconds exceed the supported duration")?;
        if duration.is_zero() {
            Err("seconds must be positive and finite")
        } else {
            Ok(Self(duration))
        }
    }
}

impl Display for PositiveSeconds {
    /// Writes the duration's canonical seconds value.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        let seconds = self.0.as_secs_f64();
        write!(formatter, "{seconds}")
    }
}

/// Nonnegative duration used for trigger and warmup offsets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct NonnegativeSeconds(
    /// Duration that may be zero.
    Duration,
);

impl NonnegativeSeconds {
    /// Returns the validated nonnegative duration.
    const fn get(self) -> Duration {
        self.0
    }
}

impl FromStr for NonnegativeSeconds {
    type Err = &'static str;

    /// Converts nonnegative finite seconds into `Duration`.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        let seconds = parse_seconds(input)?;
        if seconds < 0.0 {
            return Err("seconds must be nonnegative and finite");
        }
        Duration::try_from_secs_f64(seconds)
            .map(Self)
            .map_err(|_| "seconds exceed the supported duration")
    }
}

impl Display for NonnegativeSeconds {
    /// Writes the duration's canonical seconds value.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        let seconds = self.0.as_secs_f64();
        write!(formatter, "{seconds}")
    }
}

/// Parses finite seconds shared by the two validated duration types.
fn parse_seconds(input: &str) -> Result<f64, &'static str> {
    match input.parse::<f64>() {
        Ok(seconds) if seconds.is_finite() => Ok(seconds),
        _ => Err("seconds must be finite"),
    }
}

/// Positive fixed simulation frequency in hertz.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PositiveHertz(
    /// Nonzero fixed-step frequency in hertz.
    NonZeroU32,
);

impl PositiveHertz {
    /// Returns the validated frequency in hertz.
    const fn get(self) -> u32 {
        self.0.get()
    }
}

impl FromStr for PositiveHertz {
    type Err = &'static str;

    /// Parses a nonzero fixed-step frequency.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        input
            .parse::<u32>()
            .ok()
            .and_then(NonZeroU32::new)
            .map(Self)
            .ok_or("fixed-hz must be a positive 32-bit integer")
    }
}

impl Display for PositiveHertz {
    /// Writes the integer frequency in hertz.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        let hertz = self.0;
        write!(formatter, "{hertz}")
    }
}

/// Positive count used for backend substeps.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct PositiveCount(
    /// Nonzero backend substep count.
    NonZeroU32,
);

impl PositiveCount {
    /// Returns the validated substep count.
    const fn get(self) -> NonZeroU32 {
        self.0
    }
}

impl FromStr for PositiveCount {
    type Err = &'static str;

    /// Parses a positive substep count.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        input
            .parse::<u32>()
            .ok()
            .and_then(NonZeroU32::new)
            .map(Self)
            .ok_or("substeps must be a positive 32-bit integer")
    }
}

impl Display for PositiveCount {
    /// Writes the integer substep count.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        let count = self.0;
        write!(formatter, "{count}")
    }
}

/// Optional dynamic-character budget where zero is a valid limit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Budget(
    /// Maximum dynamic-character count, including zero.
    usize,
);

impl Budget {
    /// Returns the checked dynamic-character limit.
    const fn get(self) -> usize {
        self.0
    }
}

impl FromStr for Budget {
    type Err = &'static str;

    /// Parses a nonnegative platform-sized dynamic-character limit.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        input
            .parse::<usize>()
            .map(Self)
            .map_err(|_| "budget must be a nonnegative integer")
    }
}

impl Display for Budget {
    /// Writes the integer dynamic-character limit.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        let budget = self.0;
        write!(formatter, "{budget}")
    }
}

/// Seed value kept distinct from counts and budgets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Seed(
    /// Unsigned pseudorandom seed.
    u64,
);

impl Seed {
    /// Returns the parsed pseudorandom seed.
    const fn get(self) -> u64 {
        self.0
    }
}

impl FromStr for Seed {
    type Err = &'static str;

    /// Parses one unsigned 64-bit random seed.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        input
            .parse::<u64>()
            .map(Self)
            .map_err(|_| "seed must be an unsigned 64-bit integer")
    }
}

impl Display for Seed {
    /// Writes the canonical unsigned decimal seed.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        let seed = self.0;
        write!(formatter, "{seed}")
    }
}

/// Presence marker for an enabled deterministic mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DeterministicMode {
    /// The `--deterministic` flag was present.
    Enabled,
}

impl FromStr for DeterministicMode {
    type Err = &'static str;

    /// Parses the closed presence-marker value used by Clap's optional flag.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        match input {
            "enabled" => Ok(Self::Enabled),
            _ => Err("deterministic accepts no value"),
        }
    }
}

/// Presence marker for a run without a window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum HeadlessMode {
    /// The `--headless` flag was present.
    Enabled,
}

impl FromStr for HeadlessMode {
    type Err = &'static str;

    /// Parses the closed presence-marker value used by Clap's optional flag.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        match input {
            "enabled" => Ok(Self::Enabled),
            _ => Err("headless accepts no value"),
        }
    }
}

/// Selects the built-in sweep matrix or a RON configuration path.
#[derive(Clone, Debug, PartialEq, Eq)]
enum SweepSelection {
    /// Use the default matrix for compiled backends and scenarios.
    Default,
    /// Read the sweep matrix from a RON file.
    File(
        /// RON source path containing ordered sweep rows.
        PathBuf,
    ),
}

impl FromStr for SweepSelection {
    type Err = &'static str;

    /// Parses the reserved `default` token or a nonempty file path.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        match input {
            "default" => Ok(Self::Default),
            "" => Err("sweep path cannot be empty"),
            path => Ok(Self::File(PathBuf::from(path))),
        }
    }
}

// ---------------------------------------------------------------------------
// Run configuration (serialized into the report's `config` object)
// ---------------------------------------------------------------------------

/// Physics backend selected for one stress application process.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
enum Backend {
    /// Rapier 3D physics backend.
    Rapier3d,
    /// Avian 3D physics backend, enabled in a later phase.
    Avian3d,
    /// Jolt 3D physics backend, enabled in a later phase.
    Jolt,
}

impl FromStr for Backend {
    type Err = &'static str;

    /// Parses one backend name at the command-line boundary.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        match input {
            "rapier3d" => Ok(Self::Rapier3d),
            "avian3d" => Ok(Self::Avian3d),
            "jolt" => Ok(Self::Jolt),
            _ => Err("backend must be rapier3d, avian3d, or jolt"),
        }
    }
}

impl Display for Backend {
    /// Writes the stable command-line spelling for the backend.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(match self {
            Self::Rapier3d => "rapier3d",
            Self::Avian3d => "avian3d",
            Self::Jolt => "jolt",
        })
    }
}

/// Stress scenario selected for one application process.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
enum Scenario {
    /// Drops characters in a vertical contact-heavy stack.
    Pile,
    /// Switches a standing character grid to dynamic simulation at once.
    Grid,
    /// Switches each standing grid row to dynamic simulation in sequence.
    Wave,
    /// Holds dynamic ragdolls with muscle and pin drive.
    Powered,
    /// Runs the Phase 9 balance controller.
    Balance,
    /// Applies hit reactions to a ring of characters.
    Shooting,
    /// Mixes humans and creature profiles from Phase 11.
    Mixed,
}

impl FromStr for Scenario {
    type Err = &'static str;

    /// Parses one scenario name at the command-line boundary.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        match input {
            "pile" => Ok(Self::Pile),
            "grid" => Ok(Self::Grid),
            "wave" => Ok(Self::Wave),
            "powered" => Ok(Self::Powered),
            "balance" => Ok(Self::Balance),
            "shooting" => Ok(Self::Shooting),
            "mixed" => Ok(Self::Mixed),
            _ => Err("scenario must be pile, grid, wave, powered, balance, shooting, or mixed"),
        }
    }
}

impl Display for Scenario {
    /// Writes the stable command-line spelling for the scenario.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(match self {
            Self::Pile => "pile",
            Self::Grid => "grid",
            Self::Wave => "wave",
            Self::Powered => "powered",
            Self::Balance => "balance",
            Self::Shooting => "shooting",
            Self::Mixed => "mixed",
        })
    }
}

/// Creature profile selected for each spawned character.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
enum Creature {
    /// The TGF human profile.
    Human,
}

impl FromStr for Creature {
    type Err = &'static str;

    /// Parses the human profile name supported by this phase.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        match input {
            "human" => Ok(Self::Human),
            _ => Err("creature must be human until Phase 11 adds creature profiles"),
        }
    }
}

impl Display for Creature {
    /// Writes the stable command-line spelling for the creature profile.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(match self {
            Self::Human => "human",
        })
    }
}

/// Drive policy applied after a scenario trigger.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
enum DriveMode {
    /// Disables joint and pin drive after activation.
    Limp,
    /// Applies full muscle drive and half-strength pin drive.
    Powered,
    /// Applies the Phase 9 balance controller.
    Balance,
}

impl FromStr for DriveMode {
    type Err = &'static str;

    /// Parses one drive mode at the command-line boundary.
    fn from_str(input: &str) -> Result<Self, Self::Err> {
        match input {
            "limp" => Ok(Self::Limp),
            "powered" => Ok(Self::Powered),
            "balance" => Ok(Self::Balance),
            _ => Err("mode must be limp, powered, or balance"),
        }
    }
}

impl Display for DriveMode {
    /// Writes the stable command-line spelling for the drive mode.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(match self {
            Self::Limp => "limp",
            Self::Powered => "powered",
            Self::Balance => "balance",
        })
    }
}

/// Serializable configuration that makes a stress report reproducible.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct RunConfig {
    /// Backend enabled for the process.
    backend: Backend,
    /// Selected simulation scenario.
    scenario: Scenario,
    /// Grid columns and rows, retained for scenarios that use characters in a grid.
    grid: [u32; 2],
    /// Number of spawned characters.
    count: usize,
    /// Horizontal grid spacing in metres.
    spacing_m: f32,
    /// Character profile.
    creature: Creature,
    /// Drive mode used after the scenario trigger.
    mode: DriveMode,
    /// Seconds from start to the default activation trigger.
    trigger_at_s: f64,
    /// Measurement duration in seconds after activation.
    duration_s: f64,
    /// Initial seconds excluded from measurements.
    warmup_s: f64,
    /// Fixed simulation frequency in hertz.
    fixed_hz: u32,
    /// Backend solver substeps, when explicitly configured.
    substeps: Option<u32>,
    /// Optional cap on dynamic bodies; absence means unlimited.
    #[serde(skip_serializing_if = "Option::is_none")]
    budget: Option<usize>,
    /// Seed used to initialize deterministic stress inputs.
    seed: u64,
    /// Whether the process runs without a window.
    headless: bool,
    /// Whether backend deterministic settings were requested.
    deterministic: bool,
}

/// Converts CLI values into a run configuration and rejects unimplemented options.
fn validate_run(cli: &StressCli) -> Result<RunConfig, Box<dyn Error>> {
    let config = cli.run_config()?;
    match config.backend {
        Backend::Rapier3d => {}
        Backend::Avian3d => return Err("avian3d needs Phase 12".into()),
        Backend::Jolt => return Err("jolt needs Phase 13".into()),
    }
    match config.scenario {
        Scenario::Pile
        | Scenario::Grid
        | Scenario::Wave
        | Scenario::Powered
        | Scenario::Shooting => {}
        Scenario::Balance => return Err("balance needs Phase 9".into()),
        Scenario::Mixed => return Err("mixed needs Phase 11".into()),
    }
    if config.mode == DriveMode::Balance {
        return Err("balance mode needs Phase 9".into());
    }
    if config.deterministic {
        return Err("deterministic mode needs Phase 15".into());
    }
    if let Some(path) = cli.screenshot.as_ref() {
        if config.headless {
            return Err("screenshots require a visible run".into());
        }
        if path.as_os_str().is_empty() {
            return Err("screenshot path cannot be empty".into());
        }
    }
    Ok(config)
}

// ---------------------------------------------------------------------------
// Report schema and metrics
// ---------------------------------------------------------------------------

/// Closed report schema vocabulary serialized as the JSON number `1`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(try_from = "u8", into = "u8")]
enum ReportSchema {
    /// Initial stress report representation.
    V1,
}

impl From<ReportSchema> for u8 {
    /// Converts the closed schema version to its JSON representation.
    fn from(schema: ReportSchema) -> Self {
        match schema {
            ReportSchema::V1 => 1,
        }
    }
}

impl TryFrom<u8> for ReportSchema {
    type Error = u8;

    /// Rejects report schema versions not implemented by this binary.
    fn try_from(schema: u8) -> Result<Self, Self::Error> {
        match schema {
            1 => Ok(Self::V1),
            other => Err(other),
        }
    }
}

/// Host details recorded alongside stress measurements.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct MachineInfo {
    /// CPU model name, or the architecture name when the model is unavailable.
    cpu: String,
    /// Number of logical CPUs available to the process.
    cores: usize,
    /// Operating system name reported by Rust's platform constants.
    os: String,
    /// Rendering adapter name when a visible run has a GPU adapter.
    #[serde(skip_serializing_if = "Option::is_none")]
    gpu: Option<String>,
}

/// One complete schema-versioned stress run report.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct StressReport {
    /// Closed schema version for this report's serialized shape.
    schema: ReportSchema,
    /// Short source revision that built the measured process.
    git: String,
    /// Machine description captured by the measured process.
    machine: MachineInfo,
    /// Validated scenario and tuning values used by the process.
    config: RunConfig,
    /// Timing, population, memory, and health measurements.
    metrics: StressMetrics,
}

/// Frame-time percentiles and maximum, all measured in milliseconds.
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct FrameStatistics {
    /// Nearest-rank p50 frame duration in milliseconds.
    p50: f64,
    /// Nearest-rank p95 frame duration in milliseconds.
    p95: f64,
    /// Nearest-rank p99 frame duration in milliseconds.
    p99: f64,
    /// Maximum measured frame duration in milliseconds.
    max: f64,
}

/// Physics-step percentiles and maximum, all measured in milliseconds.
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct StepStatistics {
    /// Nearest-rank p50 fixed-step duration in milliseconds.
    p50: f64,
    /// Nearest-rank p95 fixed-step duration in milliseconds.
    p95: f64,
    /// Maximum measured fixed-step duration in milliseconds.
    max: f64,
}

/// Ragdoll core schedule percentiles, all measured in milliseconds.
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct CoreStatistics {
    /// Nearest-rank p50 core schedule duration in milliseconds.
    p50: f64,
    /// Nearest-rank p95 core schedule duration in milliseconds.
    p95: f64,
}

/// Timing, count, memory, and stability measurements for one run.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct StressMetrics {
    /// Frame timing distribution in milliseconds.
    frame_ms: FrameStatistics,
    /// Physics fixed-step timing distribution in milliseconds.
    step_ms: StepStatistics,
    /// Ragdoll core schedule timing distribution in milliseconds.
    core_ms: CoreStatistics,
    /// Worst frame time within 0.5 seconds after activation, in milliseconds.
    trigger_spike_ms: f64,
    /// Number of spawned characters at the end of the run.
    characters: usize,
    /// Number of ragdoll physics bodies at the end of the run.
    bodies: usize,
    /// Number of ragdoll joints at the end of the run.
    joints: usize,
    /// Number of dynamic physics bodies at the end of the run.
    dynamic: usize,
    /// Number of sleeping bodies at the end of the run.
    sleeping_end: usize,
    /// Number of frozen bodies at the end of the run.
    frozen_end: usize,
    /// Linux peak resident set size in megabytes, omitted on other platforms.
    #[serde(skip_serializing_if = "Option::is_none")]
    peak_rss_mb: Option<f64>,
    /// Number of bodies faster than 50 metres per second or with a non-finite pose.
    unstable_bodies: usize,
}

/// Percentile represented by one of the report's timing summaries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Percentile {
    /// Nearest-rank fiftieth percentile.
    P50,
    /// Nearest-rank ninety-fifth percentile.
    P95,
    /// Nearest-rank ninety-ninth percentile.
    P99,
}

impl Percentile {
    /// Returns the probability used by nearest-rank selection.
    const fn probability(self) -> f64 {
        match self {
            Self::P50 => 0.50,
            Self::P95 => 0.95,
            Self::P99 => 0.99,
        }
    }
}

/// Returns the nearest-rank value for the requested percentile.
///
/// Empty input returns `None`. The nearest-rank index is
/// `ceil(probability × sample_count) - 1`, so p50 of 1 through 100 is 50.
/// The function copies and sorts samples in O(n log n) time.
fn nearest_rank_percentile(samples: &[f64], percentile: Percentile) -> Option<f64> {
    if samples.is_empty() {
        return None;
    }
    let mut ordered = samples.to_vec();
    ordered.sort_by(f64::total_cmp);
    let rank = (percentile.probability() * ordered.len() as f64).ceil() as usize;
    ordered.get(rank.saturating_sub(1)).copied()
}

/// Returns the short checked-out source revision for reproducible reports.
fn source_revision() -> String {
    std::env::var("BEVY_RAGDOLL_GIT_SHA").unwrap_or_else(|_| {
        Command::new("git")
            .args(["rev-parse", "--short", "HEAD"])
            .output()
            .ok()
            .filter(|output| output.status.success())
            .map(|output| String::from_utf8_lossy(&output.stdout).trim().to_owned())
            .filter(|revision| !revision.is_empty())
            .unwrap_or_else(|| "unknown".to_owned())
    })
}

/// Captures CPU, logical-core, operating-system, and optional adapter details.
fn machine_info(gpu: Option<String>) -> MachineInfo {
    let cpu = std::fs::read_to_string("/proc/cpuinfo")
        .ok()
        .and_then(|input| {
            input.lines().find_map(|line| {
                let (key, value) = line.split_once(':')?;
                matches!(key.trim(), "model name" | "Hardware").then(|| value.trim().to_owned())
            })
        })
        .unwrap_or_else(|| std::env::consts::ARCH.to_owned());
    let cores = std::thread::available_parallelism()
        .map(std::num::NonZeroUsize::get)
        .unwrap_or(1);
    MachineInfo {
        cpu,
        cores,
        os: std::env::consts::OS.to_owned(),
        gpu,
    }
}

/// Reads Linux's process high-water RSS and converts kibibytes to mebibytes.
fn peak_rss_megabytes() -> Option<f64> {
    if !cfg!(target_os = "linux") {
        return None;
    }
    let status = std::fs::read_to_string("/proc/self/status").ok()?;
    let kilobytes = status
        .lines()
        .find_map(|line| line.strip_prefix("VmHWM:"))?
        .split_whitespace()
        .next()?
        .parse::<f64>()
        .ok()?;
    Some(kilobytes / 1024.0)
}

/// Serializes one complete report to its requested JSON path.
fn write_json(path: &Path, report: &StressReport) -> Result<(), Box<dyn Error>> {
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
    {
        std::fs::create_dir_all(parent)?;
    }
    std::fs::write(path, serde_json::to_vec_pretty(report)?)?;
    Ok(())
}

/// Reads either one report or a sweep report array.
fn read_reports(path: &Path) -> Result<Vec<StressReport>, Box<dyn Error>> {
    let value = serde_json::from_slice::<serde_json::Value>(&std::fs::read(path)?)?;
    if value.is_array() {
        Ok(serde_json::from_value(value)?)
    } else {
        Ok(vec![serde_json::from_value(value)?])
    }
}

/// Chooses the requested report destination or a target-local temporary JSON file.
///
/// The returned flag is `true` when the caller must remove the temporary file.
fn report_path(cli: &StressCli) -> Result<(PathBuf, bool), Box<dyn Error>> {
    if let Some(path) = cli.report.as_ref() {
        return Ok((path.clone(), false));
    }
    let directory = target_directory().join("stress");
    std::fs::create_dir_all(&directory)?;
    let nonce = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_nanos())
        .unwrap_or_default();
    let process_id = std::process::id();
    Ok((
        directory.join(format!(".run-{process_id}-{nonce}.json")),
        true,
    ))
}

/// Returns `$CARGO_TARGET_DIR`, or `target` when it is unset.
fn target_directory() -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR").map_or_else(|| PathBuf::from("target"), PathBuf::from)
}

/// Prints a report without `--report`, then checks it against `--compare`.
fn write_and_compare(cli: &StressCli, report: &StressReport) -> Result<(), Box<dyn Error>> {
    if cli.report.is_none() {
        let report_json = serde_json::to_string_pretty(report)?;
        println!("{report_json}");
    }
    if let Some(path) = cli.compare.as_ref() {
        let baselines = read_reports(path)?;
        let issues = compare_reports(std::slice::from_ref(report), &baselines);
        if !issues.is_empty() {
            for issue in issues {
                eprintln!("{issue}");
            }
            return Err("stress report exceeded its baseline comparison".into());
        }
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// Baseline comparison
// ---------------------------------------------------------------------------

/// Whether one metric remains within or exceeds the allowed regression limit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Comparison {
    /// Candidate p95 is no more than 10 percent slower than the baseline.
    Pass,
    /// Candidate p95 is more than 10 percent slower than the baseline.
    Regression,
}

/// One configuration or metric failure from a baseline comparison.
#[derive(Clone, Debug, PartialEq, Eq)]
struct ComparisonIssue {
    /// Backend and scenario key associated with the failure.
    run: String,
    /// Metric or lookup operation that failed its comparison.
    metric: String,
    /// Observed values or missing-baseline explanation.
    detail: String,
}

impl Display for ComparisonIssue {
    /// Formats one comparison failure for terminal output.
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        let run = &self.run;
        let metric = &self.metric;
        let detail = &self.detail;
        write!(formatter, "{run} {metric}: {detail}")
    }
}

/// Compares one candidate p95 timing in milliseconds with its baseline.
///
/// An exact 10 percent increase passes. A larger increase fails.
fn compare_p95(candidate_ms: f64, baseline_ms: f64) -> Comparison {
    if candidate_ms <= baseline_ms * 1.10 {
        Comparison::Pass
    } else {
        Comparison::Regression
    }
}

/// Compares p95 frame and step time and simulation health for matching configurations.
fn compare_reports(
    candidates: &[StressReport],
    baselines: &[StressReport],
) -> Vec<ComparisonIssue> {
    let mut issues = Vec::new();
    for candidate in candidates {
        let backend = candidate.config.backend;
        let scenario = candidate.config.scenario;
        let run = format!("{backend} {scenario}");
        let issue = |metric: &str, detail: String| ComparisonIssue {
            run: run.clone(),
            metric: metric.to_owned(),
            detail,
        };
        let Some(baseline) = baselines
            .iter()
            .find(|baseline| baseline.config == candidate.config)
        else {
            issues.push(issue(
                "baseline",
                "no report has the same configuration".to_owned(),
            ));
            continue;
        };
        for (metric, candidate_ms, baseline_ms) in [
            (
                "frame_ms.p95",
                candidate.metrics.frame_ms.p95,
                baseline.metrics.frame_ms.p95,
            ),
            (
                "step_ms.p95",
                candidate.metrics.step_ms.p95,
                baseline.metrics.step_ms.p95,
            ),
        ] {
            if !candidate_ms.is_finite()
                || !baseline_ms.is_finite()
                || candidate_ms < 0.0
                || baseline_ms < 0.0
            {
                issues.push(issue(
                    metric,
                    "timings must be finite and nonnegative".to_owned(),
                ));
            } else if compare_p95(candidate_ms, baseline_ms) == Comparison::Regression {
                issues.push(issue(
                    metric,
                    format!("{candidate_ms:.3} ms exceeds the 10% limit over {baseline_ms:.3} ms"),
                ));
            }
        }
        if candidate.metrics.unstable_bodies > 0 {
            let count = candidate.metrics.unstable_bodies;
            issues.push(issue(
                "unstable_bodies",
                format!("{count} bodies exceeded simulation health limits"),
            ));
        }
    }
    issues
}

// ---------------------------------------------------------------------------
// Sweeps: each row runs in a fresh headless child process
// ---------------------------------------------------------------------------

/// One scenario row read from a RON sweep description.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SweepRow {
    /// Backend override, or the command-line backend when absent.
    backend: Option<Backend>,
    /// Required scenario selection for this child run.
    scenario: Scenario,
    /// Positive grid dimensions for grid-based scenarios.
    grid: Option<[u32; 2]>,
    /// Positive character count for pile and shooting scenarios.
    count: Option<usize>,
}

/// RON sweep file whose rows each run in a child process.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SweepFile {
    /// Child-process rows in execution order.
    runs: Vec<SweepRow>,
}

/// Runs each selected configuration in a fresh headless child process.
fn run_sweep(cli: &StressCli, selection: &SweepSelection) -> Result<(), Box<dyn Error>> {
    if cli.backend != Backend::Rapier3d {
        return Err("the only compiled backend is rapier3d".into());
    }
    if cli.deterministic.is_some() {
        return Err("deterministic mode needs Phase 15".into());
    }
    let rows = match selection {
        SweepSelection::Default => default_rows(),
        SweepSelection::File(path) => {
            ron::from_str::<SweepFile>(&std::fs::read_to_string(path)?)?.runs
        }
    };
    if rows.is_empty() {
        return Err("sweep must contain at least one run".into());
    }
    let output_path = sweep_output_path(cli.report.as_deref());
    let parent = output_path
        .parent()
        .filter(|path| !path.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    std::fs::create_dir_all(parent)?;
    let executable = std::env::current_exe()?;
    let run_id = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_millis())
        .unwrap_or_default();

    let mut reports = Vec::with_capacity(rows.len());
    let total = rows.len();
    for (index, row) in rows.iter().enumerate() {
        let backend = row.backend.unwrap_or(cli.backend);
        let (grid, count) = validate_row(row, cli)?;
        if backend != Backend::Rapier3d {
            return Err(format!("backend {backend} needs a later phase").into());
        }
        let report_path = parent.join(format!(".sweep-{run_id}-{index}.json"));
        let arguments = cli.child_arguments(backend, row.scenario, grid, count, &report_path);
        let run_index = index + 1;
        let scenario = row.scenario;
        let population = count.get();
        println!("[sweep {run_index}/{total}] {backend} {scenario} population {population}");
        let status = Command::new(&executable).args(arguments).status()?;
        if !status.success() {
            let _ = std::fs::remove_file(&report_path);
            return Err(format!("{scenario} child run failed with {status}").into());
        }
        let child_reports = read_reports(&report_path)?;
        let _ = std::fs::remove_file(&report_path);
        let report = child_reports
            .into_iter()
            .next()
            .ok_or("child process wrote an empty report")?;
        reports.push(report);
    }

    std::fs::write(&output_path, serde_json::to_vec_pretty(&reports)?)?;
    print_summary(&reports);
    let output_path_display = output_path.display();
    println!("sweep report: {output_path_display}");

    if let Some(baseline_path) = cli.compare.as_ref() {
        let baselines = read_reports(baseline_path)?;
        let issues = compare_reports(&reports, &baselines);
        if !issues.is_empty() {
            for issue in issues {
                eprintln!("{issue}");
            }
            return Err("sweep exceeded its baseline comparison".into());
        }
        println!("baseline comparison passed");
    }
    Ok(())
}

/// Returns the default sweep rows and prints the rows that need later phases.
fn default_rows() -> Vec<SweepRow> {
    let row = |scenario, grid, count| SweepRow {
        backend: None,
        scenario,
        grid,
        count,
    };
    let mut rows = Vec::with_capacity(9);
    for size in [8, 16, 24, 32] {
        rows.push(row(Scenario::Grid, Some([size, size]), None));
    }
    for count in [32, 64, 128] {
        rows.push(row(Scenario::Pile, None, Some(count)));
    }
    rows.push(row(Scenario::Powered, Some([16, 16]), None));
    rows.push(row(Scenario::Shooting, None, Some(64)));
    println!("skip balance 8x8: needs Phase 9");
    println!("skip mixed: needs Phase 11");
    rows
}

/// Resolves one row's grid and count, falling back to the command-line values.
fn validate_row(
    row: &SweepRow,
    defaults: &StressCli,
) -> Result<(GridSize, CharacterCount), Box<dyn Error>> {
    let [columns, rows] = row
        .grid
        .unwrap_or(defaults.grid.as_array().map(NonZeroU32::get));
    let grid = GridSize::try_new(columns, rows)?;
    let count = match row.scenario {
        Scenario::Pile | Scenario::Shooting => {
            CharacterCount::try_new(row.count.unwrap_or(defaults.count.get()))?
        }
        Scenario::Grid
        | Scenario::Wave
        | Scenario::Powered
        | Scenario::Balance
        | Scenario::Mixed => {
            let population = usize::try_from(columns)
                .ok()
                .and_then(|columns| {
                    usize::try_from(rows)
                        .ok()
                        .and_then(|rows| columns.checked_mul(rows))
                })
                .ok_or("sweep grid population exceeds the supported range")?;
            CharacterCount::try_new(population)?
        }
    };
    Ok((grid, count))
}

/// Chooses the requested sweep path or `<target>/stress/<date>-<revision>.json`.
fn sweep_output_path(requested: Option<&Path>) -> PathBuf {
    if let Some(path) = requested {
        return path.to_path_buf();
    }
    let date = Command::new("date")
        .arg("+%F")
        .output()
        .ok()
        .filter(|output| output.status.success())
        .map(|output| String::from_utf8_lossy(&output.stdout).trim().to_owned())
        .filter(|value| !value.is_empty())
        .unwrap_or_else(|| {
            SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .map(|duration| duration.as_secs().to_string())
                .unwrap_or_else(|_| "unknown-date".to_owned())
        });
    let revision = source_revision();
    target_directory()
        .join("stress")
        .join(format!("{date}-{revision}.json"))
}

/// Prints a Markdown table for one ordered report set.
fn print_summary(reports: &[StressReport]) {
    println!(
        "| backend | scenario | characters | frame p95 ms | step p95 ms | core p95 ms | trigger spike ms | bodies | unstable |"
    );
    println!("|---|---:|---:|---:|---:|---:|---:|---:|---:|");
    for report in reports {
        let backend = report.config.backend;
        let scenario = report.config.scenario;
        let metrics = &report.metrics;
        let characters = metrics.characters;
        let frame_p95 = metrics.frame_ms.p95;
        let step_p95 = metrics.step_ms.p95;
        let core_p95 = metrics.core_ms.p95;
        let trigger_spike = metrics.trigger_spike_ms;
        let bodies = metrics.bodies;
        let unstable = metrics.unstable_bodies;
        println!(
            "| {backend} | {scenario} | {characters} | {frame_p95:.3} | {step_p95:.3} | {core_p95:.3} | {trigger_spike:.3} | {bodies} | {unstable} |"
        );
    }
}

// ---------------------------------------------------------------------------
// App setup
// ---------------------------------------------------------------------------

/// Validated scene inputs shared by startup and measurement systems.
#[derive(Clone, Debug, Resource)]
struct RunScene {
    /// Validated TGF profile used by every spawned character.
    profile: RagdollProfile,
    /// Serialized scenario values used by startup and report collection.
    config: RunConfig,
    /// JSON destination written by the app's final schedule.
    report_path: PathBuf,
    /// Optional PNG destination for a visible run.
    screenshot: Option<PathBuf>,
}

/// Grid row of one character, used to delay wave activation.
#[derive(Clone, Copy, Debug, Component)]
struct ScenarioCharacter {
    /// Zero-based grid row.
    row: u32,
}

/// Manual trigger, freeze, and completion state for the current app.
#[derive(Debug, Default, Resource)]
struct RunControl {
    /// Simulation time selected by Space, overriding the configured trigger time.
    manual_trigger: Option<Duration>,
    /// Time of the first activation trigger in this run.
    triggered_at: Option<Duration>,
    /// Whether all ragdoll modes are frozen by the F key.
    freeze_all: bool,
    /// Whether the configured measurement window has ended.
    finished: bool,
    /// Whether the final report was written successfully.
    report_written: bool,
    /// Whether report output or screenshot creation failed.
    report_failed: bool,
    /// Whether the app exit message was written.
    exit_requested: bool,
    /// Simulation time when measurement and playback end.
    end_at: Option<Duration>,
    /// Simulation time when the screenshot was requested.
    screenshot_requested_at: Option<Duration>,
    /// Wall-clock time when the screenshot was requested.
    screenshot_started_at: Option<Instant>,
}

/// Wall-clock samples collected around the app and backend schedules.
#[derive(Debug, Default, Resource)]
struct RunMeasurements {
    /// App update wall times in milliseconds after warmup.
    frame_ms: Vec<f64>,
    /// Rapier simulation-set wall times in milliseconds after warmup.
    step_ms: Vec<f64>,
    /// Ragdoll schedule wall times accumulated once per app update.
    core_ms: Vec<f64>,
    /// Ragdoll schedule time accumulated during the current app update.
    core_frame: Duration,
    /// Largest app update in the half-second trigger window, in milliseconds.
    trigger_spike_ms: f64,
}

/// Start time shared by the before and after physics-step systems.
#[derive(Debug, Default, Resource)]
struct StepTimer(
    /// Start of the current physics step.
    Option<Instant>,
);

/// Start time shared by the ragdoll core schedule measurement systems.
#[derive(Debug, Default, Resource)]
struct CoreTimer(
    /// Start of the current ragdoll schedule interval.
    Option<Instant>,
);

/// Start time shared by the first and last schedule frame timers.
#[derive(Debug, Default, Resource)]
struct FrameTimer(
    /// Start of the current app update.
    Option<Instant>,
);

/// Orders the variable-rate core timer pair around the ragdoll sets.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, SystemSet)]
enum StressCoreSchedule {
    /// Begins measuring the variable-rate ragdoll sets.
    VariableStart,
    /// Ends measuring the variable-rate ragdoll sets.
    VariableFinish,
}

/// Builds and runs one measured Bevy app, then reads back its report.
fn run_app(
    cli: &StressCli,
    config: RunConfig,
    profile: RagdollProfile,
    report_path: &Path,
) -> Result<StressReport, Box<dyn Error>> {
    let step = Duration::from_secs_f64(1.0 / f64::from(config.fixed_hz));
    let substeps = usize::try_from(config.substeps.unwrap_or(1))?;
    let headless = config.headless;
    let scenario = config.scenario;
    let seed = config.seed;
    let fixed_hz = f64::from(config.fixed_hz);
    let budget = config.budget.unwrap_or(usize::MAX);

    let mut app = App::new();
    if headless {
        app.add_plugins((
            MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::ZERO)),
            AssetPlugin::default(),
            TransformPlugin,
        ));
    } else {
        app.add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "bevy-ragdoll stress".to_owned(),
                resolution: (1600, 1000).into(),
                present_mode: PresentMode::AutoNoVsync,
                ..default()
            }),
            ..default()
        }))
        .insert_resource(WinitSettings::continuous())
        .add_systems(Startup, setup_visual_scene)
        .add_systems(
            PostUpdate,
            create_body_visuals
                .after(RagdollSystems::Bind)
                .before(TransformSystems::Propagate),
        )
        .add_systems(Update, handle_keyboard.before(update_population_state))
        .add_systems(Update, update_overlay.after(update_population_state))
        .add_systems(Last, request_screenshot.before(finish_frame_sample));
    }

    // Drive time manually so every frame advances exactly one fixed step.
    app.insert_resource(RunScene {
        profile,
        config,
        report_path: report_path.to_path_buf(),
        screenshot: cli.screenshot.clone(),
    })
    .insert_resource(Time::<Fixed>::from_hz(fixed_hz))
    .insert_resource(TimeUpdateStrategy::ManualDuration(step))
    .insert_resource(TimestepMode::Fixed {
        dt: step.as_secs_f32(),
        substeps,
    })
    .insert_resource(RagdollBudget::new(budget))
    .init_resource::<RunControl>()
    .init_resource::<RunMeasurements>()
    .init_resource::<StepTimer>()
    .init_resource::<CoreTimer>()
    .init_resource::<FrameTimer>()
    .add_plugins((
        RagdollPlugin::default(),
        RapierPhysicsPlugin::<RapierRagdollHooks>::default().in_fixed_schedule(),
        RapierRagdollPlugin,
        FrameTimeDiagnosticsPlugin::new(4096),
    ))
    .add_systems(Startup, (spawn_ground, spawn_population))
    .add_systems(Update, update_population_state)
    .add_systems(Last, (finish_frame_sample, finalize_report).chain());

    if scenario == Scenario::Shooting {
        app.insert_resource(ShootingState::new(seed))
            .add_systems(
                PostUpdate,
                cache_shooting_targets.after(RagdollSystems::Writeback),
            )
            .add_systems(
                FixedUpdate,
                shoot_ragdolls.before(RagdollFixedSystems::Behaviour),
            );
    }

    // Wall-clock timers around the whole frame, the physics step, and the
    // fixed- and variable-rate ragdoll schedules.
    app.add_systems(First, start_frame_timer)
        .add_systems(
            FixedUpdate,
            (
                start_step_timer.before(PhysicsSet::StepSimulation),
                finish_step_timer.after(PhysicsSet::StepSimulation),
                start_core_timer.before(RagdollFixedSystems::Drive),
                finish_core_timer.after(RagdollFixedSystems::AfterStep),
            ),
        )
        .configure_sets(
            PostUpdate,
            (
                StressCoreSchedule::VariableStart,
                StressCoreSchedule::VariableFinish,
            )
                .chain(),
        )
        .add_systems(
            PostUpdate,
            (
                start_core_timer
                    .before(RagdollSystems::Bind)
                    .in_set(StressCoreSchedule::VariableStart),
                finish_core_timer
                    .after(RagdollSystems::Writeback)
                    .in_set(StressCoreSchedule::VariableFinish),
            ),
        );

    if !matches!(app.run(), AppExit::Success) {
        return Err("stress application exited with an error".into());
    }
    if let Some(path) = cli.screenshot.as_ref()
        && !path.is_file()
    {
        let path = path.display();
        return Err(format!("screenshot was not written to {path}").into());
    }
    read_reports(report_path)?
        .into_iter()
        .next()
        .ok_or_else(|| "stress application did not write a report".into())
}

/// Adds a static floor and simple obstacles that stress contact generation.
fn spawn_ground(mut commands: Commands) {
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(100.0, 0.1, 100.0),
        Transform::from_xyz(0.0, -0.1, 0.0),
    ));
    for x in [-3.0, 3.0] {
        commands.spawn((
            RigidBody::Fixed,
            Collider::cuboid(0.6, 0.4, 0.6),
            Transform::from_xyz(x, 0.4, -x * 0.5),
        ));
    }
    commands.spawn((
        RigidBody::Fixed,
        Collider::cuboid(1.5, 0.1, 2.5),
        Transform::from_xyz(5.0, 0.35, 0.0).with_rotation(Quat::from_rotation_z(-0.25)),
    ));
}

/// Creates shared-profile character hierarchies and selects each initial body mode.
fn spawn_population(
    mut commands: Commands,
    mut profiles: ResMut<Assets<RagdollProfile>>,
    scene: Res<RunScene>,
) {
    let profile_handle = profiles.add(scene.profile.clone());
    let [columns, _] = scene.config.grid;
    let initial_mode = match scene.config.scenario {
        Scenario::Pile | Scenario::Powered => RagdollMode::Dynamic,
        Scenario::Grid | Scenario::Wave => RagdollMode::Kinematic,
        Scenario::Balance | Scenario::Shooting | Scenario::Mixed => RagdollMode::Animated,
    };
    let drive = if scene.config.scenario == Scenario::Powered {
        RagdollDrive::new(1.0, 0.5)
    } else {
        RagdollDrive::new(0.0, 0.0)
    };
    for index in 0..scene.config.count {
        let column = u32::try_from(index).unwrap_or(u32::MAX) % columns;
        let row = u32::try_from(index).unwrap_or(u32::MAX) / columns;
        let position = character_position(&scene.config, index, column, row);
        let character = commands
            .spawn((
                Name::new(format!("stress human {index}")),
                Ragdoll::new(profile_handle.clone()),
                initial_mode,
                drive,
                ScenarioCharacter { row },
                Transform::from_translation(position),
            ))
            .id();
        spawn_profile_bones(&mut commands, character, &scene.profile);
    }
}

/// Computes the character root position for the selected scenario.
///
/// Piles stack vertically, grids are centered on the origin, and shooting
/// targets stand on a ring whose circumference is `count × spacing`.
fn character_position(config: &RunConfig, index: usize, column: u32, row: u32) -> Vec3 {
    match config.scenario {
        Scenario::Pile => Vec3::new(0.0, 1.5 + index as f32 * 0.8, 0.0),
        Scenario::Grid
        | Scenario::Wave
        | Scenario::Powered
        | Scenario::Balance
        | Scenario::Mixed => {
            let columns = config.grid[0] as f32;
            let rows = config.grid[1] as f32;
            let x = (column as f32 - (columns - 1.0) * 0.5) * config.spacing_m;
            let z = (row as f32 - (rows - 1.0) * 0.5) * config.spacing_m;
            Vec3::new(x, 0.0, z)
        }
        Scenario::Shooting => {
            let count = config.count as f32;
            let angle = std::f32::consts::TAU * index as f32 / count;
            let radius = config.spacing_m * count / std::f32::consts::TAU;
            Vec3::new(radius * angle.cos(), 0.0, radius * angle.sin())
        }
    }
}

/// Builds named profile bones in parent-first order so the runtime can bind them.
fn spawn_profile_bones(commands: &mut Commands, character: Entity, profile: &RagdollProfile) {
    let bodies = profile.bodies();
    let mut parents = vec![None; bodies.len()];
    for joint in profile.joints() {
        parents[joint.child().get()] = Some(joint.parent().get());
    }
    let mut bones = Vec::<Entity>::with_capacity(bodies.len());
    for (index, body) in bodies.iter().enumerate() {
        let rest = body.rest();
        // Root bones use their rest pose directly; children are parent-relative.
        let (parent, transform) = match parents[index] {
            None => (character, transform_from_isometry(rest)),
            Some(parent_index) => {
                let parent_pose = bodies[parent_index].rest();
                let inverse = parent_pose.rotation.inverse();
                let translation = inverse * (rest.translation - parent_pose.translation);
                (
                    bones[parent_index],
                    Transform::from_translation(translation.into())
                        .with_rotation(inverse * rest.rotation),
                )
            }
        };
        let bone = commands
            .spawn((
                Name::new(body.bone().to_owned()),
                transform,
                ChildOf(parent),
            ))
            .id();
        bones.push(bone);
    }
}

/// Converts a validated rigid profile pose into a Bevy transform.
fn transform_from_isometry(pose: Isometry3d) -> Transform {
    Transform::from_translation(pose.translation.into()).with_rotation(pose.rotation)
}

// ---------------------------------------------------------------------------
// Scenario control
// ---------------------------------------------------------------------------

/// Applies activation timing, drive mode, wave offsets, and total run duration.
fn update_population_state(
    time: Res<Time>,
    scene: Res<RunScene>,
    mut control: ResMut<RunControl>,
    mut time_update: ResMut<TimeUpdateStrategy>,
    mut app_exit: MessageWriter<AppExit>,
    mut characters: Query<(&ScenarioCharacter, &mut RagdollMode, &mut RagdollDrive)>,
) {
    let config = &scene.config;
    let starts_dynamic = matches!(config.scenario, Scenario::Pile | Scenario::Powered);
    let configured_trigger = if starts_dynamic {
        Duration::ZERO
    } else {
        Duration::from_secs_f64(config.trigger_at_s)
    };
    let trigger_at = control.manual_trigger.unwrap_or(configured_trigger);

    // Waves activate one row every 0.1 s, so measurement waits for the last row.
    let row_delay = |row: u32| {
        if config.scenario == Scenario::Wave {
            Duration::from_secs_f64(f64::from(row) * 0.1)
        } else {
            Duration::ZERO
        }
    };
    let wave_tail = row_delay(config.grid[1].saturating_sub(1));
    let measurement_start =
        Duration::from_secs_f64(config.warmup_s).max(trigger_at.saturating_add(wave_tail));
    let end_at = measurement_start.saturating_add(Duration::from_secs_f64(config.duration_s));
    if time.elapsed() >= trigger_at && control.triggered_at.is_none() {
        control.triggered_at = Some(trigger_at);
    }

    let powered = config.scenario == Scenario::Powered || config.mode == DriveMode::Powered;
    for (character, mut mode, mut drive) in &mut characters {
        let active =
            starts_dynamic || time.elapsed() >= trigger_at.saturating_add(row_delay(character.row));
        let target_mode = if control.freeze_all {
            RagdollMode::Frozen
        } else if active {
            RagdollMode::Dynamic
        } else {
            RagdollMode::Kinematic
        };
        if *mode != target_mode {
            *mode = target_mode;
        }
        if target_mode == RagdollMode::Dynamic {
            let (muscle, pin) = if powered { (1.0, 0.5) } else { (0.0, 0.0) };
            drive.set(muscle, pin);
        }
    }

    control.end_at = Some(end_at);
    control.finished = time.elapsed() >= end_at;
    if !control.finished {
        return;
    }
    // Stop simulation time, then exit once the report and screenshot are done.
    *time_update = TimeUpdateStrategy::ManualDuration(Duration::ZERO);
    if let Some(started) = control.screenshot_started_at
        && started.elapsed() >= Duration::from_secs(30)
        && scene
            .screenshot
            .as_ref()
            .is_some_and(|path| !path.is_file())
    {
        control.report_failed = true;
    }
    let screenshot_complete = scene
        .screenshot
        .as_ref()
        .is_none_or(|path| control.screenshot_requested_at.is_some() && path.is_file());
    if !control.exit_requested
        && (control.report_written || control.report_failed)
        && (screenshot_complete || control.report_failed)
    {
        app_exit.write(AppExit::Success);
        control.exit_requested = true;
    }
}

// ---------------------------------------------------------------------------
// Shooting scenario
// ---------------------------------------------------------------------------

/// Fixed simulation interval between deterministic shooting hits.
const SHOOTING_INTERVAL: Duration = Duration::from_millis(50);

/// Cached targets and deterministic clock state for the shooting scenario.
#[derive(Debug, Resource)]
struct ShootingState {
    /// Seeded generator used to choose a character and one profile body.
    random: SplitMix64,
    /// Fixed-step time carried toward the next 50 ms firing interval.
    elapsed: Duration,
    /// Profile-ordered body entities grouped by stable character entity order.
    targets: Option<Vec<Vec<Entity>>>,
    /// Number of scheduled rifle hits since the run began.
    scheduled_hits: usize,
    /// Number of hit messages written to Bevy's message queue.
    messages_written: usize,
}

impl ShootingState {
    /// Starts a shooting scenario with a fixed seed and no cached body targets.
    fn new(seed: u64) -> Self {
        Self {
            random: SplitMix64::new(seed),
            elapsed: Duration::ZERO,
            targets: None,
            scheduled_hits: 0,
            messages_written: 0,
        }
    }
}

/// Small deterministic pseudo-random generator for reproducible stress inputs.
#[derive(Debug)]
struct SplitMix64 {
    /// Current state advanced by the generator's fixed arithmetic sequence.
    state: u64,
}

impl SplitMix64 {
    /// Initializes the generator without reserving a special seed value.
    fn new(seed: u64) -> Self {
        Self { state: seed }
    }

    /// Produces one deterministic 64-bit sample.
    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut value = self.state;
        value = (value ^ (value >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        value = (value ^ (value >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        value ^ (value >> 31)
    }

    /// Chooses an index below `upper_bound`, returning `None` for an empty range.
    fn index(&mut self, upper_bound: usize) -> Option<usize> {
        let upper_bound = u64::try_from(upper_bound).ok()?;
        if upper_bound == 0 {
            return None;
        }
        usize::try_from(self.next_u64() % upper_bound).ok()
    }
}

/// One profile-ordered body row copied while the shooting cache is built.
#[derive(Clone, Copy, Debug)]
struct ShootingBodyRecord {
    /// Character root that owns the physics body.
    character: Entity,
    /// Validated profile index used to stabilize body selection order.
    index: BodyIndex,
    /// Physics entity that receives a selected hit.
    body: Entity,
}

/// Groups a complete body snapshot by character and profile body index.
///
/// Returns `None` unless every character has exactly one body per profile index.
fn group_shooting_targets(
    mut characters: Vec<Entity>,
    mut bodies: Vec<ShootingBodyRecord>,
    body_count: usize,
) -> Option<Vec<Vec<Entity>>> {
    if characters.is_empty() || body_count == 0 {
        return None;
    }
    if bodies.len() != characters.len().checked_mul(body_count)? {
        return None;
    }

    // Sort both dimensions so seeded choices do not depend on ECS query order.
    characters.sort_unstable_by_key(|character| character.to_bits());
    bodies.sort_unstable_by_key(|record| (record.character.to_bits(), record.index.get()));

    // Accept only complete profile-indexed rows for every expected character.
    let mut records = bodies.into_iter();
    let mut targets = Vec::with_capacity(characters.len());
    for character in characters {
        let mut character_bodies = Vec::with_capacity(body_count);
        for index in 0..body_count {
            let record = records.next()?;
            if record.character != character || record.index.get() != index {
                return None;
            }
            character_bodies.push(record.body);
        }
        targets.push(character_bodies);
    }
    Some(targets)
}

/// Carries fixed-step fractions and returns the number of due shooting hits.
fn take_due_shots(elapsed: &mut Duration, delta: Duration) -> usize {
    *elapsed = elapsed.saturating_add(delta);
    let mut due = 0;
    while *elapsed >= SHOOTING_INTERVAL {
        *elapsed = elapsed.saturating_sub(SHOOTING_INTERVAL);
        due += 1;
    }
    due
}

/// Builds the shooting target cache once every character has all profile bodies.
fn cache_shooting_targets(
    scene: Res<RunScene>,
    mut shooting: ResMut<ShootingState>,
    characters: Query<Entity, (With<Ragdoll>, With<ScenarioCharacter>)>,
    bodies: Query<(Entity, &RagdollBodyOf, &BodyIndex), With<BodyShape>>,
) {
    if shooting.targets.is_some() {
        return;
    }
    let characters: Vec<Entity> = characters.iter().collect();
    if characters.len() != scene.config.count {
        return;
    }
    let body_records = bodies
        .iter()
        .map(|(body, owner, index)| ShootingBodyRecord {
            character: owner.0,
            index: *index,
            body,
        })
        .collect();
    shooting.targets =
        group_shooting_targets(characters, body_records, scene.profile.bodies().len());
}

/// Writes one deterministic random rifle hit for each due 50 ms interval.
fn shoot_ragdolls(
    control: Res<RunControl>,
    time: Res<Time<Fixed>>,
    settings: Res<HitSettings>,
    mut shooting: ResMut<ShootingState>,
    bodies: Query<&BodyPhysicsPose, With<BodyShape>>,
    mut hits: MessageWriter<RagdollHit>,
) {
    if control.triggered_at.is_none() || control.finished {
        return;
    }
    let Some(magnitude) = settings.impulse_magnitude(HitProfile::Rifle) else {
        return;
    };
    let ShootingState {
        random,
        elapsed,
        targets,
        scheduled_hits,
        messages_written,
    } = &mut *shooting;
    let Some(targets) = targets.as_ref().filter(|targets| !targets.is_empty()) else {
        return;
    };

    // Preserve fractional fixed-step time and emit every interval after stalls.
    let due = take_due_shots(elapsed, time.delta());
    *scheduled_hits = scheduled_hits.saturating_add(due);
    for _ in 0..due {
        let Some(character_bodies) = random.index(targets.len()).map(|index| &targets[index])
        else {
            return;
        };
        let Some(body) = random
            .index(character_bodies.len())
            .map(|index| character_bodies[index])
        else {
            continue;
        };
        let Ok(pose) = bodies.get(body) else {
            continue;
        };
        hits.write(RagdollHit {
            body,
            point: pose.current.translation.into(),
            impulse: Vec3::NEG_Z * magnitude,
            kind: HitKind::Impact,
        });
        *messages_written = messages_written.saturating_add(1);
    }
}

// ---------------------------------------------------------------------------
// Measurement
// ---------------------------------------------------------------------------

/// Records the start of a Rapier fixed simulation step.
fn start_step_timer(mut timer: ResMut<StepTimer>) {
    timer.0 = Some(Instant::now());
}

/// Records one Rapier fixed simulation-step duration while measurement is active.
fn finish_step_timer(
    time: Res<Time>,
    scene: Res<RunScene>,
    control: Res<RunControl>,
    mut timer: ResMut<StepTimer>,
    mut measurements: ResMut<RunMeasurements>,
) {
    let Some(started) = timer.0.take() else {
        return;
    };
    if time.elapsed().as_secs_f64() >= scene.config.warmup_s
        && control
            .end_at
            .is_some_and(|end_at| time.elapsed() <= end_at)
    {
        measurements
            .step_ms
            .push(started.elapsed().as_secs_f64() * 1000.0);
    }
}

/// Records the start of a ragdoll core schedule interval.
fn start_core_timer(mut timer: ResMut<CoreTimer>) {
    timer.0 = Some(Instant::now());
}

/// Adds one ragdoll core schedule interval to the current app frame.
fn finish_core_timer(mut timer: ResMut<CoreTimer>, mut measurements: ResMut<RunMeasurements>) {
    if let Some(started) = timer.0.take() {
        measurements.core_frame = measurements.core_frame.saturating_add(started.elapsed());
    }
}

/// Starts wall-clock measurement for one complete app update.
fn start_frame_timer(mut timer: ResMut<FrameTimer>) {
    timer.0 = Some(Instant::now());
}

/// Records frame, core, and trigger-spike samples after one complete app update.
///
/// Visible runs use Bevy's frame-time diagnostic so the sample includes rendering.
fn finish_frame_sample(
    time: Res<Time>,
    scene: Res<RunScene>,
    control: Res<RunControl>,
    diagnostics: Option<Res<DiagnosticsStore>>,
    mut timer: ResMut<FrameTimer>,
    mut measurements: ResMut<RunMeasurements>,
) {
    let Some(started) = timer.0.take() else {
        return;
    };
    let elapsed = time.elapsed();
    let wall_frame_ms = started.elapsed().as_secs_f64() * 1000.0;
    let frame_ms = if scene.config.headless {
        wall_frame_ms
    } else {
        diagnostics
            .and_then(|store| {
                store
                    .get_measurement(&FrameTimeDiagnosticsPlugin::FRAME_TIME)
                    .map(|measurement| measurement.value)
            })
            .unwrap_or(wall_frame_ms)
    };
    let core_ms = std::mem::take(&mut measurements.core_frame).as_secs_f64() * 1000.0;
    let measuring = !control.report_written
        && elapsed.as_secs_f64() >= scene.config.warmup_s
        && control.end_at.is_some_and(|end_at| elapsed <= end_at);
    if !measuring {
        return;
    }
    measurements.frame_ms.push(frame_ms);
    measurements.core_ms.push(core_ms);
    if control.triggered_at.is_some_and(|trigger| {
        elapsed >= trigger && elapsed <= trigger.saturating_add(Duration::from_millis(500))
    }) {
        measurements.trigger_spike_ms = measurements.trigger_spike_ms.max(frame_ms);
    }
}

/// Writes the final report once the measurement window ends.
///
/// A shooting run without one accepted hit fails instead of writing a report.
fn finalize_report(world: &mut World) {
    let control = world.resource::<RunControl>();
    if !control.finished || control.report_written || control.report_failed {
        return;
    }
    let scene = world.resource::<RunScene>();
    let config = scene.config.clone();
    let report_path = scene.report_path.clone();
    if config.scenario == Scenario::Shooting && accepted_shooting_hit_count(world) == 0 {
        let shooting = world.resource::<ShootingState>();
        let cached_characters = shooting.targets.as_ref().map_or(0, Vec::len);
        let scheduled_hits = shooting.scheduled_hits;
        let messages_written = shooting.messages_written;
        error!(
            cached_characters,
            scheduled_hits, messages_written, "shooting completed without an accepted rifle hit"
        );
        world.resource_mut::<RunControl>().report_failed = true;
        return;
    }
    let gpu = world
        .get_resource::<RenderAdapterInfo>()
        .map(|info| info.name.clone());
    let report = StressReport {
        schema: ReportSchema::V1,
        git: source_revision(),
        machine: machine_info(gpu),
        config,
        metrics: collect_metrics(world),
    };
    match write_json(&report_path, &report) {
        Ok(()) => world.resource_mut::<RunControl>().report_written = true,
        Err(report_error) => {
            error!(%report_error, "could not write stress report");
            world.resource_mut::<RunControl>().report_failed = true;
        }
    }
}

/// Counts characters whose hit processing accepted at least one shooting hit.
fn accepted_shooting_hit_count(world: &mut World) -> usize {
    world
        .query_filtered::<&LastHit, With<Ragdoll>>()
        .iter(world)
        .filter(|last_hit| last_hit.last_at().is_some())
        .count()
}

/// Counts final ragdoll state and summarizes all timing samples.
fn collect_metrics(world: &mut World) -> StressMetrics {
    let measurements = world.resource::<RunMeasurements>();
    let percentile =
        |samples: &[f64], percentile| nearest_rank_percentile(samples, percentile).unwrap_or(0.0);
    let maximum = |samples: &[f64]| samples.iter().copied().fold(0.0, f64::max);
    let frame_ms = FrameStatistics {
        p50: percentile(&measurements.frame_ms, Percentile::P50),
        p95: percentile(&measurements.frame_ms, Percentile::P95),
        p99: percentile(&measurements.frame_ms, Percentile::P99),
        max: maximum(&measurements.frame_ms),
    };
    let step_ms = StepStatistics {
        p50: percentile(&measurements.step_ms, Percentile::P50),
        p95: percentile(&measurements.step_ms, Percentile::P95),
        max: maximum(&measurements.step_ms),
    };
    let core_ms = CoreStatistics {
        p50: percentile(&measurements.core_ms, Percentile::P50),
        p95: percentile(&measurements.core_ms, Percentile::P95),
    };
    let trigger_spike_ms = measurements.trigger_spike_ms;

    let characters = world
        .query_filtered::<(), With<Ragdoll>>()
        .iter(world)
        .count();
    let joints = world
        .query_filtered::<(), With<JointToParent>>()
        .iter(world)
        .count();
    let mut metrics = StressMetrics {
        frame_ms,
        step_ms,
        core_ms,
        trigger_spike_ms,
        characters,
        joints,
        peak_rss_mb: peak_rss_megabytes(),
        ..default()
    };
    // A body is unstable above 50 m/s or with a non-finite velocity or pose.
    let mut bodies = world.query_filtered::<(
        &BodyKind,
        Has<BodyAtRest>,
        &BodyVelocity,
        &BodyPhysicsPose,
    ), With<BodyShape>>();
    for (kind, at_rest, velocity, pose) in bodies.iter(world) {
        metrics.bodies += 1;
        match kind {
            BodyKind::Dynamic => metrics.dynamic += 1,
            BodyKind::Kinematic => {}
            BodyKind::Fixed => metrics.frozen_end += 1,
        }
        if at_rest {
            metrics.sleeping_end += 1;
        }
        if !velocity.linear.is_finite()
            || velocity.linear.length() > 50.0
            || !pose.current.translation.is_finite()
            || !pose.current.rotation.is_finite()
        {
            metrics.unstable_bodies += 1;
        }
    }
    metrics
}

// ---------------------------------------------------------------------------
// Visible mode
// ---------------------------------------------------------------------------

/// Overlay text that lists the keyboard controls.
const CONTROLS: &str = "Space trigger  F freeze";

/// Mesh cache key that shares one rendered mesh for each exact shape size.
///
/// Sizes are stored as IEEE-754 bit patterns so the key can derive `Hash`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
enum MeshKey {
    /// Capsule radius and segment length bits.
    Capsule {
        /// Capsule radius in metres.
        radius_bits: u32,
        /// Capsule cylinder length in metres.
        length_bits: u32,
    },
    /// Sphere radius bits.
    Sphere {
        /// Sphere radius in metres.
        radius_bits: u32,
    },
    /// Cuboid half-extent bits.
    Cuboid {
        /// Half extents in metres, in x, y, z order.
        half_extent_bits: [u32; 3],
    },
}

/// Shared rendered meshes and one material for every ragdoll body.
#[derive(Debug, Resource)]
struct SharedBodyAssets {
    /// One mesh handle for each distinct capsule, sphere, or cuboid size.
    meshes: HashMap<MeshKey, Handle<Mesh>>,
    /// Material shared by all ragdoll body render entities.
    material: Handle<StandardMaterial>,
}

/// Marker for the corner overlay's text entity.
#[derive(Clone, Copy, Debug, Component)]
struct StressOverlay;

/// Adds the camera, light, ground mesh, shared body material, and overlay.
fn setup_visual_scene(
    mut commands: Commands,
    scene: Res<RunScene>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 22.0, 30.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.spawn((
        DirectionalLight {
            illuminance: 14_000.0,
            shadow_maps_enabled: true,
            ..default()
        },
        Transform::from_xyz(-8.0, 16.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
    commands.insert_resource(SharedBodyAssets {
        meshes: HashMap::new(),
        material: materials.add(StandardMaterial {
            base_color: Color::srgb(0.18, 0.62, 0.76),
            metallic: 0.02,
            perceptual_roughness: 0.42,
            ..default()
        }),
    });
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(200.0, 200.0))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.11, 0.15, 0.19),
            perceptual_roughness: 0.92,
            ..default()
        })),
        Transform::from_xyz(0.0, -0.02, 0.0),
    ));
    let scenario = scene.config.scenario;
    commands.spawn((
        Text::new(format!("{scenario} · {CONTROLS}")),
        TextFont {
            font_size: FontSize::Px(20.0),
            ..default()
        },
        TextColor(Color::WHITE),
        StressOverlay,
        Node {
            position_type: PositionType::Absolute,
            top: px(18),
            left: px(20),
            ..default()
        },
    ));
}

/// Attaches a cached mesh and the shared material to each new physics body.
fn create_body_visuals(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut cache: ResMut<SharedBodyAssets>,
    bodies: Query<(Entity, &BodyShape), Added<BodyShape>>,
) {
    for (body, shape) in &bodies {
        // Give the physics parent the visibility state inherited by its mesh child.
        commands.entity(body).insert(Visibility::Inherited);
        let (key, transform) = shape_mesh_key(&shape.0);
        let mesh = cache
            .meshes
            .entry(key)
            .or_insert_with(|| meshes.add(mesh_from_key(key)))
            .clone();
        commands.spawn((
            Mesh3d(mesh),
            MeshMaterial3d(cache.material.clone()),
            Visibility::Inherited,
            transform,
            ChildOf(body),
        ));
    }
}

/// Converts a validated shape into a shared mesh key and its body-local pose.
fn shape_mesh_key(shape: &ShapeSpec) -> (MeshKey, Transform) {
    match shape {
        ShapeSpec::Capsule { a, b, radius } => {
            let segment = *b - *a;
            let orientation = if segment.length_squared() > f32::EPSILON {
                Quat::from_rotation_arc(Vec3::Y, segment.normalize())
            } else {
                Quat::IDENTITY
            };
            (
                MeshKey::Capsule {
                    radius_bits: radius.to_bits(),
                    length_bits: segment.length().to_bits(),
                },
                Transform::from_translation((*a + *b) * 0.5).with_rotation(orientation),
            )
        }
        ShapeSpec::Sphere { center, radius } => (
            MeshKey::Sphere {
                radius_bits: radius.to_bits(),
            },
            Transform::from_translation(*center),
        ),
        ShapeSpec::Cuboid {
            center,
            rotation,
            half_extents,
        } => (
            MeshKey::Cuboid {
                half_extent_bits: half_extents.to_array().map(f32::to_bits),
            },
            Transform::from_translation(*center).with_rotation(*rotation),
        ),
    }
}

/// Creates one Bevy mesh for an exact mesh cache key.
fn mesh_from_key(key: MeshKey) -> Mesh {
    match key {
        MeshKey::Capsule {
            radius_bits,
            length_bits,
        } => Capsule3d::new(f32::from_bits(radius_bits), f32::from_bits(length_bits)).into(),
        MeshKey::Sphere { radius_bits } => Sphere::new(f32::from_bits(radius_bits)).into(),
        MeshKey::Cuboid { half_extent_bits } => {
            Cuboid::from_size(Vec3::from_array(half_extent_bits.map(f32::from_bits)) * 2.0).into()
        }
    }
}

/// Handles manual activation (Space) and the global freeze toggle (F).
fn handle_keyboard(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut control: ResMut<RunControl>,
) {
    if keys.just_pressed(KeyCode::Space) {
        control.manual_trigger = Some(time.elapsed());
    }
    if keys.just_pressed(KeyCode::KeyF) {
        control.freeze_all = !control.freeze_all;
    }
}

/// Refreshes the overlay's population counts twice per simulated second.
fn update_overlay(
    time: Res<Time>,
    scene: Res<RunScene>,
    control: Res<RunControl>,
    characters: Query<&RagdollMode, With<ScenarioCharacter>>,
    bodies: Query<(), With<BodyShape>>,
    mut text: Single<&mut Text, With<StressOverlay>>,
    mut last_update: Local<f64>,
) {
    let elapsed = f64::from(time.elapsed_secs());
    if elapsed - *last_update < 0.5 {
        return;
    }
    *last_update = elapsed;
    let active = characters
        .iter()
        .filter(|mode| **mode == RagdollMode::Dynamic)
        .count();
    let body_count = bodies.iter().count();
    let trigger_state = if control.triggered_at.is_some() {
        "triggered"
    } else {
        "standing"
    };
    let scenario = scene.config.scenario;
    text.0 = format!(
        "{scenario} · {active} dynamic · {body_count} bodies · {trigger_state} · {CONTROLS}"
    );
}

/// Requests the final window screenshot after the measurement window ends.
fn request_screenshot(
    mut commands: Commands,
    scene: Res<RunScene>,
    time: Res<Time>,
    mut control: ResMut<RunControl>,
) {
    let Some(path) = scene.screenshot.as_ref() else {
        return;
    };
    if !control.finished || control.screenshot_requested_at.is_some() {
        return;
    }
    if let Some(parent) = path.parent()
        && !parent.as_os_str().is_empty()
        && let Err(screenshot_directory_error) = std::fs::create_dir_all(parent)
    {
        error!(%screenshot_directory_error, "could not create screenshot directory");
        control.report_failed = true;
        return;
    }
    // Remove a prior output so file existence confirms this request completed.
    if path.exists()
        && let Err(screenshot_remove_error) = std::fs::remove_file(path)
    {
        error!(%screenshot_remove_error, "could not replace the existing screenshot");
        control.report_failed = true;
        return;
    }
    commands
        .spawn(Screenshot::primary_window())
        .observe(save_to_disk(path.clone()));
    control.screenshot_requested_at = Some(time.elapsed());
    control.screenshot_started_at = Some(Instant::now());
}

#[cfg(test)]
mod tests {
    //! Checks CLI parsing, report schema, comparison, sweep, and shooting logic.

    use super::*;

    /// Parses arguments after the program name.
    fn parse(arguments: &[&str]) -> Result<StressCli, clap::Error> {
        StressCli::try_parse_from(
            std::iter::once("ragdoll_stress").chain(arguments.iter().copied()),
        )
    }

    /// Returns a valid profile index for test records.
    fn body_index(index: usize) -> BodyIndex {
        BodyIndex::try_from(index).expect("small profile indexes are valid")
    }

    /// Documented defaults construct a 16 by 16 headless grid configuration.
    #[test]
    fn defaults_preserve_the_phase_six_configuration() {
        let config = parse(&["--headless"])
            .expect("documented defaults parse")
            .run_config()
            .expect("default run configuration is valid");

        assert_eq!(config.grid, [16, 16]);
        assert_eq!(config.count, 256);
        assert_eq!(config.fixed_hz, 60);
        assert_eq!(config.seed, 42);
        assert!(config.headless);
    }

    /// Nonpositive counts, dimensions, durations, rates, and spacing fail at ingress.
    #[test]
    fn invalid_positive_arguments_are_rejected_independently() {
        for arguments in [
            ["--count", "0"],
            ["--grid", "0x16"],
            ["--duration", "0"],
            ["--fixed-hz", "0"],
            ["--spacing", "NaN"],
            ["--substeps", "0"],
        ] {
            assert!(parse(&arguments).is_err(), "{arguments:?} must fail");
        }
    }

    /// Zero is a valid dynamic budget and headless is a presence-only marker.
    #[test]
    fn budget_zero_and_presence_markers_parse() {
        let cli = parse(&["--budget", "0", "--headless"])
            .expect("zero budget and headless marker are valid");

        assert_eq!(cli.budget.map(Budget::get), Some(0));
        assert!(cli.headless.is_some());
    }

    /// Child arguments keep validated values and always select a headless report run.
    #[test]
    fn child_arguments_keep_values_and_omit_absent_options() {
        let cli = parse(&["--headless"]).expect("headless parent options parse");
        let arguments = cli.child_arguments(
            Backend::Rapier3d,
            Scenario::Shooting,
            cli.grid,
            CharacterCount::try_new(64).expect("64 characters is a valid population"),
            Path::new("sweep.json"),
        );
        let has_argument = |value: &str| arguments.iter().any(|argument| argument == value);
        let has_pair = |flag: &str, value: &str| {
            arguments
                .windows(2)
                .any(|pair| pair[0] == flag && pair[1] == value)
        };

        assert!(has_pair("--scenario", "shooting"));
        assert!(has_pair("--count", "64"));
        assert!(has_pair("--report", "sweep.json"));
        assert!(has_argument("--headless"));
        assert!(!has_argument("--deterministic"));
        assert!(!has_argument("--substeps"));
        assert!(!has_argument("--budget"));

        let child = parse(&arguments.iter().map(String::as_str).collect::<Vec<_>>())
            .expect("child arguments parse");
        assert_eq!(child.scenario, Scenario::Shooting);
        assert_eq!(child.count.get(), 64);
    }

    /// The default sweep includes 64-character shooting.
    #[test]
    fn default_sweep_includes_shooting_64() {
        let rows = default_rows();
        let shooting = rows
            .iter()
            .find(|row| row.scenario == Scenario::Shooting)
            .expect("the default sweep includes shooting");

        assert_eq!(rows.len(), 9);
        assert_eq!(shooting.count, Some(64));
        assert_eq!(shooting.grid, None);
    }

    /// Phase-deferred options are rejected before an app starts.
    #[test]
    fn deferred_options_are_rejected() {
        for arguments in [
            ["--backend", "avian3d"],
            ["--scenario", "balance"],
            ["--scenario", "mixed"],
            ["--mode", "balance"],
            ["--screenshot", "out.png"],
        ] {
            let mut arguments = arguments.to_vec();
            arguments.push("--headless");
            let cli = parse(&arguments).expect("arguments parse");
            assert!(validate_run(&cli).is_err(), "{arguments:?} must fail");
        }
    }

    /// The shooting scenario validates.
    #[test]
    fn shooting_scenario_is_available() {
        let cli = parse(&["--scenario", "shooting", "--headless"])
            .expect("the shooting stress arguments are valid");

        let config = validate_run(&cli).expect("shooting stress runs are enabled");

        assert_eq!(config.scenario, Scenario::Shooting);
    }

    /// Shooting targets are grouped by character bits, then profile index.
    #[test]
    fn shooting_targets_are_sorted_by_character_and_profile_index() {
        let first_character = Entity::from_bits(1);
        let second_character = Entity::from_bits(2);
        let [first_body, second_body, third_body, fourth_body] =
            [10, 11, 20, 21].map(Entity::from_bits);
        let record = |character, index, body| ShootingBodyRecord {
            character,
            index: body_index(index),
            body,
        };
        let bodies = vec![
            record(second_character, 1, fourth_body),
            record(first_character, 1, second_body),
            record(second_character, 0, third_body),
            record(first_character, 0, first_body),
        ];

        let targets = group_shooting_targets(vec![second_character, first_character], bodies, 2)
            .expect("complete character body trees are accepted");

        assert_eq!(
            targets,
            [vec![first_body, second_body], vec![third_body, fourth_body]]
        );
    }

    /// Incomplete or empty snapshots do not produce a cache.
    #[test]
    fn shooting_target_cache_waits_for_a_complete_profile() {
        let character = Entity::from_bits(1);
        let incomplete = vec![ShootingBodyRecord {
            character,
            index: body_index(1),
            body: Entity::from_bits(10),
        }];

        assert!(group_shooting_targets(vec![character], incomplete, 2).is_none());
        assert!(group_shooting_targets(vec![character], Vec::new(), 0).is_none());
        assert!(group_shooting_targets(Vec::new(), Vec::new(), 1).is_none());
    }

    /// Duplicate profile indexes for one character are rejected.
    #[test]
    fn shooting_target_cache_rejects_duplicate_profile_indexes() {
        let character = Entity::from_bits(1);
        let bodies = [10, 11]
            .map(|bits| ShootingBodyRecord {
                character,
                index: body_index(0),
                body: Entity::from_bits(bits),
            })
            .to_vec();

        assert!(group_shooting_targets(vec![character], bodies, 2).is_none());
    }

    /// The shooting clock keeps fractional time and catches up after long steps.
    #[test]
    fn shooting_clock_carries_fixed_step_fractions_and_catches_up() {
        let mut elapsed = Duration::ZERO;

        assert_eq!(take_due_shots(&mut elapsed, Duration::from_millis(49)), 0);
        assert_eq!(elapsed, Duration::from_millis(49));
        assert_eq!(take_due_shots(&mut elapsed, Duration::from_millis(1)), 1);
        assert_eq!(elapsed, Duration::ZERO);
        assert_eq!(take_due_shots(&mut elapsed, Duration::from_millis(120)), 2);
        assert_eq!(elapsed, Duration::from_millis(20));
    }

    /// Equal seeds produce equal bounded indexes, and empty ranges produce none.
    #[test]
    fn shooting_random_indexes_are_repeatable_and_bounded() {
        let mut first = SplitMix64::new(42);
        let mut second = SplitMix64::new(42);

        assert_eq!(first.index(0), None);
        assert_eq!(second.index(0), None);
        assert_eq!(SplitMix64::new(42).index(1), Some(0));
        for _ in 0..256 {
            let first_index = first.index(17);
            assert_eq!(first_index, second.index(17));
            assert!(first_index.is_some_and(|index| index < 17));
        }
    }

    /// Shooting characters stand on a ring whose radius scales with spacing.
    #[test]
    fn shooting_characters_are_placed_on_a_spacing_scaled_ring() {
        let config = parse(&[
            "--scenario",
            "shooting",
            "--count",
            "4",
            "--spacing",
            "2",
            "--headless",
        ])
        .expect("the shooting stress arguments are valid")
        .run_config()
        .expect("the shooting configuration is valid");
        let radius = config.spacing_m * config.count as f32 / std::f32::consts::TAU;

        for index in 0..config.count {
            let position = character_position(&config, index, 0, 0);
            assert!((position.length() - radius).abs() < 1.0e-5);
        }
    }

    /// The embedded TGF human profile parses and validates.
    #[test]
    fn embedded_profile_loads() {
        let profile = load_profile().expect("the embedded profile is valid");

        assert!(!profile.bodies().is_empty());
    }

    /// Nearest-rank p50 selects the fiftieth value from one hundred ordered samples.
    #[test]
    fn nearest_rank_percentile_uses_the_documented_rank() {
        let samples = (1..=100).map(f64::from).collect::<Vec<_>>();

        assert_eq!(
            nearest_rank_percentile(&samples, Percentile::P50),
            Some(50.0)
        );
    }

    /// Empty timing history has no percentile value.
    #[test]
    fn nearest_rank_percentile_rejects_empty_samples() {
        assert_eq!(nearest_rank_percentile(&[], Percentile::P95), None);
    }

    /// The allowed comparison edge passes while a 10.1 percent regression fails.
    #[test]
    fn comparison_threshold_accepts_ten_percent_only() {
        assert_eq!(compare_p95(110.0, 100.0), Comparison::Pass);
        assert_eq!(compare_p95(110.1, 100.0), Comparison::Regression);
    }

    /// A complete schema-one report preserves its JSON values through a round trip.
    #[test]
    fn stress_report_round_trips_json() {
        let json = r#"{
            "schema": 1,
            "git": "abc1234",
            "machine": {"cpu": "test cpu", "cores": 8, "os": "linux"},
            "config": {
                "backend": "rapier3d",
                "scenario": "grid",
                "grid": [16, 16],
                "count": 256,
                "spacing_m": 1.5,
                "creature": "human",
                "mode": "limp",
                "trigger_at_s": 2.0,
                "duration_s": 10.0,
                "warmup_s": 1.0,
                "fixed_hz": 60,
                "substeps": 1,
                "seed": 42,
                "headless": true,
                "deterministic": false
            },
            "metrics": {
                "frame_ms": {"p50": 1.0, "p95": 2.0, "p99": 3.0, "max": 4.0},
                "step_ms": {"p50": 0.5, "p95": 0.75, "max": 1.0},
                "core_ms": {"p50": 0.1, "p95": 0.2},
                "trigger_spike_ms": 3.0,
                "characters": 256,
                "bodies": 4096,
                "joints": 3840,
                "dynamic": 4096,
                "sleeping_end": 0,
                "frozen_end": 0,
                "unstable_bodies": 0
            }
        }"#;
        let report = serde_json::from_str::<StressReport>(json).expect("valid report JSON");
        let round_trip = serde_json::to_value(report).expect("serializable report");
        let original = serde_json::from_str::<serde_json::Value>(json).expect("valid JSON value");

        assert_eq!(round_trip, original);
    }

    /// Schema one rejects unsupported numeric versions before deserializing metrics.
    #[test]
    fn stress_report_rejects_unknown_schema_versions() {
        assert!(serde_json::from_str::<StressReport>(r#"{"schema":2}"#).is_err());
    }

    /// Creates a compact report with caller-selected comparison boundary values.
    fn comparison_report(
        scenario: Scenario,
        frame_p95: f64,
        step_p95: f64,
        unstable_bodies: usize,
    ) -> StressReport {
        let config = parse(&["--headless", "--scenario", &scenario.to_string()])
            .expect("comparison arguments parse")
            .run_config()
            .expect("comparison configuration is valid");
        StressReport {
            schema: ReportSchema::V1,
            git: "abc1234".to_owned(),
            machine: MachineInfo {
                cpu: "test cpu".to_owned(),
                cores: 8,
                os: "linux".to_owned(),
                gpu: None,
            },
            config,
            metrics: StressMetrics {
                frame_ms: FrameStatistics {
                    p95: frame_p95,
                    ..default()
                },
                step_ms: StepStatistics {
                    p95: step_p95,
                    ..default()
                },
                unstable_bodies,
                ..default()
            },
        }
    }

    /// Matching reports accept the exact ten percent frame and step boundaries.
    #[test]
    fn matching_reports_accept_exactly_ten_percent_regression() {
        let baseline = comparison_report(Scenario::Grid, 100.0, 40.0, 0);
        let candidate = comparison_report(Scenario::Grid, 110.0, 44.0, 0);

        assert!(compare_reports(&[candidate], &[baseline]).is_empty());
    }

    /// Comparison reports timing regressions and unstable bodies in metric order.
    #[test]
    fn matching_reports_collect_each_failed_metric() {
        let baseline = comparison_report(Scenario::Grid, 100.0, 40.0, 0);
        let candidate = comparison_report(Scenario::Grid, 110.1, 44.04, 1);
        let issues = compare_reports(&[candidate], &[baseline]);

        assert_eq!(
            issues
                .iter()
                .map(|issue| issue.metric.as_str())
                .collect::<Vec<_>>(),
            ["frame_ms.p95", "step_ms.p95", "unstable_bodies"]
        );
    }

    /// Comparison rejects invalid timing values and reports a missing baseline.
    #[test]
    fn comparison_rejects_invalid_timings_and_missing_configurations() {
        let baseline = comparison_report(Scenario::Grid, 100.0, 40.0, 0);
        let invalid = comparison_report(Scenario::Grid, f64::NAN, -1.0, 0);
        let invalid_issues = compare_reports(&[invalid], std::slice::from_ref(&baseline));
        assert_eq!(invalid_issues.len(), 2);
        assert!(
            invalid_issues
                .iter()
                .all(|issue| issue.detail == "timings must be finite and nonnegative")
        );

        let missing = comparison_report(Scenario::Pile, 1.0, 1.0, 0);
        let missing_issues = compare_reports(&[missing], &[baseline]);
        assert_eq!(missing_issues.len(), 1);
        assert_eq!(missing_issues[0].metric, "baseline");
    }
}
