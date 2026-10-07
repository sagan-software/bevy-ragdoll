//! Typed command-line values for native stress runs.

use std::num::NonZeroU32;
use std::path::Path;
use std::path::PathBuf;
use std::str::FromStr;
use std::time::Duration;

use clap::Parser;

use super::config::{Backend, Creature, DriveMode, RunConfig, Scenario};

/// Native command-line options for one stress run or a sweep.
#[derive(Clone, Debug, Parser)]
#[command(
    name = "ragdoll_stress",
    about = "Measure bevy-ragdoll stress scenarios"
)]
pub(super) struct StressCli {
    /// Select one compiled physics backend.
    #[arg(long, default_value = "rapier3d")]
    pub(super) backend: Backend,
    /// Select the simulation population and activation scenario.
    #[arg(long, default_value = "grid")]
    pub(super) scenario: Scenario,
    /// Character count for pile and shooting scenarios.
    #[arg(long, default_value = "64")]
    pub(super) count: CharacterCount,
    /// Grid width and depth written as `columns x rows`.
    #[arg(long, default_value = "16x16")]
    pub(super) grid: GridSize,
    /// Distance between grid characters in metres.
    #[arg(long, default_value = "1.5")]
    pub(super) spacing: Meters,
    /// Select the character rig profile.
    #[arg(long, default_value = "human")]
    pub(super) creature: Creature,
    /// Select the drive policy after activation.
    #[arg(long, default_value = "limp")]
    pub(super) mode: DriveMode,
    /// Seconds before the first activation trigger.
    #[arg(long, default_value = "2")]
    pub(super) trigger_at: NonnegativeSeconds,
    /// Seconds measured after activation.
    #[arg(long, default_value = "10")]
    pub(super) duration: PositiveSeconds,
    /// Initial seconds excluded from measurements.
    #[arg(long, default_value = "1")]
    pub(super) warmup: NonnegativeSeconds,
    /// Fixed simulation frequency in hertz.
    #[arg(long, default_value = "60")]
    pub(super) fixed_hz: PositiveHertz,
    /// Rapier substeps per fixed interval; the backend default is used when absent.
    #[arg(long)]
    pub(super) substeps: Option<PositiveCount>,
    /// Maximum dynamic-character budget; absence means unlimited.
    #[arg(long)]
    pub(super) budget: Option<Budget>,
    /// Seed used for all deterministic stress inputs.
    #[arg(long, default_value = "42")]
    pub(super) seed: Seed,
    /// Enable the reserved deterministic backend path.
    #[arg(long, num_args = 0..=1, default_missing_value = "enabled")]
    pub(super) deterministic: Option<DeterministicMode>,
    /// Run without a window or renderer.
    #[arg(long, num_args = 0..=1, default_missing_value = "enabled")]
    pub(super) headless: Option<HeadlessMode>,
    /// Write one run report to this JSON path.
    #[arg(long)]
    pub(super) report: Option<PathBuf>,
    /// Save a visible-run screenshot to this PNG path.
    #[arg(long)]
    pub(super) screenshot: Option<PathBuf>,
    /// Run the named default sweep or read a RON sweep file.
    #[arg(long)]
    pub(super) sweep: Option<SweepSelection>,
    /// Compare this run or sweep with a prior JSON report.
    #[arg(long)]
    pub(super) compare: Option<PathBuf>,
}

/// Typed values passed from a parent stress run to one child process.
#[derive(Clone, Copy, Debug)]
pub(super) struct ChildRun<'a> {
    /// Physics backend selected for the child.
    pub(super) backend: Backend,
    /// Scenario selected for the child.
    pub(super) scenario: Scenario,
    /// Grid dimensions retained for grid scenarios.
    pub(super) grid: GridSize,
    /// Positive population count selected for the child.
    pub(super) count: CharacterCount,
    /// Optional JSON report destination for the child.
    pub(super) report: Option<&'a Path>,
    /// Optional screenshot destination for a visible child.
    pub(super) screenshot: Option<&'a Path>,
    /// Presence marker selecting a headless child process.
    pub(super) headless: Option<HeadlessMode>,
    /// Presence marker selecting the reserved deterministic mode.
    pub(super) deterministic: Option<DeterministicMode>,
}

impl Default for StressCli {
    /// Creates browser-safe stress options matching the documented defaults.
    fn default() -> Self {
        Self {
            backend: Backend::Rapier3d,
            scenario: Scenario::Grid,
            count: CharacterCount(64),
            grid: GridSize {
                columns: NonZeroU32::new(16).expect("sixteen is positive"),
                rows: NonZeroU32::new(16).expect("sixteen is positive"),
            },
            spacing: Meters(1.5),
            creature: Creature::Human,
            mode: DriveMode::Limp,
            trigger_at: NonnegativeSeconds(Duration::from_secs(2)),
            duration: PositiveSeconds(Duration::from_secs(10)),
            warmup: NonnegativeSeconds(Duration::from_secs(1)),
            fixed_hz: PositiveHertz(NonZeroU32::new(60).expect("sixty is positive")),
            substeps: None,
            budget: None,
            seed: Seed(42),
            deterministic: None,
            headless: None,
            report: None,
            screenshot: None,
            sweep: None,
            compare: None,
        }
    }
}

impl StressCli {
    /// Converts parsed boundary values into the serializable run description.
    pub(super) fn run_config(&self) -> Result<RunConfig, &'static str> {
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

    /// Builds child-process arguments from typed values for keyboard restarts.
    #[cfg(feature = "visual")]
    pub(super) fn restart_arguments(&self, backend: Backend, scenario: Scenario) -> Vec<String> {
        self.child_arguments(ChildRun {
            backend,
            scenario,
            grid: self.grid,
            count: self.count,
            report: self.report.as_deref(),
            screenshot: self.screenshot.as_deref(),
            headless: self.headless,
            deterministic: self.deterministic,
        })
    }

    /// Builds child arguments from validated values without reparsing raw process arguments.
    pub(super) fn child_arguments(&self, child: ChildRun<'_>) -> Vec<String> {
        let ChildRun {
            backend,
            scenario,
            grid,
            count,
            report,
            screenshot,
            headless,
            deterministic,
        } = child;
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
        if headless.is_some() {
            arguments.push("--headless".to_owned());
        }
        if deterministic.is_some() {
            arguments.push("--deterministic".to_owned());
        }
        if let Some(path) = report {
            arguments.extend(["--report".to_owned(), path.display().to_string()]);
        }
        if let Some(path) = screenshot {
            arguments.extend(["--screenshot".to_owned(), path.display().to_string()]);
        }
        arguments
    }
}

/// Positive character count parsed before allocating a population.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct CharacterCount(
    /// Nonzero number of characters allocated by this run.
    usize,
);

impl CharacterCount {
    /// Returns the validated nonzero count.
    pub(super) const fn get(self) -> usize {
        self.0
    }

    /// Creates a positive character count from a RON sweep row.
    pub(super) fn try_new(value: usize) -> Result<Self, &'static str> {
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
        let count = input
            .parse::<usize>()
            .map_err(|_| "count must be a positive integer")?;
        if count == 0 {
            return Err("count must be a positive integer");
        }
        Ok(Self(count))
    }
}

impl std::fmt::Display for CharacterCount {
    /// Writes the canonical positive decimal character count.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let count = self.0;
        write!(formatter, "{count}")
    }
}

/// Positive grid dimensions stored as nonzero unsigned integers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct GridSize {
    /// Number of columns in the horizontal grid.
    columns: NonZeroU32,
    /// Number of rows in the horizontal grid.
    rows: NonZeroU32,
}

impl GridSize {
    /// Returns checked grid dimensions in column-then-row order.
    pub(super) const fn as_array(self) -> [NonZeroU32; 2] {
        [self.columns, self.rows]
    }

    /// Creates positive grid dimensions from a RON sweep row.
    pub(super) fn try_new(columns: u32, rows: u32) -> Result<Self, &'static str> {
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
        let columns = columns
            .parse::<u32>()
            .ok()
            .and_then(NonZeroU32::new)
            .ok_or("grid dimensions must be positive 32-bit integers")?;
        let rows = rows
            .parse::<u32>()
            .ok()
            .and_then(NonZeroU32::new)
            .ok_or("grid dimensions must be positive 32-bit integers")?;
        Ok(Self { columns, rows })
    }
}

impl std::fmt::Display for GridSize {
    /// Writes the canonical column-then-row spelling.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let columns = self.columns;
        let rows = self.rows;
        write!(formatter, "{columns}x{rows}")
    }
}

/// Positive finite grid distance measured in metres.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Meters(
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
        let value = input
            .parse::<f64>()
            .map_err(|_| "spacing must be positive and finite")?;
        if !value.is_finite() || value <= 0.0 || value > f64::from(f32::MAX) {
            return Err("spacing must be positive and finite");
        }
        let meters = value as f32;
        if meters.is_finite() && meters > 0.0 {
            Ok(Self(meters))
        } else {
            Err("spacing must be positive and finite")
        }
    }
}

impl std::fmt::Display for Meters {
    /// Writes the canonical decimal distance in metres.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let meters = self.0;
        write!(formatter, "{meters}")
    }
}

/// Positive duration parsed from seconds without retaining a raw float.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct PositiveSeconds(Duration);

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

impl std::fmt::Display for PositiveSeconds {
    /// Writes the duration's canonical seconds value.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let seconds = self.0.as_secs_f64();
        write!(formatter, "{seconds}")
    }
}

/// Nonnegative duration used for trigger and warmup offsets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct NonnegativeSeconds(Duration);

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

impl std::fmt::Display for NonnegativeSeconds {
    /// Writes the duration's canonical seconds value.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let seconds = self.0.as_secs_f64();
        write!(formatter, "{seconds}")
    }
}

/// Parses finite seconds shared by the two validated duration types.
fn parse_seconds(input: &str) -> Result<f64, &'static str> {
    let seconds = input.parse::<f64>().map_err(|_| "seconds must be finite")?;
    if seconds.is_finite() {
        Ok(seconds)
    } else {
        Err("seconds must be finite")
    }
}

/// Positive fixed simulation frequency in hertz.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct PositiveHertz(
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
        let hz = input
            .parse::<u32>()
            .ok()
            .and_then(NonZeroU32::new)
            .ok_or("fixed-hz must be a positive 32-bit integer")?;
        Ok(Self(hz))
    }
}

impl std::fmt::Display for PositiveHertz {
    /// Writes the integer frequency in hertz.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let hertz = self.0;
        write!(formatter, "{hertz}")
    }
}

/// Positive count used for backend substeps.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct PositiveCount(
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
        let count = input
            .parse::<u32>()
            .ok()
            .and_then(NonZeroU32::new)
            .ok_or("substeps must be a positive 32-bit integer")?;
        Ok(Self(count))
    }
}

impl std::fmt::Display for PositiveCount {
    /// Writes the integer substep count.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let count = self.0;
        write!(formatter, "{count}")
    }
}

/// Optional dynamic-character budget where zero is a valid limit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Budget(
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

impl std::fmt::Display for Budget {
    /// Writes the integer dynamic-character limit.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let budget = self.0;
        write!(formatter, "{budget}")
    }
}

/// Seed value kept distinct from counts and budgets.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Seed(
    /// Unsigned ChaCha8 pseudorandom seed.
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

impl std::fmt::Display for Seed {
    /// Writes the canonical unsigned decimal seed.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let seed = self.0;
        write!(formatter, "{seed}")
    }
}

/// Presence marker for an enabled deterministic mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum DeterministicMode {
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
pub(super) enum HeadlessMode {
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
pub(super) enum SweepSelection {
    /// Use the Phase 6 default matrix for compiled backends and scenarios.
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
        if input == "default" {
            Ok(Self::Default)
        } else if input.is_empty() {
            Err("sweep path cannot be empty")
        } else {
            Ok(Self::File(PathBuf::from(input)))
        }
    }
}

#[cfg(test)]
mod tests {
    //! Checks CLI defaults and rejected numeric or closed-vocabulary inputs.

    use clap::Parser;

    use super::{
        Backend, CharacterCount, ChildRun, DeterministicMode, HeadlessMode, Scenario, StressCli,
    };
    use std::path::Path;

    /// Documented defaults construct a 16 by 16 headless grid configuration.
    #[test]
    fn defaults_preserve_the_phase_six_configuration() {
        let cli = StressCli::try_parse_from(["ragdoll_stress", "--headless"])
            .expect("documented defaults parse");
        let config = cli
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
            ["ragdoll_stress", "--count", "0"],
            ["ragdoll_stress", "--grid", "0x16"],
            ["ragdoll_stress", "--duration", "0"],
            ["ragdoll_stress", "--fixed-hz", "0"],
            ["ragdoll_stress", "--spacing", "NaN"],
            ["ragdoll_stress", "--substeps", "0"],
        ] {
            assert!(StressCli::try_parse_from(arguments).is_err());
        }
    }

    /// Zero is a valid dynamic budget and headless is a presence-only marker.
    #[test]
    fn budget_zero_and_presence_markers_parse() {
        let cli = StressCli::try_parse_from(["ragdoll_stress", "--budget", "0", "--headless"])
            .expect("zero budget and headless marker are valid");

        assert_eq!(cli.budget.map(super::Budget::get), Some(0));
        assert!(cli.headless.is_some());
    }

    /// Child arguments preserve validated values and optional mode markers.
    #[test]
    fn child_arguments_keep_values_and_omit_absent_options() {
        let cli = StressCli::try_parse_from(["ragdoll_stress", "--headless"])
            .expect("headless parent options parse");
        let arguments = cli.child_arguments(ChildRun {
            backend: Backend::Rapier3d,
            scenario: Scenario::Shooting,
            grid: cli.grid,
            count: CharacterCount::try_new(64).expect("64 characters is a valid population"),
            report: Some(Path::new("sweep.json")),
            screenshot: Some(Path::new("screen.png")),
            headless: Some(HeadlessMode::Enabled),
            deterministic: Some(DeterministicMode::Enabled),
        });
        let has_argument = |value: &str| arguments.iter().any(|argument| argument == value);
        let has_pair = |flag: &str, value: &str| {
            arguments
                .windows(2)
                .any(|pair| pair[0] == flag && pair[1] == value)
        };

        assert!(has_pair("--scenario", "shooting"));
        assert!(has_pair("--count", "64"));
        assert!(has_pair("--report", "sweep.json"));
        assert!(has_pair("--screenshot", "screen.png"));
        assert!(has_argument("--headless"));
        assert!(has_argument("--deterministic"));

        let without_options = cli.child_arguments(ChildRun {
            backend: Backend::Rapier3d,
            scenario: Scenario::Grid,
            grid: cli.grid,
            count: cli.count,
            report: None,
            screenshot: None,
            headless: None,
            deterministic: None,
        });
        let has_optional_argument =
            |flag: &str| without_options.iter().any(|argument| argument == flag);
        assert!(!has_optional_argument("--report"));
        assert!(!has_optional_argument("--screenshot"));
        assert!(!has_optional_argument("--headless"));
        assert!(!has_optional_argument("--deterministic"));
    }
}
