//! Stress-run configuration, reports, measurements, and process orchestration.

mod cli;
mod compare;
mod config;
mod metrics;
mod report;
mod runner;
mod sweep;

pub use self::compare::{Comparison, ComparisonIssue, compare_p95, compare_reports};
pub use self::config::{Backend, Creature, DriveMode, RunConfig, Scenario};
pub use self::metrics::{
    CoreStatistics, FrameStatistics, Percentile, StepStatistics, StressMetrics,
    nearest_rank_percentile,
};
pub use self::report::{MachineInfo, ReportSchema, StressReport};
pub use self::runner::run_stress;
