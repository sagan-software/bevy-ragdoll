//! Schema-versioned machine, run configuration, and stress metrics reports.

use serde::{Deserialize, Serialize};

use super::config::RunConfig;
use super::metrics::StressMetrics;

/// Closed report schema vocabulary serialized as the JSON number `1`.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize)]
#[serde(try_from = "u8", into = "u8")]
pub enum ReportSchema {
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
pub struct MachineInfo {
    /// CPU model name, or the architecture name when the model is unavailable.
    pub cpu: String,
    /// Number of logical CPUs available to the process.
    pub cores: usize,
    /// Operating system name reported by Rust's platform constants.
    pub os: String,
    /// Rendering adapter name when a visible run has a GPU adapter.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub gpu: Option<String>,
}

/// One complete schema-versioned stress run report.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct StressReport {
    /// Closed schema version for this report's serialized shape.
    pub schema: ReportSchema,
    /// Short source revision that built the measured process.
    pub git: String,
    /// Machine description captured by the measured process.
    pub machine: MachineInfo,
    /// Validated scenario and tuning values used by the process.
    pub config: RunConfig,
    /// Timing, population, memory, and health measurements.
    pub metrics: StressMetrics,
}

impl StressReport {
    /// Creates the current schema version around one measured run.
    pub fn new(
        git: String,
        machine: MachineInfo,
        config: RunConfig,
        metrics: StressMetrics,
    ) -> Self {
        Self {
            schema: ReportSchema::V1,
            git,
            machine,
            config,
            metrics,
        }
    }
}
