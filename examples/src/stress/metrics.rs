//! Timing summaries and simulation health measurements for one stress run.

use serde::{Deserialize, Serialize};

/// Percentile represented by one of the report's documented timing summaries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Percentile {
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

/// Frame-time percentiles and maximum, all measured in milliseconds.
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct FrameStatistics {
    /// Nearest-rank p50 frame duration in milliseconds.
    pub p50: f64,
    /// Nearest-rank p95 frame duration in milliseconds.
    pub p95: f64,
    /// Nearest-rank p99 frame duration in milliseconds.
    pub p99: f64,
    /// Maximum measured frame duration in milliseconds.
    pub max: f64,
}

/// Physics-step percentiles and maximum, all measured in milliseconds.
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct StepStatistics {
    /// Nearest-rank p50 fixed-step duration in milliseconds.
    pub p50: f64,
    /// Nearest-rank p95 fixed-step duration in milliseconds.
    pub p95: f64,
    /// Maximum measured fixed-step duration in milliseconds.
    pub max: f64,
}

/// Rag-doll core schedule percentiles, all measured in milliseconds.
#[derive(Clone, Copy, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CoreStatistics {
    /// Nearest-rank p50 core schedule duration in milliseconds.
    pub p50: f64,
    /// Nearest-rank p95 core schedule duration in milliseconds.
    pub p95: f64,
}

/// Timing, count, memory, and stability measurements for one run.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct StressMetrics {
    /// Frame timing distribution in milliseconds.
    pub frame_ms: FrameStatistics,
    /// Physics fixed-step timing distribution in milliseconds.
    pub step_ms: StepStatistics,
    /// Ragdoll core schedule timing distribution in milliseconds.
    pub core_ms: CoreStatistics,
    /// Worst frame time within 0.5 seconds after activation, in milliseconds.
    pub trigger_spike_ms: f64,
    /// Number of spawned characters at the end of the run.
    pub characters: usize,
    /// Number of ragdoll physics bodies at the end of the run.
    pub bodies: usize,
    /// Number of ragdoll joints at the end of the run.
    pub joints: usize,
    /// Number of dynamic physics bodies at the end of the run.
    pub dynamic: usize,
    /// Number of sleeping bodies at the end of the run.
    pub sleeping_end: usize,
    /// Number of frozen bodies at the end of the run.
    pub frozen_end: usize,
    /// Linux peak resident set size in megabytes, omitted on other platforms.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub peak_rss_mb: Option<f64>,
    /// Number of bodies faster than 50 metres per second or with a non-finite pose.
    pub unstable_bodies: usize,
}

/// Returns the nearest-rank value for the requested percentile.
///
/// The function copies and sorts samples, so it takes O(n log n) time and O(n)
/// auxiliary space. Empty input returns `None`; samples must be finite,
/// nonnegative durations in milliseconds. The nearest-rank index is
/// `ceil(probability × sample_count) - 1`, so p50 of 1 through 100 is 50.
pub fn nearest_rank_percentile(samples: &[f64], percentile: Percentile) -> Option<f64> {
    if samples.is_empty() {
        return None;
    }
    let mut ordered = samples.to_vec();
    ordered.sort_by(f64::total_cmp);
    let rank = (percentile.probability() * ordered.len() as f64).ceil() as usize;
    ordered.get(rank.saturating_sub(1)).copied()
}
