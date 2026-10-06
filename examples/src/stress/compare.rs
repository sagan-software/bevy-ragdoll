//! Baseline matching and threshold evaluation for stress reports.

use std::fmt::{Display, Formatter};

use super::report::StressReport;

/// Whether one metric remains within or exceeds the allowed regression limit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Comparison {
    /// Candidate p95 is no more than 10 percent slower than the baseline.
    Pass,
    /// Candidate p95 is more than 10 percent slower than the baseline.
    Regression,
}

/// One exact configuration or metric failure from a baseline comparison.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ComparisonIssue {
    /// Scenario and backend key associated with the mismatch.
    pub run: String,
    /// Metric or lookup operation that failed its comparison.
    pub metric: String,
    /// Human-readable observed values or missing-baseline explanation.
    pub detail: String,
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
/// An exact 10 percent increase passes. A candidate above the baseline by more
/// than 10 percent fails.
pub fn compare_p95(candidate_ms: f64, baseline_ms: f64) -> Comparison {
    if candidate_ms <= baseline_ms * 1.10 {
        Comparison::Pass
    } else {
        Comparison::Regression
    }
}

/// Compares p95 frame/step time and simulation health for matching run configurations.
pub fn compare_reports(
    candidates: &[StressReport],
    baselines: &[StressReport],
) -> Vec<ComparisonIssue> {
    let mut issues = Vec::new();
    for candidate in candidates {
        let backend = candidate.config.backend;
        let scenario = candidate.config.scenario;
        let run = format!("{backend} {scenario}");
        let Some(baseline) = baselines
            .iter()
            .find(|baseline| baseline.config == candidate.config)
        else {
            issues.push(ComparisonIssue {
                run,
                metric: "baseline".to_owned(),
                detail: "no report has the same configuration".to_owned(),
            });
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
                issues.push(ComparisonIssue {
                    run: run.clone(),
                    metric: metric.to_owned(),
                    detail: "timings must be finite and nonnegative".to_owned(),
                });
            } else if compare_p95(candidate_ms, baseline_ms) == Comparison::Regression {
                issues.push(ComparisonIssue {
                    run: run.clone(),
                    metric: metric.to_owned(),
                    detail: format!(
                        "{candidate_ms:.3} ms exceeds the 10% limit over {baseline_ms:.3} ms"
                    ),
                });
            }
        }
        if candidate.metrics.unstable_bodies > 0 {
            let count = candidate.metrics.unstable_bodies;
            issues.push(ComparisonIssue {
                run,
                metric: "unstable_bodies".to_owned(),
                detail: format!("{count} bodies exceeded simulation health limits"),
            });
        }
    }
    issues
}
