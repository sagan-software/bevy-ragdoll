//! Checks the public report and comparison boundary used by stress runs.

use bevy_ragdoll_examples::stress::{
    Backend, Comparison, CoreStatistics, DriveMode, FrameStatistics, MachineInfo, Percentile,
    RunConfig, Scenario, StepStatistics, StressMetrics, StressReport, compare_p95, compare_reports,
    nearest_rank_percentile,
};

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

/// Creates a compact report with caller-selected comparison boundary values.
fn comparison_report(
    scenario: Scenario,
    seed: u64,
    frame_p95: f64,
    step_p95: f64,
    unstable_bodies: usize,
) -> StressReport {
    StressReport::new(
        "abc1234".to_owned(),
        MachineInfo {
            cpu: "test cpu".to_owned(),
            cores: 8,
            os: "linux".to_owned(),
            gpu: None,
        },
        RunConfig {
            backend: Backend::Rapier3d,
            scenario,
            grid: [16, 16],
            count: 256,
            spacing_m: 1.5,
            creature: bevy_ragdoll_examples::stress::Creature::Human,
            mode: DriveMode::Limp,
            trigger_at_s: 2.0,
            duration_s: 10.0,
            warmup_s: 1.0,
            fixed_hz: 60,
            substeps: Some(1),
            budget: None,
            seed,
            headless: true,
            deterministic: false,
        },
        StressMetrics {
            frame_ms: FrameStatistics {
                p50: frame_p95,
                p95: frame_p95,
                p99: frame_p95,
                max: frame_p95,
            },
            step_ms: StepStatistics {
                p50: step_p95,
                p95: step_p95,
                max: step_p95,
            },
            core_ms: CoreStatistics::default(),
            unstable_bodies,
            ..StressMetrics::default()
        },
    )
}

/// Matching reports accept the exact ten percent frame and step boundaries.
#[test]
fn matching_reports_accept_exactly_ten_percent_regression() {
    let baseline = comparison_report(Scenario::Grid, 42, 100.0, 40.0, 0);
    let candidate = comparison_report(Scenario::Grid, 42, 110.0, 44.0, 0);

    assert!(compare_reports(&[candidate], &[baseline]).is_empty());
}

/// Comparison reports timing regressions, invalid timings, and unstable bodies.
#[test]
fn matching_reports_collect_each_failed_metric() {
    let baseline = comparison_report(Scenario::Grid, 42, 100.0, 40.0, 0);
    let candidate = comparison_report(Scenario::Grid, 42, 110.1, 44.04, 1);
    let issues = compare_reports(&[candidate], &[baseline]);

    assert_eq!(
        issues
            .iter()
            .map(|issue| issue.metric.as_str())
            .collect::<Vec<_>>(),
        ["frame_ms.p95", "step_ms.p95", "unstable_bodies"]
    );
}

/// Comparison rejects invalid timing values and stops after a missing baseline.
#[test]
fn comparison_rejects_invalid_timings_and_missing_configurations() {
    let baseline = comparison_report(Scenario::Grid, 42, 100.0, 40.0, 0);
    let invalid = comparison_report(Scenario::Grid, 42, f64::NAN, -1.0, 0);
    let invalid_issues = compare_reports(&[invalid], std::slice::from_ref(&baseline));
    assert_eq!(invalid_issues.len(), 2);
    assert!(
        invalid_issues
            .iter()
            .all(|issue| issue.detail == "timings must be finite and nonnegative")
    );

    let missing = comparison_report(Scenario::Pile, 42, 1.0, 1.0, 0);
    let missing_issues = compare_reports(&[missing], &[baseline]);
    assert_eq!(missing_issues.len(), 1);
    assert_eq!(missing_issues[0].metric, "baseline");
}

/// Schema one rejects unsupported numeric versions before deserializing metrics.
#[test]
fn stress_report_rejects_unknown_schema_versions() {
    let result = serde_json::from_str::<StressReport>(r#"{"schema":2}"#);

    assert!(result.is_err());
}
