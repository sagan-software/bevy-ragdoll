//! Child-process sweep execution and Markdown result summaries.

use std::error::Error;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

use serde::Deserialize;

use super::cli::{CharacterCount, ChildRun, GridSize, HeadlessMode, StressCli, SweepSelection};
use super::compare::compare_reports;
use super::config::{Backend, Scenario};
use super::report::StressReport;
use super::runner::{read_reports, source_revision};

/// One checked scenario row read from a RON sweep description.
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

/// RON sweep file containing rows that each run in a child process.
#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SweepFile {
    /// Child-process rows in the requested execution order.
    runs: Vec<SweepRow>,
}

/// Runs each selected configuration in a fresh headless child process.
pub(super) fn run_sweep(cli: &StressCli, selection: &SweepSelection) -> Result<(), Box<dyn Error>> {
    if cli.backend != Backend::Rapier3d {
        return Err("the only compiled Phase 6 backend is rapier3d".into());
    }
    if cli.deterministic.is_some() {
        return Err("deterministic mode needs Phase 15".into());
    }
    let rows = match selection {
        SweepSelection::Default => default_rows(),
        SweepSelection::File(path) => read_sweep_file(path)?,
    };
    if rows.is_empty() {
        return Err("sweep must contain at least one run".into());
    }
    let output_path = sweep_output_path(cli.report.as_deref())?;
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
    for (index, row) in rows.iter().enumerate() {
        let backend = row.backend.unwrap_or(cli.backend);
        let (grid, count) = validate_row(row, cli)?;
        if backend != Backend::Rapier3d {
            return Err(format!("backend {backend} needs a later phase").into());
        }
        let report_path = parent.join(format!(".sweep-{run_id}-{index}.json"));
        let arguments = cli.child_arguments(ChildRun {
            backend,
            scenario: row.scenario,
            grid,
            count,
            report: Some(&report_path),
            screenshot: None,
            headless: Some(HeadlessMode::Enabled),
            deterministic: None,
        });
        let run_index = index + 1;
        let total = rows.len();
        let scenario = row.scenario;
        let population = count.get();
        println!("[sweep {run_index}/{total}] {backend} {scenario} population {population}");
        let status = Command::new(&executable).args(arguments).status()?;
        if !status.success() {
            let _ = std::fs::remove_file(&report_path);
            let scenario = row.scenario;
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
    let encoded = serde_json::to_vec_pretty(&reports)?;
    std::fs::write(&output_path, encoded)?;
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

/// Returns the default stress configurations implemented through Phase 7.
fn default_rows() -> Vec<SweepRow> {
    let mut rows = Vec::with_capacity(9);
    for size in [8, 16, 24, 32] {
        rows.push(SweepRow {
            backend: None,
            scenario: Scenario::Grid,
            grid: Some([size, size]),
            count: None,
        });
    }
    for count in [32, 64, 128] {
        rows.push(SweepRow {
            backend: None,
            scenario: Scenario::Pile,
            grid: None,
            count: Some(count),
        });
    }
    rows.push(SweepRow {
        backend: None,
        scenario: Scenario::Powered,
        grid: Some([16, 16]),
        count: None,
    });
    rows.push(SweepRow {
        backend: None,
        scenario: Scenario::Shooting,
        grid: None,
        count: Some(64),
    });
    println!("skip balance 8x8: needs Phase 9");
    println!("skip mixed: needs Phase 11");
    rows
}

/// Reads and validates one ordered RON sweep definition.
fn read_sweep_file(path: &Path) -> Result<Vec<SweepRow>, Box<dyn Error>> {
    let source = std::fs::read_to_string(path)?;
    let file = ron::from_str::<SweepFile>(&source)?;
    Ok(file.runs)
}

/// Resolves one row's grid and count while preserving the active CLI defaults.
fn validate_row(
    row: &SweepRow,
    defaults: &StressCli,
) -> Result<(GridSize, CharacterCount), Box<dyn Error>> {
    let [columns, rows] = row
        .grid
        .unwrap_or(defaults.grid.as_array().map(|value| value.get()));
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

/// Chooses the requested report path or a target-local date and source revision.
fn sweep_output_path(requested: Option<&Path>) -> Result<PathBuf, Box<dyn Error>> {
    if let Some(path) = requested {
        return Ok(path.to_path_buf());
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
    let target = std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("target"));
    let revision = source_revision();
    Ok(target
        .join("stress")
        .join(format!("{date}-{revision}.json")))
}

/// Prints a compact Markdown table for one ordered report set.
fn print_summary(reports: &[StressReport]) {
    println!(
        "| backend | scenario | characters | frame p95 ms | step p95 ms | core p95 ms | trigger spike ms | bodies | unstable |"
    );
    println!("|---|---:|---:|---:|---:|---:|---:|---:|---:|");
    for report in reports {
        let backend = report.config.backend;
        let scenario = report.config.scenario;
        let characters = report.metrics.characters;
        let frame_p95 = report.metrics.frame_ms.p95;
        let step_p95 = report.metrics.step_ms.p95;
        let core_p95 = report.metrics.core_ms.p95;
        let trigger_spike = report.metrics.trigger_spike_ms;
        let bodies = report.metrics.bodies;
        let unstable = report.metrics.unstable_bodies;
        println!(
            "| {backend} | {scenario} | {characters} | {frame_p95:.3} | {step_p95:.3} | {core_p95:.3} | {trigger_spike:.3} | {bodies} | {unstable} |"
        );
    }
}

#[cfg(test)]
mod tests {
    use super::{Scenario, default_rows};

    #[test]
    fn phase7_default_sweep_includes_shooting_64() {
        let rows = default_rows();
        let shooting = rows
            .iter()
            .find(|row| row.scenario == Scenario::Shooting)
            .expect("Phase 7 adds shooting to the default sweep");

        assert_eq!(rows.len(), 9);
        assert_eq!(shooting.count, Some(64));
        assert_eq!(shooting.grid, None);
    }
}
