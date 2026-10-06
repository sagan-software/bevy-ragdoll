//! Closed stress scenario, backend, creature, and drive configuration values.

use std::str::FromStr;

use serde::{Deserialize, Serialize};

/// Physics backend selected for one stress application process.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Backend {
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

impl std::fmt::Display for Backend {
    /// Writes the stable command-line spelling for the backend.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
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
pub enum Scenario {
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
    /// Applies Phase 7 hit reactions to a ring of characters.
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

impl std::fmt::Display for Scenario {
    /// Writes the stable command-line spelling for the scenario.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
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
pub enum Creature {
    /// The TGF human profile available in Phase 6.
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

impl std::fmt::Display for Creature {
    /// Writes the stable command-line spelling for the creature profile.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        formatter.write_str(match self {
            Self::Human => "human",
        })
    }
}

/// Drive policy applied after a scenario trigger.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DriveMode {
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

impl std::fmt::Display for DriveMode {
    /// Writes the stable command-line spelling for the drive mode.
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
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
pub struct RunConfig {
    /// Backend enabled for the child process.
    pub backend: Backend,
    /// Selected simulation scenario.
    pub scenario: Scenario,
    /// Grid columns and rows, retained for scenarios that use characters in a grid.
    pub grid: [u32; 2],
    /// Number of characters for pile and shooting scenarios.
    pub count: usize,
    /// Horizontal grid spacing in metres.
    pub spacing_m: f32,
    /// Character profile selected for this phase.
    pub creature: Creature,
    /// Drive mode used after the scenario trigger.
    pub mode: DriveMode,
    /// Seconds from start to the default activation trigger.
    pub trigger_at_s: f64,
    /// Measurement duration in seconds after activation.
    pub duration_s: f64,
    /// Initial seconds excluded from measurements.
    pub warmup_s: f64,
    /// Fixed simulation frequency in hertz.
    pub fixed_hz: u32,
    /// Backend solver substeps, when explicitly configured.
    pub substeps: Option<u32>,
    /// Optional cap on dynamic bodies; absence means unlimited.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub budget: Option<usize>,
    /// Seed used to initialize deterministic stress inputs.
    pub seed: u64,
    /// Whether the process runs without a window.
    pub headless: bool,
    /// Whether backend deterministic settings were requested.
    pub deterministic: bool,
}
