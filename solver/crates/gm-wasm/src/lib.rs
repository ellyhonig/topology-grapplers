//! gm-wasm: browser-facing API. One `Engine` holds the parsed database, the
//! solver, and the current state; the frontend feeds effector targets every
//! frame and reads back the pose plus diagnostics.
//!
//! Effectors arrive as a flat Float64Array [player, joint, x, y, z, stiffness]*n
//! to keep the per-frame boundary allocation-free; diagnostics and reports are
//! JSON strings (read at UI rate, not solver rate).

use gm_core::{DbEntry, PlayerId, PlayerJoint, Pose};
use gm_solver::{Effector, Solver, SolverConfig, SolverState};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct Engine {
    entries: Vec<DbEntry>,
    solver: Option<Solver>,
    state: Option<SolverState>,
    config: SolverConfig,
    floors: Vec<f64>,
    last_diagnostics: String,
}

fn err_js(e: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&e.to_string())
}

#[wasm_bindgen]
impl Engine {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Engine {
        Engine {
            entries: Vec::new(),
            solver: None,
            state: None,
            config: SolverConfig::default(),
            floors: Vec::new(),
            last_diagnostics: String::new(),
        }
    }

    /// Patch solver config fields from a JSON object, e.g. {"gravity": 0.0}.
    /// Persists across loadEntry/loadPose calls.
    #[wasm_bindgen(js_name = setConfig)]
    pub fn set_config(&mut self, json: &str) -> Result<(), JsValue> {
        let mut value = serde_json::to_value(self.config).map_err(err_js)?;
        let patch: serde_json::Value = serde_json::from_str(json).map_err(err_js)?;
        let (Some(obj), Some(patch_obj)) = (value.as_object_mut(), patch.as_object()) else {
            return Err(err_js("config patch must be a JSON object"));
        };
        for (k, v) in patch_obj {
            if !obj.contains_key(k) {
                return Err(err_js(format!("unknown config field: {}", k)));
            }
            obj.insert(k.clone(), v.clone());
        }
        self.config = serde_json::from_value(value).map_err(err_js)?;
        if let Some(solver) = self.solver.as_mut() {
            solver.config = self.config;
        }
        Ok(())
    }

    /// Parse a GrappleMap database. Returns JSON:
    /// [{ index, name, frames }] for every entry (positions have frames == 1).
    #[wasm_bindgen(js_name = loadDatabase)]
    pub fn load_database(&mut self, text: &str) -> Result<String, JsValue> {
        self.entries = gm_core::parse_database(text).map_err(err_js)?;
        let listing: Vec<serde_json::Value> = self
            .entries
            .iter()
            .enumerate()
            .map(|(i, e)| {
                serde_json::json!({
                    "index": i,
                    "name": e.name().replace("\\n", " "),
                    "frames": e.frames.len(),
                })
            })
            .collect();
        serde_json::to_string(&listing).map_err(err_js)
    }

    /// Load frame 0 of a database entry into the solver.
    #[wasm_bindgen(js_name = loadEntry)]
    pub fn load_entry(&mut self, index: usize) -> Result<(), JsValue> {
        let entry = self.entries.get(index).ok_or_else(|| err_js("bad entry index"))?;
        let pose = *entry.frames.first().ok_or_else(|| err_js("entry has no frames"))?;
        self.load_pose_internal(pose);
        Ok(())
    }

    /// Load a pose from a flat [x,y,z]*46 array (player 0 joints then player 1).
    #[wasm_bindgen(js_name = loadPose)]
    pub fn load_pose(&mut self, data: &[f64]) -> Result<(), JsValue> {
        let pose = Pose::from_flat(data).ok_or_else(|| err_js("expected 138 floats"))?;
        self.load_pose_internal(pose);
        Ok(())
    }

    /// Current pose as a flat [x,y,z]*46 array.
    #[wasm_bindgen(js_name = pose)]
    pub fn pose_flat(&self) -> Vec<f64> {
        self.state.map(|s| s.pose.to_flat()).unwrap_or_default()
    }

    /// Hard-pin joints: flat [player, joint]*n. Pass empty to clear.
    #[wasm_bindgen(js_name = setPins)]
    pub fn set_pins(&mut self, data: &[f64]) -> Result<(), JsValue> {
        let solver = self.solver.as_mut().ok_or_else(|| err_js("no pose loaded"))?;
        let mut pins = Vec::with_capacity(data.len() / 2);
        for chunk in data.chunks_exact(2) {
            pins.push(parse_joint(chunk[0], chunk[1])?);
        }
        solver.set_pins(pins);
        Ok(())
    }

    /// Advance one frame. `effectors` is a flat
    /// [player, joint, x, y, z, stiffness]*n array. Returns the new pose flat.
    pub fn step(&mut self, effectors: &[f64], dt: f64) -> Result<Vec<f64>, JsValue> {
        let solver = self.solver.as_ref().ok_or_else(|| err_js("no pose loaded"))?;
        let state = self.state.as_ref().ok_or_else(|| err_js("no pose loaded"))?;
        let mut eff = Vec::with_capacity(effectors.len() / 6);
        for chunk in effectors.chunks_exact(6) {
            eff.push(Effector {
                joint: parse_joint(chunk[0], chunk[1])?,
                target: gm_core::v3(chunk[2], chunk[3], chunk[4]),
                stiffness: chunk[5],
            });
        }
        let (next, diag) = solver.step(state, &eff, dt);
        self.state = Some(next);
        self.last_diagnostics = serde_json::to_string(&diag).map_err(err_js)?;
        Ok(next.pose.to_flat())
    }

    /// Diagnostics JSON from the most recent step.
    pub fn diagnostics(&self) -> String {
        self.last_diagnostics.clone()
    }

    /// Full validation report (JSON) for the current pose.
    pub fn validate(&self) -> Result<String, JsValue> {
        let solver = self.solver.as_ref().ok_or_else(|| err_js("no pose loaded"))?;
        let state = self.state.as_ref().ok_or_else(|| err_js("no pose loaded"))?;
        let report = gm_validate::validate_pose(
            &state.pose,
            solver.bones(),
            solver.capsules(),
            solver.pairs(),
            &self.floors,
            &gm_validate::Tolerances::default(),
        );
        serde_json::to_string(&report).map_err(err_js)
    }

    /// Topology summary (JSON): total writhe of the most entangled chain pairs.
    #[wasm_bindgen(js_name = topologySummary)]
    pub fn topology_summary(&self) -> Result<String, JsValue> {
        let state = self.state.as_ref().ok_or_else(|| err_js("no pose loaded"))?;
        let chains = gm_topology::player_chains();
        let mut rows = Vec::new();
        for i in 0..chains.len() {
            for j in (i + 1)..chains.len() {
                let w = gm_topology::matrix_sum(&gm_topology::writhe_matrix(
                    &state.pose,
                    &chains[i],
                    &chains[j],
                ));
                if w.abs() > 0.05 {
                    rows.push(serde_json::json!({
                        "a": format!("p{} {}", chains[i].player.index(), chains[i].chain.id),
                        "b": format!("p{} {}", chains[j].player.index(), chains[j].chain.id),
                        "writhe": w,
                    }));
                }
            }
        }
        rows.sort_by(|x, y| {
            y["writhe"].as_f64().unwrap().abs().total_cmp(&x["writhe"].as_f64().unwrap().abs())
        });
        serde_json::to_string(&rows).map_err(err_js)
    }

    /// Joint names in index order (JSON array), for building UIs.
    #[wasm_bindgen(js_name = jointNames)]
    pub fn joint_names(&self) -> String {
        let names: Vec<&str> = gm_core::Joint::ALL.iter().map(|j| j.name()).collect();
        serde_json::to_string(&names).unwrap()
    }
}

impl Engine {
    fn load_pose_internal(&mut self, pose: Pose) {
        let solver = Solver::new(&pose, self.config);
        self.floors = gm_validate::penetration_floors(&pose, solver.capsules(), solver.pairs());
        self.state = Some(SolverState::from_pose(pose));
        self.solver = Some(solver);
        self.last_diagnostics = String::new();
    }
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}

fn parse_joint(player: f64, joint: f64) -> Result<PlayerJoint, JsValue> {
    let p = player as usize;
    let j = joint as usize;
    if p >= 2 {
        return Err(err_js("player must be 0 or 1"));
    }
    Ok(PlayerJoint {
        player: PlayerId(p as u8),
        joint: gm_core::Joint::from_index(j).ok_or_else(|| err_js("bad joint index"))?,
    })
}
