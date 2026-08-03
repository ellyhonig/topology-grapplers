//! Stable C ABI for the Unity native plug-in.
//!
//! All exported calls contain panics, validate pointers and lengths, and copy
//! data into caller-owned buffers. An engine is intentionally single-threaded.

use gm_core::{DbEntry, Joint, PlayerId, PlayerJoint, Pose};
use gm_solver::{
    Effector, Solver, SolverConfig, SolverState, StepDiagnostics, RUNTIME_GRIP_STRENGTH,
};
use std::cell::RefCell;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::ptr;
use std::slice;

pub const ABI_VERSION: u32 = 4;
pub const POSE_LEN: usize = 138;
pub const ENTRY_NAME_CAPACITY: usize = 256;

#[repr(i32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GmResult {
    Ok = 0,
    NullPointer = 1,
    InvalidArgument = 2,
    InvalidState = 3,
    NotFound = 4,
    ParseError = 5,
    SerializationError = 6,
    Panic = 7,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GmPlayerJoint {
    pub player: u32,
    pub joint: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GmEffector {
    pub player: u32,
    pub joint: u32,
    pub target_x: f64,
    pub target_y: f64,
    pub target_z: f64,
    pub stiffness: f64,
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct GmEntryInfo {
    pub frame_count: u32,
    pub name_len: u32,
    pub name_utf8: [u8; ENTRY_NAME_CAPACITY],
}

impl Default for GmEntryInfo {
    fn default() -> Self {
        Self { frame_count: 0, name_len: 0, name_utf8: [0; ENTRY_NAME_CAPACITY] }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GmStepDiagnostics {
    pub min_clearance: f64,
    pub max_bone_error: f64,
    pub max_hinge_violation: f64,
    pub max_writhe_jump: f64,
    pub max_effector_residual: f64,
    pub contact_count: u32,
    pub retries: u32,
    pub rejected: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GmGripCandidate {
    pub valid: u32,
    pub capsule: u32,
    pub target_player: u32,
    pub end_a: u32,
    pub end_b: u32,
    pub closest_x: f64,
    pub closest_y: f64,
    pub closest_z: f64,
    pub surface_gap: f64,
    pub palm_alignment: f64,
    pub wrap_alignment: f64,
    pub score: f64,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct GmGripState {
    /// 0 none, 1 acquiring, 2 holding, 3 slipping, 4 broken.
    pub status: u32,
    pub capsule: u32,
    pub target_player: u32,
    pub end_a: u32,
    pub end_b: u32,
    /// 0 none, 1 positive/counter-clockwise, 2 negative/clockwise.
    pub wrap_direction: u32,
    pub normalized_strain: f64,
    pub normalized_release: f64,
    pub normalized_wrap: f64,
    pub normalized_contact: f64,
    pub normalized_coverage: f64,
    pub normalized_strength: f64,
    pub selected_writhe: f64,
    pub alternate_writhe: f64,
}

pub struct GmEngine {
    entries: Vec<DbEntry>,
    solver: Option<Solver>,
    state: Option<SolverState>,
    config: SolverConfig,
    floors: Vec<f64>,
    last_error: RefCell<String>,
}

impl GmEngine {
    fn new() -> Self {
        Self {
            entries: Vec::new(), solver: None, state: None,
            config: SolverConfig::default(), floors: Vec::new(),
            last_error: RefCell::new(String::new()),
        }
    }

    fn fail(&self, code: GmResult, message: impl Into<String>) -> GmResult {
        *self.last_error.borrow_mut() = message.into();
        code
    }

    fn clear_error(&self) { self.last_error.borrow_mut().clear(); }

    fn load_pose_internal(&mut self, pose: Pose) {
        let solver = Solver::new(&pose, self.config);
        self.floors = gm_validate::penetration_floors(&pose, solver.capsules(), solver.pairs());
        self.state = Some(SolverState::from_pose(pose));
        self.solver = Some(solver);
    }
}

fn parse_joint(player: u32, joint: u32) -> Result<PlayerJoint, &'static str> {
    if player >= 2 { return Err("player must be 0 or 1"); }
    let joint = Joint::from_index(joint as usize).ok_or("joint index must be in [0, 23)")?;
    Ok(PlayerJoint { player: PlayerId(player as u8), joint })
}

fn panic_message(payload: Box<dyn std::any::Any + Send>) -> String {
    if let Some(s) = payload.downcast_ref::<&str>() { (*s).to_owned() }
    else if let Some(s) = payload.downcast_ref::<String>() { s.clone() }
    else { "unknown Rust panic".to_owned() }
}

unsafe fn with_engine_mut<F>(engine: *mut GmEngine, f: F) -> GmResult
where F: FnOnce(&mut GmEngine) -> GmResult {
    if engine.is_null() { return GmResult::NullPointer; }
    match catch_unwind(AssertUnwindSafe(|| f(&mut *engine))) {
        Ok(result) => result,
        Err(payload) => {
            let engine = &*engine;
            engine.fail(GmResult::Panic, format!("panic contained at native boundary: {}", panic_message(payload)))
        }
    }
}

unsafe fn with_engine<F>(engine: *const GmEngine, f: F) -> GmResult
where F: FnOnce(&GmEngine) -> GmResult {
    if engine.is_null() { return GmResult::NullPointer; }
    match catch_unwind(AssertUnwindSafe(|| f(&*engine))) {
        Ok(result) => result,
        Err(payload) => {
            let engine = &*engine;
            engine.fail(GmResult::Panic, format!("panic contained at native boundary: {}", panic_message(payload)))
        }
    }
}

unsafe fn bytes<'a>(data: *const u8, len: usize) -> Result<&'a [u8], GmResult> {
    if len == 0 { return Ok(&[]); }
    if data.is_null() { return Err(GmResult::NullPointer); }
    Ok(slice::from_raw_parts(data, len))
}

unsafe fn values<'a, T>(data: *const T, len: usize) -> Result<&'a [T], GmResult> {
    if len == 0 { return Ok(&[]); }
    if data.is_null() { return Err(GmResult::NullPointer); }
    Ok(slice::from_raw_parts(data, len))
}

unsafe fn copy_string(value: &str, out: *mut u8, capacity: usize) -> usize {
    let required = value.len();
    if capacity == 0 || out.is_null() { return required; }
    let count = required.min(capacity - 1);
    ptr::copy_nonoverlapping(value.as_ptr(), out, count);
    *out.add(count) = 0;
    required
}

#[no_mangle]
pub extern "C" fn gm_abi_version() -> u32 { ABI_VERSION }

#[no_mangle]
pub extern "C" fn gm_engine_create() -> *mut GmEngine {
    catch_unwind(|| Box::into_raw(Box::new(GmEngine::new()))).unwrap_or(ptr::null_mut())
}

#[no_mangle]
pub unsafe extern "C" fn gm_engine_destroy(engine: *mut GmEngine) {
    if engine.is_null() { return; }
    let _ = catch_unwind(AssertUnwindSafe(|| drop(Box::from_raw(engine))));
}

#[no_mangle]
pub unsafe extern "C" fn gm_set_config_json(engine: *mut GmEngine, utf8: *const u8, len: usize) -> GmResult {
    with_engine_mut(engine, |e| {
        let raw = match bytes(utf8, len) { Ok(v) => v, Err(c) => return e.fail(c, "config pointer is null") };
        let text = match std::str::from_utf8(raw) { Ok(v) => v, Err(x) => return e.fail(GmResult::ParseError, x.to_string()) };
        let mut value = match serde_json::to_value(e.config) { Ok(v) => v, Err(x) => return e.fail(GmResult::SerializationError, x.to_string()) };
        let patch: serde_json::Value = match serde_json::from_str(text) { Ok(v) => v, Err(x) => return e.fail(GmResult::ParseError, x.to_string()) };
        let (Some(config), Some(patch)) = (value.as_object_mut(), patch.as_object()) else { return e.fail(GmResult::InvalidArgument, "config patch must be a JSON object"); };
        for (key, val) in patch {
            if !config.contains_key(key) { return e.fail(GmResult::InvalidArgument, format!("unknown config field: {key}")); }
            config.insert(key.clone(), val.clone());
        }
        e.config = match serde_json::from_value(value) { Ok(v) => v, Err(x) => return e.fail(GmResult::InvalidArgument, x.to_string()) };
        if let Some(solver) = e.solver.as_mut() { solver.config = e.config; }
        e.clear_error(); GmResult::Ok
    })
}

#[no_mangle]
pub unsafe extern "C" fn gm_load_database(engine: *mut GmEngine, utf8: *const u8, len: usize) -> GmResult {
    with_engine_mut(engine, |e| {
        let raw = match bytes(utf8, len) { Ok(v) => v, Err(c) => return e.fail(c, "database pointer is null") };
        let text = match std::str::from_utf8(raw) { Ok(v) => v, Err(x) => return e.fail(GmResult::ParseError, x.to_string()) };
        e.entries = match gm_core::parse_database(text) { Ok(v) => v, Err(x) => return e.fail(GmResult::ParseError, x.to_string()) };
        e.clear_error(); GmResult::Ok
    })
}

#[no_mangle]
pub unsafe extern "C" fn gm_entry_count(engine: *const GmEngine) -> usize {
    if engine.is_null() { return 0; }
    catch_unwind(AssertUnwindSafe(|| (*engine).entries.len())).unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn gm_entry_info(engine: *const GmEngine, index: usize, out: *mut GmEntryInfo) -> GmResult {
    with_engine(engine, |e| {
        if out.is_null() { return e.fail(GmResult::NullPointer, "entry info output is null"); }
        let Some(entry) = e.entries.get(index) else { return e.fail(GmResult::NotFound, "bad entry index"); };
        let mut info = GmEntryInfo::default();
        info.frame_count = entry.frames.len().min(u32::MAX as usize) as u32;
        let name = entry.name().replace("\\n", " ");
        let mut count = name.len().min(ENTRY_NAME_CAPACITY - 1);
        while !name.is_char_boundary(count) { count -= 1; }
        info.name_utf8[..count].copy_from_slice(&name.as_bytes()[..count]);
        info.name_len = count as u32;
        *out = info;
        e.clear_error(); GmResult::Ok
    })
}

#[no_mangle]
pub unsafe extern "C" fn gm_load_entry(engine: *mut GmEngine, index: usize) -> GmResult {
    with_engine_mut(engine, |e| {
        let pose = match e.entries.get(index).and_then(|entry| entry.frames.first()).copied() {
            Some(v) => v, None => return e.fail(GmResult::NotFound, "bad entry index or entry has no frames"),
        };
        e.load_pose_internal(pose); e.clear_error(); GmResult::Ok
    })
}

#[no_mangle]
pub unsafe extern "C" fn gm_load_pose(engine: *mut GmEngine, xyz: *const f64, len: usize) -> GmResult {
    with_engine_mut(engine, |e| {
        if len != POSE_LEN { return e.fail(GmResult::InvalidArgument, "pose length must be 138"); }
        let data = match values(xyz, len) { Ok(v) => v, Err(c) => return e.fail(c, "pose pointer is null") };
        let Some(pose) = Pose::from_flat(data) else { return e.fail(GmResult::InvalidArgument, "pose must contain 138 finite floats"); };
        if !pose.is_finite() { return e.fail(GmResult::InvalidArgument, "pose values must be finite"); }
        e.load_pose_internal(pose); e.clear_error(); GmResult::Ok
    })
}

#[no_mangle]
pub unsafe extern "C" fn gm_get_pose(engine: *const GmEngine, out: *mut f64, len: usize) -> GmResult {
    with_engine(engine, |e| {
        if len != POSE_LEN { return e.fail(GmResult::InvalidArgument, "pose output length must be 138"); }
        if out.is_null() { return e.fail(GmResult::NullPointer, "pose output is null"); }
        let Some(state) = e.state.as_ref() else { return e.fail(GmResult::InvalidState, "no pose loaded"); };
        ptr::copy_nonoverlapping(state.pose.to_flat().as_ptr(), out, POSE_LEN);
        e.clear_error(); GmResult::Ok
    })
}

#[no_mangle]
pub unsafe extern "C" fn gm_set_pins(engine: *mut GmEngine, pins: *const GmPlayerJoint, count: usize) -> GmResult {
    with_engine_mut(engine, |e| {
        let raw = match values(pins, count) { Ok(v) => v, Err(c) => return e.fail(c, "pins pointer is null") };
        let mut parsed = Vec::with_capacity(count);
        for pin in raw { match parse_joint(pin.player, pin.joint) { Ok(v) => parsed.push(v), Err(x) => return e.fail(GmResult::InvalidArgument, x) } }
        let Some(solver) = e.solver.as_mut() else { return e.fail(GmResult::InvalidState, "no pose loaded"); };
        solver.set_pins(parsed); e.clear_error(); GmResult::Ok
    })
}

#[no_mangle]
pub unsafe extern "C" fn gm_release_grips(engine: *mut GmEngine) -> GmResult {
    with_engine_mut(engine, |e| {
        let Some(solver) = e.solver.as_mut() else { return e.fail(GmResult::InvalidState, "no pose loaded"); };
        solver.release_grips(); e.clear_error(); GmResult::Ok
    })
}

#[no_mangle]
pub unsafe extern "C" fn gm_grip_count(engine: *const GmEngine) -> usize {
    if engine.is_null() { return 0; }
    catch_unwind(AssertUnwindSafe(|| (*engine).solver.as_ref().map_or(0, Solver::grip_count))).unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn gm_query_grip_candidate(
    engine: *const GmEngine,
    player: u32,
    joint: u32,
    palm_x: f64,
    palm_y: f64,
    palm_z: f64,
    aim_x: f64,
    aim_y: f64,
    aim_z: f64,
    max_gap: f64,
    out: *mut GmGripCandidate,
) -> GmResult {
    with_engine(engine, |e| {
        if out.is_null() {
            return e.fail(GmResult::NullPointer, "grip candidate output is null");
        }
        let hand = match parse_joint(player, joint) {
            Ok(value) => value,
            Err(message) => return e.fail(GmResult::InvalidArgument, message),
        };
        if ![palm_x, palm_y, palm_z, aim_x, aim_y, aim_z, max_gap].iter().all(|v| v.is_finite())
            || max_gap <= 0.0
        {
            return e.fail(GmResult::InvalidArgument, "grip query values must be finite and max_gap positive");
        }
        let (Some(solver), Some(state)) = (e.solver.as_ref(), e.state.as_ref()) else {
            return e.fail(GmResult::InvalidState, "no pose loaded");
        };
        let mut result = GmGripCandidate::default();
        if let Some(candidate) = solver.query_grip_candidate(
            state,
            hand,
            gm_core::v3(palm_x, palm_y, palm_z),
            gm_core::v3(aim_x, aim_y, aim_z),
            max_gap,
            0.2,
        ) {
            result.valid = 1;
            result.capsule = candidate.capsule as u32;
            result.target_player = candidate.player.index() as u32;
            result.end_a = candidate.ends[0].index() as u32;
            result.end_b = candidate.ends[1].index() as u32;
            result.closest_x = candidate.closest.x;
            result.closest_y = candidate.closest.y;
            result.closest_z = candidate.closest.z;
            result.surface_gap = candidate.surface_gap;
            result.palm_alignment = candidate.palm_alignment;
            result.wrap_alignment = candidate.wrap_alignment;
            result.score = candidate.score;
        }
        *out = result;
        e.clear_error();
        GmResult::Ok
    })
}

#[no_mangle]
pub unsafe extern "C" fn gm_begin_runtime_grip(
    engine: *mut GmEngine,
    player: u32,
    joint: u32,
    capsule: u32,
    max_gap: f64,
) -> GmResult {
    with_engine_mut(engine, |e| {
        let hand = match parse_joint(player, joint) {
            Ok(value) => value,
            Err(message) => return e.fail(GmResult::InvalidArgument, message),
        };
        if !max_gap.is_finite() || max_gap <= 0.0 {
            return e.fail(GmResult::InvalidArgument, "max_gap must be finite and positive");
        }
        let started = match (e.solver.as_mut(), e.state.as_mut()) {
            (Some(solver), Some(state)) => {
                solver.begin_runtime_grip(state, hand, capsule as usize, max_gap)
            }
            _ => return e.fail(GmResult::InvalidState, "no pose loaded"),
        };
        if !started {
            return e.fail(GmResult::NotFound, "grip target is stale, remote, disallowed, or unavailable");
        }
        e.clear_error();
        GmResult::Ok
    })
}

#[no_mangle]
pub unsafe extern "C" fn gm_end_runtime_grip(
    engine: *mut GmEngine,
    player: u32,
    joint: u32,
) -> GmResult {
    with_engine_mut(engine, |e| {
        let hand = match parse_joint(player, joint) {
            Ok(value) => value,
            Err(message) => return e.fail(GmResult::InvalidArgument, message),
        };
        match (e.solver.as_ref(), e.state.as_mut()) {
            (Some(solver), Some(state)) => { solver.end_runtime_grip(state, hand); }
            _ => return e.fail(GmResult::InvalidState, "no pose loaded"),
        }
        e.clear_error();
        GmResult::Ok
    })
}

#[no_mangle]
pub unsafe extern "C" fn gm_release_runtime_grips(engine: *mut GmEngine) -> GmResult {
    with_engine_mut(engine, |e| {
        match (e.solver.as_ref(), e.state.as_mut()) {
            (Some(solver), Some(state)) => solver.release_runtime_grips(state),
            _ => return e.fail(GmResult::InvalidState, "no pose loaded"),
        }
        e.clear_error();
        GmResult::Ok
    })
}

#[no_mangle]
pub unsafe extern "C" fn gm_runtime_grip_state(
    engine: *const GmEngine,
    player: u32,
    joint: u32,
    out: *mut GmGripState,
) -> GmResult {
    with_engine(engine, |e| {
        if out.is_null() {
            return e.fail(GmResult::NullPointer, "runtime grip state output is null");
        }
        let hand = match parse_joint(player, joint) {
            Ok(value) => value,
            Err(message) => return e.fail(GmResult::InvalidArgument, message),
        };
        let (Some(solver), Some(state)) = (e.solver.as_ref(), e.state.as_ref()) else {
            return e.fail(GmResult::InvalidState, "no pose loaded");
        };
        let mut result = GmGripState::default();
        if let Some(grip) = solver.runtime_grip_state(state, hand) {
            result.status = if grip.broken {
                4
            } else if grip.wrap < 1.0 {
                1
            } else if grip.release > 0.0 {
                3
            } else {
                2
            };
            result.capsule = grip.capsule as u32;
            result.target_player = grip.player.index() as u32;
            result.end_a = grip.ends[0].index() as u32;
            result.end_b = grip.ends[1].index() as u32;
            result.wrap_direction = if grip.direction >= 0.0 { 1 } else { 2 };
            result.normalized_strain =
                (grip.strain / RUNTIME_GRIP_STRENGTH).clamp(0.0, 1.0);
            result.normalized_release = (grip.release / 0.25).clamp(0.0, 1.0);
            result.normalized_wrap = grip.wrap.clamp(0.0, 1.0);
            result.normalized_contact = grip.contact.clamp(0.0, 1.0);
            result.normalized_coverage = grip.coverage.clamp(0.0, 1.0);
            result.normalized_strength = grip.strength.clamp(0.0, 1.0);
            result.selected_writhe = grip.selected_writhe;
            result.alternate_writhe = grip.alternate_writhe;
        }
        *out = result;
        e.clear_error();
        GmResult::Ok
    })
}

#[no_mangle]
pub unsafe extern "C" fn gm_step(engine: *mut GmEngine, effectors: *const GmEffector, effector_count: usize, dt: f64, out_xyz: *mut f64, out_len: usize, out_diag: *mut GmStepDiagnostics) -> GmResult {
    with_engine_mut(engine, |e| {
        if out_len != POSE_LEN { return e.fail(GmResult::InvalidArgument, "pose output length must be 138"); }
        if out_xyz.is_null() || out_diag.is_null() { return e.fail(GmResult::NullPointer, "step output pointer is null"); }
        if !dt.is_finite() || dt <= 0.0 { return e.fail(GmResult::InvalidArgument, "dt must be finite and positive"); }
        let raw = match values(effectors, effector_count) { Ok(v) => v, Err(c) => return e.fail(c, "effectors pointer is null") };
        let mut parsed = Vec::with_capacity(effector_count);
        for item in raw {
            let joint = match parse_joint(item.player, item.joint) { Ok(v) => v, Err(x) => return e.fail(GmResult::InvalidArgument, x) };
            if ![item.target_x,item.target_y,item.target_z,item.stiffness].iter().all(|v| v.is_finite()) { return e.fail(GmResult::InvalidArgument, "effector values must be finite"); }
            if !(0.0..=1.0).contains(&item.stiffness) { return e.fail(GmResult::InvalidArgument, "effector stiffness must be in [0, 1]"); }
            parsed.push(Effector { joint, target: gm_core::v3(item.target_x,item.target_y,item.target_z), stiffness: item.stiffness });
        }
        let (Some(solver), Some(state)) = (e.solver.as_ref(), e.state.as_ref()) else { return e.fail(GmResult::InvalidState, "no pose loaded"); };
        let (next, diag) = solver.step(state, &parsed, dt);
        ptr::copy_nonoverlapping(next.pose.to_flat().as_ptr(), out_xyz, POSE_LEN);
        *out_diag = diagnostics(&diag);
        e.state = Some(next); e.clear_error(); GmResult::Ok
    })
}

fn diagnostics(diag: &StepDiagnostics) -> GmStepDiagnostics {
    GmStepDiagnostics {
        min_clearance: diag.min_clearance, max_bone_error: diag.max_bone_error,
        max_hinge_violation: diag.max_hinge_violation, max_writhe_jump: diag.max_writhe_jump,
        max_effector_residual: diag.effector_residuals.iter().copied().fold(0.0, f64::max),
        contact_count: diag.contact_count.min(u32::MAX as usize) as u32,
        retries: diag.retries.min(u32::MAX as usize) as u32,
        rejected: u32::from(diag.rejected),
    }
}

fn validation_json(e: &GmEngine) -> Result<String, GmResult> {
    let (Some(solver), Some(state)) = (e.solver.as_ref(), e.state.as_ref()) else { return Err(GmResult::InvalidState); };
    let report = gm_validate::validate_pose(&state.pose, solver.bones(), solver.capsules(), solver.pairs(), &e.floors, &gm_validate::Tolerances::default());
    serde_json::to_string(&report).map_err(|_| GmResult::SerializationError)
}

fn topology_json(e: &GmEngine) -> Result<String, GmResult> {
    let Some(state) = e.state.as_ref() else { return Err(GmResult::InvalidState); };
    let chains = gm_topology::player_chains();
    let mut rows = Vec::new();
    for i in 0..chains.len() { for j in (i + 1)..chains.len() {
        let w = gm_topology::matrix_sum(&gm_topology::writhe_matrix(&state.pose, &chains[i], &chains[j]));
        if w.abs() > 0.05 { rows.push(serde_json::json!({"a":format!("p{} {}",chains[i].player.index(),chains[i].chain.id),"b":format!("p{} {}",chains[j].player.index(),chains[j].chain.id),"writhe":w})); }
    }}
    rows.sort_by(|a,b| b["writhe"].as_f64().unwrap_or(0.0).abs().total_cmp(&a["writhe"].as_f64().unwrap_or(0.0).abs()));
    serde_json::to_string(&rows).map_err(|_| GmResult::SerializationError)
}

unsafe fn report_json(engine: *mut GmEngine, out: *mut u8, capacity: usize, make: fn(&GmEngine)->Result<String,GmResult>) -> usize {
    if engine.is_null() { return 0; }
    match catch_unwind(AssertUnwindSafe(|| {
        let e = &*engine;
        match make(e) { Ok(v) => { e.clear_error(); copy_string(&v,out,capacity) }, Err(c) => { e.fail(c,"no pose loaded or report serialization failed"); 0 } }
    })) { Ok(v) => v, Err(payload) => { (*engine).fail(GmResult::Panic,panic_message(payload)); 0 } }
}

#[no_mangle]
pub unsafe extern "C" fn gm_validation_json(engine:*mut GmEngine,out:*mut u8,capacity:usize)->usize { report_json(engine,out,capacity,validation_json) }
#[no_mangle]
pub unsafe extern "C" fn gm_topology_json(engine:*mut GmEngine,out:*mut u8,capacity:usize)->usize { report_json(engine,out,capacity,topology_json) }

#[no_mangle]
pub unsafe extern "C" fn gm_last_error(engine:*const GmEngine,out:*mut u8,capacity:usize)->usize {
    if engine.is_null() { return 0; }
    catch_unwind(AssertUnwindSafe(|| copy_string(&(*engine).last_error.borrow(),out,capacity))).unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn gm_build_id(out:*mut u8,capacity:usize)->usize {
    const BUILD_ID: &str = match option_env!("GM_BUILD_ID") { Some(v)=>v, None=>"development" };
    catch_unwind(AssertUnwindSafe(|| copy_string(BUILD_ID,out,capacity))).unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::mem::{align_of, size_of};

    fn pose() -> [f64; POSE_LEN] {
        let mut p = [0.0; POSE_LEN];
        for i in 0..46 { p[i*3]=(i%23) as f64*0.01; p[i*3+1]=1.0+(i%5) as f64*0.05; p[i*3+2]=(i/23) as f64; }
        p
    }

    #[test] fn abi_layouts_are_stable() {
        assert_eq!(size_of::<GmPlayerJoint>(),8); assert_eq!(align_of::<GmPlayerJoint>(),4);
        assert_eq!(size_of::<GmEffector>(),40); assert_eq!(align_of::<GmEffector>(),8);
        assert_eq!(size_of::<GmEntryInfo>(),264); assert_eq!(size_of::<GmStepDiagnostics>(),56);
        assert_eq!(size_of::<GmGripCandidate>(),80);
        assert_eq!(size_of::<GmGripState>(),88);
    }
    #[test] fn nulls_and_lengths_are_rejected() { unsafe {
        assert_eq!(gm_load_pose(ptr::null_mut(),ptr::null(),POSE_LEN),GmResult::NullPointer);
        let e=gm_engine_create(); assert!(!e.is_null());
        assert_eq!(gm_load_pose(e,pose().as_ptr(),POSE_LEN-1),GmResult::InvalidArgument);
        assert_eq!(gm_load_pose(e,ptr::null(),POSE_LEN),GmResult::NullPointer);
        gm_engine_destroy(e); gm_engine_destroy(ptr::null_mut());
    }}
    #[test] fn create_load_step_destroy_repeats() { unsafe {
        for _ in 0..32 {
            let e=gm_engine_create(); assert_eq!(gm_load_pose(e,pose().as_ptr(),POSE_LEN),GmResult::Ok);
            let mut out=[0.0;POSE_LEN]; let mut diag=GmStepDiagnostics::default();
            assert_eq!(gm_step(e,ptr::null(),0,1.0/90.0,out.as_mut_ptr(),POSE_LEN,&mut diag),GmResult::Ok);
            assert!(out.iter().all(|v|v.is_finite())); gm_engine_destroy(e);
        }
    }}
    #[test] fn malformed_inputs_set_copyable_error() { unsafe {
        let e=gm_engine_create(); let invalid=b"{";
        assert_eq!(gm_set_config_json(e,invalid.as_ptr(),invalid.len()),GmResult::ParseError);
        let needed=gm_last_error(e,ptr::null_mut(),0); assert!(needed>0);
        let mut text=vec![0u8;needed+1]; assert_eq!(gm_last_error(e,text.as_mut_ptr(),text.len()),needed); assert_eq!(text[needed],0);
        let bad=GmPlayerJoint{player:7,joint:0};
        assert_eq!(gm_set_pins(e,&bad,1),GmResult::InvalidArgument);
        gm_engine_destroy(e);
    }}
    #[test] fn runtime_grip_query_begin_state_and_release() { unsafe {
        let e = gm_engine_create();
        let mut p = pose();
        let wrist = Joint::LeftWrist.index();
        let hand = Joint::LeftHand.index();
        let fingers = Joint::LeftFingers.index();
        let target = Joint::LeftElbow.index();
        for axis in 0..3 {
            p[wrist * 3 + axis] = p[(23 + target) * 3 + axis] - if axis == 2 { 0.03 } else { 0.0 };
            p[hand * 3 + axis] = p[wrist * 3 + axis] + if axis == 1 { 0.08 } else { 0.0 };
            p[fingers * 3 + axis] = p[hand * 3 + axis] + if axis == 0 { 0.05 } else { 0.0 };
        }
        assert_eq!(gm_load_pose(e, p.as_ptr(), POSE_LEN), GmResult::Ok);
        let mut candidate = GmGripCandidate::default();
        assert_eq!(
            gm_query_grip_candidate(
                e, 0, hand as u32,
                p[hand * 3], p[hand * 3 + 1], p[hand * 3 + 2],
                0.0, 0.0, 1.0, 0.08, &mut candidate,
            ),
            GmResult::Ok
        );
        assert_eq!(candidate.valid, 1);
        assert_eq!(
            gm_begin_runtime_grip(e, 0, hand as u32, candidate.capsule, 0.08),
            GmResult::Ok
        );
        let mut state = GmGripState::default();
        assert_eq!(gm_runtime_grip_state(e, 0, hand as u32, &mut state), GmResult::Ok);
        assert_eq!(state.status, 1);
        assert_eq!(gm_end_runtime_grip(e, 0, hand as u32), GmResult::Ok);
        assert_eq!(gm_runtime_grip_state(e, 0, hand as u32, &mut state), GmResult::Ok);
        assert_eq!(state.status, 4);
        gm_engine_destroy(e);
    }}
}
