/* tslint:disable */
/* eslint-disable */

export class Engine {
    free(): void;
    [Symbol.dispose](): void;
    /**
     * Diagnostics JSON from the most recent step.
     */
    diagnostics(): string;
    /**
     * Joint names in index order (JSON array), for building UIs.
     */
    jointNames(): string;
    /**
     * Parse a GrappleMap database. Returns JSON:
     * [{ index, name, frames }] for every entry (positions have frames == 1).
     */
    loadDatabase(text: string): string;
    /**
     * Load frame 0 of a database entry into the solver.
     */
    loadEntry(index: number): void;
    /**
     * Load a pose from a flat [x,y,z]*46 array (player 0 joints then player 1).
     */
    loadPose(data: Float64Array): void;
    constructor();
    /**
     * Current pose as a flat [x,y,z]*46 array.
     */
    pose(): Float64Array;
    /**
     * Patch solver config fields from a JSON object, e.g. {"gravity": 0.0}.
     * Persists across loadEntry/loadPose calls.
     */
    setConfig(json: string): void;
    /**
     * Hard-pin joints: flat [player, joint]*n. Pass empty to clear.
     */
    setPins(data: Float64Array): void;
    /**
     * Advance one frame. `effectors` is a flat
     * [player, joint, x, y, z, stiffness]*n array. Returns the new pose flat.
     */
    step(effectors: Float64Array, dt: number): Float64Array;
    /**
     * Topology summary (JSON): total writhe of the most entangled chain pairs.
     */
    topologySummary(): string;
    /**
     * Full validation report (JSON) for the current pose.
     */
    validate(): string;
}

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
    readonly memory: WebAssembly.Memory;
    readonly __wbg_engine_free: (a: number, b: number) => void;
    readonly engine_diagnostics: (a: number) => [number, number];
    readonly engine_jointNames: (a: number) => [number, number];
    readonly engine_loadDatabase: (a: number, b: number, c: number) => [number, number, number, number];
    readonly engine_loadEntry: (a: number, b: number) => [number, number];
    readonly engine_loadPose: (a: number, b: number, c: number) => [number, number];
    readonly engine_new: () => number;
    readonly engine_pose: (a: number) => [number, number];
    readonly engine_setConfig: (a: number, b: number, c: number) => [number, number];
    readonly engine_setPins: (a: number, b: number, c: number) => [number, number];
    readonly engine_step: (a: number, b: number, c: number, d: number) => [number, number, number, number];
    readonly engine_topologySummary: (a: number) => [number, number, number, number];
    readonly engine_validate: (a: number) => [number, number, number, number];
    readonly __wbindgen_externrefs: WebAssembly.Table;
    readonly __wbindgen_free: (a: number, b: number, c: number) => void;
    readonly __wbindgen_malloc: (a: number, b: number) => number;
    readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
    readonly __externref_table_dealloc: (a: number) => void;
    readonly __wbindgen_start: () => void;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;

/**
 * Instantiates the given `module`, which can either be bytes or
 * a precompiled `WebAssembly.Module`.
 *
 * @param {{ module: SyncInitInput }} module - Passing `SyncInitInput` directly is deprecated.
 *
 * @returns {InitOutput}
 */
export function initSync(module: { module: SyncInitInput } | SyncInitInput): InitOutput;

/**
 * If `module_or_path` is {RequestInfo} or {URL}, makes a request and
 * for everything else, calls `WebAssembly.instantiate` directly.
 *
 * @param {{ module_or_path: InitInput | Promise<InitInput> }} module_or_path - Passing `InitInput` directly is deprecated.
 *
 * @returns {Promise<InitOutput>}
 */
export default function __wbg_init (module_or_path?: { module_or_path: InitInput | Promise<InitInput> } | InitInput | Promise<InitInput>): Promise<InitOutput>;
