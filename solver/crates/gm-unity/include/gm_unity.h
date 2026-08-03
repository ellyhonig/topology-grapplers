#ifndef GM_UNITY_H
#define GM_UNITY_H

#include <stddef.h>
#include <stdint.h>

#ifdef _WIN32
#define GM_API __declspec(dllimport)
#else
#define GM_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define GM_ABI_VERSION 4u
#define GM_POSE_LENGTH 138u
#define GM_ENTRY_NAME_CAPACITY 256u

typedef struct GmEngine GmEngine;
typedef enum GmResult {
    GM_OK = 0, GM_NULL_POINTER = 1, GM_INVALID_ARGUMENT = 2,
    GM_INVALID_STATE = 3, GM_NOT_FOUND = 4, GM_PARSE_ERROR = 5,
    GM_SERIALIZATION_ERROR = 6, GM_PANIC = 7
} GmResult;

typedef struct GmPlayerJoint { uint32_t player, joint; } GmPlayerJoint;
typedef struct GmEffector {
    uint32_t player, joint;
    double target_x, target_y, target_z, stiffness;
} GmEffector;
typedef struct GmEntryInfo {
    uint32_t frame_count, name_len;
    uint8_t name_utf8[GM_ENTRY_NAME_CAPACITY];
} GmEntryInfo;
typedef struct GmStepDiagnostics {
    double min_clearance, max_bone_error, max_hinge_violation;
    double max_writhe_jump, max_effector_residual;
    uint32_t contact_count, retries, rejected;
} GmStepDiagnostics;
typedef struct GmGripCandidate {
    uint32_t valid, capsule, target_player, end_a, end_b;
    double closest_x, closest_y, closest_z;
    double surface_gap, palm_alignment, wrap_alignment, score;
} GmGripCandidate;
typedef struct GmGripState {
    uint32_t status, capsule, target_player, end_a, end_b, wrap_direction;
    double normalized_strain, normalized_release;
    double normalized_wrap, normalized_contact;
    double normalized_coverage, normalized_strength;
    double selected_writhe, alternate_writhe;
} GmGripState;

GM_API uint32_t gm_abi_version(void);
GM_API GmEngine *gm_engine_create(void);
GM_API void gm_engine_destroy(GmEngine *engine);
GM_API GmResult gm_set_config_json(GmEngine *, const uint8_t *, size_t);
GM_API GmResult gm_load_database(GmEngine *, const uint8_t *, size_t);
GM_API size_t gm_entry_count(const GmEngine *);
GM_API GmResult gm_entry_info(const GmEngine *, size_t, GmEntryInfo *);
GM_API GmResult gm_load_entry(GmEngine *, size_t);
GM_API GmResult gm_load_pose(GmEngine *, const double *, size_t);
GM_API GmResult gm_get_pose(const GmEngine *, double *, size_t);
GM_API GmResult gm_set_pins(GmEngine *, const GmPlayerJoint *, size_t);
GM_API GmResult gm_release_grips(GmEngine *);
GM_API size_t gm_grip_count(const GmEngine *);
GM_API GmResult gm_query_grip_candidate(
    const GmEngine *, uint32_t, uint32_t,
    double, double, double, double, double, double, double, GmGripCandidate *);
GM_API GmResult gm_begin_runtime_grip(GmEngine *, uint32_t, uint32_t, uint32_t, double);
GM_API GmResult gm_end_runtime_grip(GmEngine *, uint32_t, uint32_t);
GM_API GmResult gm_release_runtime_grips(GmEngine *);
GM_API GmResult gm_runtime_grip_state(const GmEngine *, uint32_t, uint32_t, GmGripState *);
GM_API GmResult gm_step(GmEngine *, const GmEffector *, size_t, double,
                        double *, size_t, GmStepDiagnostics *);
GM_API size_t gm_validation_json(GmEngine *, uint8_t *, size_t);
GM_API size_t gm_topology_json(GmEngine *, uint8_t *, size_t);
GM_API size_t gm_last_error(const GmEngine *, uint8_t *, size_t);
GM_API size_t gm_build_id(uint8_t *, size_t);

#ifdef __cplusplus
}
#endif
#endif
