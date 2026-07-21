//! Muscle tone: per-player shape matching (Mueller et al. 2005).
//!
//! Each body remembers a rest *shape* (joint offsets from the mass-weighted
//! centroid). Every substep the best-fit rigid transform from that shape to the
//! current pose is computed, and every joint is pulled compliantly toward its
//! rigidly-transformed goal. Because only the *shape* is matched - never a world
//! orientation or position - tone resists crumpling but offers zero resistance
//! to whole-body translation or rotation: an unsupported body falls and topples
//! like a stiff ragdoll, while a supported one stands.
//!
//! The rest shape is *plastic*: deviations beyond a dead zone are slowly
//! absorbed, so sustained input (an arm dragged to extension, a body settled on
//! the ground) becomes the new held pose instead of springing back, while
//! gravity-level sag inside the dead zone never melts the pose.

use gm_core::{Joint, PlayerId, PlayerJoint, Pose, V3, JOINT_COUNT, PLAYER_COUNT};

pub type RestShape = [[V3; JOINT_COUNT]; PLAYER_COUNT];

type Mat3 = [[f64; 3]; 3];

const IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn mul_vec(m: &Mat3, v: V3) -> V3 {
    gm_core::v3(
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z,
    )
}

fn mul_vec_t(m: &Mat3, v: V3) -> V3 {
    gm_core::v3(
        m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z,
        m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z,
        m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z,
    )
}

fn det(m: &Mat3) -> f64 {
    m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
}

fn inverse(m: &Mat3) -> Option<Mat3> {
    let d = det(m);
    if d.abs() < 1e-12 {
        return None;
    }
    let inv_d = 1.0 / d;
    let mut r = IDENTITY;
    r[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_d;
    r[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_d;
    r[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_d;
    r[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_d;
    r[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_d;
    r[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_d;
    r[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_d;
    r[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_d;
    r[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_d;
    Some(r)
}

fn norm_f(m: &Mat3) -> f64 {
    m.iter().flatten().map(|x| x * x).sum::<f64>().sqrt()
}

/// Orthogonal polar factor of `a` via the scaled Newton iteration
/// X <- (g X + (X^-1)^T / g) / 2. Deterministic, quadratically convergent.
/// Returns None for (near-)singular or reflecting inputs, which bone-length
/// constraints prevent in practice.
fn polar_rotation(a: &Mat3) -> Option<Mat3> {
    if det(a) < 1e-9 {
        return None;
    }
    let mut x = *a;
    for _ in 0..24 {
        let xit = {
            let xi = inverse(&x)?;
            [
                [xi[0][0], xi[1][0], xi[2][0]],
                [xi[0][1], xi[1][1], xi[2][1]],
                [xi[0][2], xi[1][2], xi[2][2]],
            ]
        };
        let g = (norm_f(&xit) / norm_f(&x)).sqrt();
        let mut next = IDENTITY;
        let mut diff = 0.0f64;
        for r in 0..3 {
            for c in 0..3 {
                next[r][c] = 0.5 * (g * x[r][c] + xit[r][c] / g);
                diff = diff.max((next[r][c] - x[r][c]).abs());
            }
        }
        x = next;
        if diff < 1e-12 {
            break;
        }
    }
    Some(x)
}

/// Mass-weighted centroid of one player's joints.
pub fn centroid(pose: &Pose, player: PlayerId) -> V3 {
    let mut c = V3::ZERO;
    let mut total = 0.0;
    for j in Joint::ALL {
        let m = j.mass();
        c += pose.get(player, j) * m;
        total += m;
    }
    c / total
}

/// Capture the rest shape (centroid-centered joint offsets) from a pose.
pub fn capture_rest_shape(pose: &Pose) -> RestShape {
    let mut rest = [[V3::ZERO; JOINT_COUNT]; PLAYER_COUNT];
    for player in PlayerId::ALL {
        let c = centroid(pose, player);
        for j in Joint::ALL {
            rest[player.index()][j.index()] = pose.get(player, j) - c;
        }
    }
    rest
}

/// Weighted centroids of the current pose and of the rest shape, using the
/// same weights as the rotation fit so the whole fit is self-consistent.
fn weighted_centroids(
    pose: &Pose,
    player: PlayerId,
    rest: &RestShape,
    weight: &dyn Fn(Joint) -> f64,
) -> Option<(V3, V3)> {
    let mut cp = V3::ZERO;
    let mut cq = V3::ZERO;
    let mut total = 0.0;
    for j in Joint::ALL {
        let w = weight(j);
        cp += pose.get(player, j) * w;
        cq += rest[player.index()][j.index()] * w;
        total += w;
    }
    if total <= 1e-9 {
        return None;
    }
    Some((cp / total, cq / total))
}

/// Best-fit rotation taking the (re-centered) rest shape onto the current
/// pose: polar factor of the weighted covariance sum w_i (p_i - cp)(q_i - cq)^T.
fn best_rotation(
    pose: &Pose,
    player: PlayerId,
    rest: &RestShape,
    cp: V3,
    cq: V3,
    weight: &dyn Fn(Joint) -> f64,
) -> Option<Mat3> {
    let mut a = [[0.0f64; 3]; 3];
    for j in Joint::ALL {
        let w = weight(j);
        if w <= 0.0 {
            continue;
        }
        let p = (pose.get(player, j) - cp) * w;
        let q = rest[player.index()][j.index()] - cq;
        let pv = [p.x, p.y, p.z];
        let qv = [q.x, q.y, q.z];
        for r in 0..3 {
            for col in 0..3 {
                a[r][col] += pv[r] * qv[col];
            }
        }
    }
    polar_rotation(&a)
}

/// Compliantly pull every joint toward its shape-matching goal.
/// `scale` attenuates tone per joint (0 = fully relaxed): joints under active
/// effector input relax so input is never fought to a standstill.
///
/// Two properties keep tone from towing the body after a held limb:
/// - the rigid fit (centroids + rotation) is weighted by mass * scale, so a
///   relaxed, input-driven limb cannot bias the goal shape of the rest of the
///   body toward itself;
/// - the mass-weighted net pull is subtracted (muscles are internal forces and
///   cannot move the center of mass). Support against gravity still works:
///   tone pushes the feet down as much as it lifts the torso, and the floor
///   supplies the actual upward reaction.
pub fn project_muscle_tone(
    pose: &mut Pose,
    rest: &RestShape,
    stiffness: f64,
    scale: &[[f64; JOINT_COUNT]; PLAYER_COUNT],
    inv_mass: &dyn Fn(PlayerJoint) -> f64,
) {
    for player in PlayerId::ALL {
        let weight =
            |j: Joint| j.mass() * scale[player.index()][j.index()];
        let Some((cp, cq)) = weighted_centroids(pose, player, rest, &weight) else {
            continue;
        };
        let Some(rot) = best_rotation(pose, player, rest, cp, cq, &weight) else {
            continue;
        };
        let mut pulls = [V3::ZERO; JOINT_COUNT];
        let mut net = V3::ZERO;
        let mut total_mass = 0.0;
        for j in Joint::ALL {
            let pj = PlayerJoint { player, joint: j };
            if inv_mass(pj) <= 0.0 {
                continue;
            }
            let goal = cp + mul_vec(&rot, rest[player.index()][j.index()] - cq);
            let pull = (goal - pose[pj]) * (stiffness * scale[player.index()][j.index()]);
            pulls[j.index()] = pull;
            let m = j.mass();
            net += pull * m;
            total_mass += m;
        }
        if total_mass <= 0.0 {
            continue;
        }
        let mean = net / total_mass;
        for j in Joint::ALL {
            let pj = PlayerJoint { player, joint: j };
            if inv_mass(pj) <= 0.0 {
                continue;
            }
            pose[pj] += pulls[j.index()] - mean;
        }
    }
}

/// Plastically absorb deviations beyond the dead zone into the rest shape,
/// then re-center it (absorption shifts the centroid slightly).
pub fn adapt_rest_shape(
    pose: &Pose,
    rest: &mut RestShape,
    plasticity: f64,
    deadzone: f64,
    dt: f64,
) {
    let rate = (plasticity * dt).min(1.0);
    if rate <= 0.0 {
        return;
    }
    for player in PlayerId::ALL {
        let mass = |j: Joint| j.mass();
        let c = centroid(pose, player);
        let Some(rot) = best_rotation(pose, player, &*rest, c, V3::ZERO, &mass) else {
            continue;
        };
        let mut total = 0.0;
        let mut shift = V3::ZERO;
        for j in Joint::ALL {
            let qi = &mut rest[player.index()][j.index()];
            let local = mul_vec_t(&rot, pose.get(player, j) - c);
            let dev = local - *qi;
            let len = dev.length();
            if len > deadzone {
                *qi += dev * ((len - deadzone) / len * rate);
            }
            let m = j.mass();
            shift += *qi * m;
            total += m;
        }
        // Re-center so the rest shape's mass-weighted centroid stays at origin.
        let shift = shift / total;
        for j in Joint::ALL {
            rest[player.index()][j.index()] -= shift;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use gm_core::v3;

    #[test]
    fn polar_of_rotation_is_itself() {
        // 90 degrees about y.
        let r: Mat3 = [[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]];
        // Scale it (polar must strip the stretch).
        let mut a = r;
        for row in a.iter_mut() {
            for x in row.iter_mut() {
                *x *= 2.5;
            }
        }
        let p = polar_rotation(&a).unwrap();
        for i in 0..3 {
            for j in 0..3 {
                assert!((p[i][j] - r[i][j]).abs() < 1e-9, "entry {} {}", i, j);
            }
        }
    }

    #[test]
    fn mul_vec_t_is_transpose() {
        let m: Mat3 = [[1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [7.0, 8.0, 9.0]];
        let v = v3(1.0, -2.0, 0.5);
        let a = mul_vec_t(&m, v);
        let mt: Mat3 = [[1.0, 4.0, 7.0], [2.0, 5.0, 8.0], [3.0, 6.0, 9.0]];
        let b = mul_vec(&mt, v);
        assert!((a - b).length() < 1e-12);
    }
}
