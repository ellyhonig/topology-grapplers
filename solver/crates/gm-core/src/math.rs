//! Minimal 3D vector math. All solver math is f64 for cross-platform determinism
//! (f64 ops are IEEE-754-exact on both native and WASM; no fast-math, no SIMD).

use serde::{Deserialize, Serialize};
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

#[derive(Debug, Clone, Copy, PartialEq, Default, Serialize, Deserialize)]
pub struct V3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub const fn v3(x: f64, y: f64, z: f64) -> V3 {
    V3 { x, y, z }
}

impl V3 {
    pub const ZERO: V3 = v3(0.0, 0.0, 0.0);
    pub const Y: V3 = v3(0.0, 1.0, 0.0);

    pub fn dot(self, o: V3) -> f64 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: V3) -> V3 {
        v3(
            self.y * o.z - self.z * o.y,
            self.z * o.x - self.x * o.z,
            self.x * o.y - self.y * o.x,
        )
    }

    pub fn length_squared(self) -> f64 {
        self.dot(self)
    }

    pub fn length(self) -> f64 {
        self.length_squared().sqrt()
    }

    pub fn distance(self, o: V3) -> f64 {
        (self - o).length()
    }

    pub fn distance_squared(self, o: V3) -> f64 {
        (self - o).length_squared()
    }

    /// Unit vector, or `None` if the length is below `eps`.
    pub fn normalized(self, eps: f64) -> Option<V3> {
        let len = self.length();
        if len < eps {
            None
        } else {
            Some(self / len)
        }
    }

    /// Unit vector, or zero if degenerate.
    pub fn normalized_or_zero(self) -> V3 {
        self.normalized(1e-12).unwrap_or(V3::ZERO)
    }

    /// Clamp the vector to a maximum length.
    pub fn clamped_len(self, max_len: f64) -> V3 {
        let len = self.length();
        if len > max_len && len > 1e-12 {
            self * (max_len / len)
        } else {
            self
        }
    }

    pub fn lerp(self, o: V3, t: f64) -> V3 {
        self + (o - self) * t
    }

    pub fn is_finite(self) -> bool {
        self.x.is_finite() && self.y.is_finite() && self.z.is_finite()
    }

    /// Component perpendicular to a unit axis.
    pub fn perp_to(self, unit_axis: V3) -> V3 {
        self - unit_axis * self.dot(unit_axis)
    }
}

impl Add for V3 {
    type Output = V3;
    fn add(self, o: V3) -> V3 {
        v3(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl Sub for V3 {
    type Output = V3;
    fn sub(self, o: V3) -> V3 {
        v3(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl Mul<f64> for V3 {
    type Output = V3;
    fn mul(self, s: f64) -> V3 {
        v3(self.x * s, self.y * s, self.z * s)
    }
}

impl Div<f64> for V3 {
    type Output = V3;
    fn div(self, s: f64) -> V3 {
        v3(self.x / s, self.y / s, self.z / s)
    }
}

impl Neg for V3 {
    type Output = V3;
    fn neg(self) -> V3 {
        v3(-self.x, -self.y, -self.z)
    }
}

impl AddAssign for V3 {
    fn add_assign(&mut self, o: V3) {
        *self = *self + o;
    }
}

impl SubAssign for V3 {
    fn sub_assign(&mut self, o: V3) {
        *self = *self - o;
    }
}

pub fn clamp(x: f64, lo: f64, hi: f64) -> f64 {
    x.max(lo).min(hi)
}

/// Closest points between segments (p1,q1) and (p2,q2).
/// Returns (point on segment 1, point on segment 2, s, t).
pub fn closest_segment_points(p1: V3, q1: V3, p2: V3, q2: V3) -> (V3, V3, f64, f64) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;
    let a = d1.dot(d1);
    let e = d2.dot(d2);
    let f = d2.dot(r);

    let (s, t);
    if a <= 1e-12 && e <= 1e-12 {
        return (p1, p2, 0.0, 0.0);
    }
    if a <= 1e-12 {
        s = 0.0;
        t = clamp(f / e, 0.0, 1.0);
    } else {
        let c = d1.dot(r);
        if e <= 1e-12 {
            t = 0.0;
            s = clamp(-c / a, 0.0, 1.0);
        } else {
            let b = d1.dot(d2);
            let denom = a * e - b * b;
            let mut s_ = if denom.abs() > 1e-12 {
                clamp((b * f - c * e) / denom, 0.0, 1.0)
            } else {
                0.0
            };
            let mut t_ = (b * s_ + f) / e;
            if t_ < 0.0 {
                t_ = 0.0;
                s_ = clamp(-c / a, 0.0, 1.0);
            } else if t_ > 1.0 {
                t_ = 1.0;
                s_ = clamp((b - c) / a, 0.0, 1.0);
            }
            s = s_;
            t = t_;
        }
    }
    (p1 + d1 * s, p2 + d2 * t, s, t)
}

/// Closest point on segment (a, b) to point `p`. Returns (point, parameter).
pub fn closest_point_on_segment(p: V3, a: V3, b: V3) -> (V3, f64) {
    let d = b - a;
    let len2 = d.dot(d);
    if len2 <= 1e-12 {
        return (a, 0.0);
    }
    let t = clamp((p - a).dot(d) / len2, 0.0, 1.0);
    (a + d * t, t)
}

/// Angle at vertex `b` of triangle (a, b, c), in radians in [0, pi].
pub fn angle_at(a: V3, b: V3, c: V3) -> f64 {
    let u = a - b;
    let w = c - b;
    let lu = u.length();
    let lw = w.length();
    if lu < 1e-12 || lw < 1e-12 {
        return 0.0;
    }
    clamp(u.dot(w) / (lu * lw), -1.0, 1.0).acos()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn segment_closest_points_parallel_and_crossing() {
        let (c1, c2, _, _) =
            closest_segment_points(v3(0.0, 0.0, 0.0), v3(1.0, 0.0, 0.0), v3(0.5, 1.0, 0.0), v3(0.5, 2.0, 0.0));
        assert!((c1 - v3(0.5, 0.0, 0.0)).length() < 1e-12);
        assert!((c2 - v3(0.5, 1.0, 0.0)).length() < 1e-12);
    }

    #[test]
    fn angle_at_right_angle() {
        let a = v3(1.0, 0.0, 0.0);
        let b = V3::ZERO;
        let c = v3(0.0, 1.0, 0.0);
        assert!((angle_at(a, b, c) - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
    }
}
