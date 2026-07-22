//! GrappleMap.txt codec: base62 position encoding and database parsing.
//! Byte-compatible with `topology-grapplers/src/persistence.cpp`.

use crate::body::JOINT_COUNT;
use crate::math::{v3, V3};
use crate::pose::{Pose, PLAYER_COUNT};

const BASE62: &[u8; 62] = b"abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";

/// 2 players * 23 joints * 3 coords * 2 digits = 276 digits per pose.
pub const POSE_DIGITS: usize = PLAYER_COUNT * JOINT_COUNT * 3 * 2;
const LINE_DIGITS: usize = POSE_DIGITS / 4;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CodecError {
    pub message: String,
}

impl std::fmt::Display for CodecError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "codec error: {}", self.message)
    }
}

impl std::error::Error for CodecError {}

fn err<T>(message: impl Into<String>) -> Result<T, CodecError> {
    Err(CodecError { message: message.into() })
}

fn from_base62(c: u8) -> Result<u32, CodecError> {
    match c {
        b'a'..=b'z' => Ok((c - b'a') as u32),
        b'A'..=b'Z' => Ok((c - b'A') as u32 + 26),
        b'0'..=b'9' => Ok((c - b'0') as u32 + 52),
        _ => err(format!("not a base62 digit: {:?}", c as char)),
    }
}

/// Decode a pose from a 276-digit base62 string (whitespace ignored).
pub fn decode_pose(text: &str) -> Result<Pose, CodecError> {
    let digits: Vec<u8> = text.bytes().filter(|b| !b.is_ascii_whitespace()).collect();
    if digits.len() != POSE_DIGITS {
        return err(format!("expected {} digits, got {}", POSE_DIGITS, digits.len()));
    }
    let mut vals = [0.0f64; PLAYER_COUNT * JOINT_COUNT * 3];
    for (i, out) in vals.iter_mut().enumerate() {
        let hi = from_base62(digits[i * 2])?;
        let lo = from_base62(digits[i * 2 + 1])?;
        *out = f64::from(hi * 62 + lo) / 1000.0;
    }
    let mut pose = Pose::default();
    let mut k = 0;
    for player in 0..PLAYER_COUNT {
        for joint in 0..JOINT_COUNT {
            pose.joints[player][joint] =
                v3(vals[k] - 2.0, vals[k + 1], vals[k + 2] - 2.0);
            k += 3;
        }
    }
    Ok(pose)
}

fn encode_coord(out: &mut String, d: f64) -> Result<(), CodecError> {
    let i = (d * 1000.0).round() as i64;
    if !(0..4000).contains(&i) {
        return err(format!("coordinate out of range: {}", d));
    }
    out.push(BASE62[(i / 62) as usize] as char);
    out.push(BASE62[(i % 62) as usize] as char);
    Ok(())
}

/// Encode a pose as the four indented database lines (with trailing newline each).
pub fn encode_pose(pose: &Pose) -> Result<String, CodecError> {
    let mut digits = String::with_capacity(POSE_DIGITS);
    for player in 0..PLAYER_COUNT {
        for joint in 0..JOINT_COUNT {
            let p: V3 = pose.joints[player][joint];
            encode_coord(&mut digits, p.x + 2.0)?;
            encode_coord(&mut digits, p.y)?;
            encode_coord(&mut digits, p.z + 2.0)?;
        }
    }
    let mut out = String::with_capacity(POSE_DIGITS + 4 * 5);
    for i in 0..4 {
        out.push_str("    ");
        out.push_str(&digits[i * LINE_DIGITS..(i + 1) * LINE_DIGITS]);
        out.push('\n');
    }
    Ok(out)
}

/// One database record: a named position (1 frame) or transition (2+ frames).
#[derive(Debug, Clone)]
pub struct DbEntry {
    pub description: Vec<String>,
    pub frames: Vec<Pose>,
    pub line_nr: usize,
}

impl DbEntry {
    pub fn is_position(&self) -> bool {
        self.frames.len() == 1
    }

    pub fn name(&self) -> &str {
        self.description.first().map(String::as_str).unwrap_or("?")
    }

    pub fn properties(&self) -> Vec<&str> {
        self.description
            .iter()
            .filter_map(|l| l.strip_prefix("properties:"))
            .flat_map(|l| l.split_whitespace())
            .collect()
    }

    pub fn tags(&self) -> Vec<&str> {
        self.description
            .iter()
            .filter_map(|l| l.strip_prefix("tags:"))
            .flat_map(|l| l.split_whitespace())
            .collect()
    }
}

/// Parse the full GrappleMap.txt database.
pub fn parse_database(text: &str) -> Result<Vec<DbEntry>, CodecError> {
    let mut entries: Vec<DbEntry> = Vec::new();
    let mut desc: Vec<String> = Vec::new();
    let mut pose_lines: Vec<&str> = Vec::new();
    let mut last_was_position = false;

    for (line_idx, line) in text.lines().enumerate() {
        let is_position = line.starts_with(' ');
        if is_position {
            if !last_was_position {
                if desc.is_empty() {
                    return err(format!("line {}: position block without description", line_idx + 1));
                }
                let description = std::mem::take(&mut desc);
                let line_nr = line_idx + 1 - description.len();
                entries.push(DbEntry { description, frames: Vec::new(), line_nr });
            }
            pose_lines.push(line);
            if pose_lines.len() == 4 {
                let joined = pose_lines.join("");
                let entry = entries.last_mut().unwrap();
                entry.frames.push(decode_pose(&joined).map_err(|e| CodecError {
                    message: format!("line {}: {}", line_idx + 1, e.message),
                })?);
                pose_lines.clear();
            }
        } else {
            if !pose_lines.is_empty() {
                return err(format!("line {}: truncated position block", line_idx + 1));
            }
            if last_was_position {
                desc.clear();
            }
            desc.push(line.to_string());
        }
        last_was_position = is_position;
    }
    if !pose_lines.is_empty() {
        return err("database ends with truncated position block");
    }
    Ok(entries)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pose::PlayerJoint;

    fn sample_pose() -> Pose {
        let mut pose = Pose::default();
        for (i, pj) in PlayerJoint::all().enumerate() {
            let f = i as f64;
            pose[pj] = v3(
                -1.9 + f * 0.05,
                0.001 * f + 0.02,
                1.8 - f * 0.06,
            );
        }
        pose
    }

    #[test]
    fn pose_round_trip_is_exact_at_mm_resolution() {
        let pose = sample_pose();
        let encoded = encode_pose(&pose).unwrap();
        let decoded = decode_pose(&encoded).unwrap();
        for pj in PlayerJoint::all() {
            assert!(
                (decoded[pj] - pose[pj]).length() < 1.5e-3,
                "joint {:?} moved {} m",
                pj,
                (decoded[pj] - pose[pj]).length()
            );
        }
        // A second round trip must be bit-exact (idempotent quantization).
        let encoded2 = encode_pose(&decoded).unwrap();
        assert_eq!(encoded, encoded2);
    }

    #[test]
    fn parse_minimal_database() {
        let pose = sample_pose();
        let block = encode_pose(&pose).unwrap();
        let db = format!(
            "some position\ntags: guard\n{}transition to elsewhere\nproperties: top\n{}{}",
            block, block, block
        );
        let entries = parse_database(&db).unwrap();
        assert_eq!(entries.len(), 2);
        assert!(entries[0].is_position());
        assert_eq!(entries[0].name(), "some position");
        assert_eq!(entries[0].tags(), vec!["guard"]);
        assert_eq!(entries[1].frames.len(), 2);
        assert_eq!(entries[1].properties(), vec!["top"]);
    }
}
