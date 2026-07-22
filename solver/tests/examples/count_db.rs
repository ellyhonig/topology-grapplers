fn main() {
    let text = std::fs::read_to_string("../GrappleMap.txt").unwrap();
    let entries = gm_core::parse_database(&text).unwrap();
    let positions = entries.iter().filter(|e| e.is_position()).count();
    let frames: usize = entries.iter().map(|e| e.frames.len()).sum();
    println!("entries={} positions={} transitions={} frames={}", entries.len(), positions, entries.len()-positions, frames);
}
