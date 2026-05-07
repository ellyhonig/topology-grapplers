# Knowledge Base Parent: Animation Drivers

Animation drivers are sources of motion-generation logic for GrappleMap. These nodes describe methods that can generate, repair, validate, or classify animated transitions once pose data already exists.

Child nodes:

- `KNOWLEDGE_BASE_TOPOLOGY_COORDINATES.md`: topology-coordinate synthesis for tangled character motion.
- `KNOWLEDGE_BASE_ATTACK_DEFENSE_FSM.md`: topology-coordinate finite state machine for interactive attack/defense wrestling motion.

## Role In GrappleMap

Animation drivers should be treated as derived assistance layers over the canonical `GrappleMap.txt` pose graph. Their first use should be analysis, validation, reports, and suggested intermediate frames rather than automatic writes to the database.

Good animation-driver tasks:

- detect impossible limb threading between keyframes
- preserve entanglement topology while repairing a transition
- classify transitions by attack, defense, escape, or attack-switch topology
- suggest missing intermediate frames for tangled positions
- drive local synthesis demos or editor-assistance tools
