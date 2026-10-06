"""Shared simulation configuration. Every other script imports from here."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MENAGERIE = ROOT / "third_party" / "mujoco_menagerie"
MENAGERIE_COMMIT = "f054586a8e90465d49ee5be15335c4a0c7f57caf"
SCENE_XML = MENAGERIE / "unitree_g1" / "scene.xml"   # G1, 29 actuated DoF, no hands

KEYFRAME = "stand"
CONTROL_HZ = 50            # policy / logging rate
# Physics timestep comes from the model (0.002 s -> 500 Hz); checked in smoke tests.
RESULTS = ROOT / "results"
