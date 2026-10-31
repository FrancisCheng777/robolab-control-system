"""Build a documented initial Desk profile; never connects to any hardware."""
import hashlib
import json
from pathlib import Path
import tarfile
import xml.etree.ElementTree as ET
import numpy as np

ROOT = Path(__file__).resolve().parent
source = ROOT / "sources/robotiq_2f_85_macro.urdf.xacro"
macro = ET.parse(source).getroot().find("{http://wiki.ros.org/xacro}macro")
# All vendor link/joint rotations are identity at the fully-open q=0 pose.
# Do not evaluate arbitrary xacro expressions. Reject any changed convention.
poses = {"${prefix}robotiq_85_base_link": np.zeros(3)}
for _ in range(10):
    for joint in macro.findall("joint"):
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        origin = joint.find("origin")
        if parent not in poses or origin is None:
            continue
        assert np.allclose(np.fromstring(origin.get("rpy", "0 0 0"), sep=" "), 0)
        poses[child] = poses[parent] + np.fromstring(origin.get("xyz"), sep=" ")
parts = []
for link in macro.findall("link"):
    inertial = link.find("inertial")
    origin = inertial.find("origin")
    assert np.allclose(np.fromstring(origin.get("rpy", "0 0 0"), sep=" "), 0)
    center = poses[link.get("name")] + np.fromstring(origin.get("xyz"), sep=" ")
    mass = float(inertial.find("mass").get("value"))
    v = {k: float(x) for k, x in inertial.find("inertia").attrib.items()}
    inertia = np.array([[v["ixx"], v["ixy"], v["ixz"]],
                        [v["ixy"], v["iyy"], v["iyz"]],
                        [v["ixz"], v["iyz"], v["izz"]]])
    parts.append((mass, center, inertia))
mass = sum(p[0] for p in parts)
center = sum(m * c for m, c, _ in parts) / mass
inertia = np.zeros((3, 3))
for m, c, tensor in parts:
    d = c - center
    inertia += tensor + m * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
eigenvalues = np.linalg.eigvalsh(inertia)
assert len(parts) == 9
assert abs(mass - .921) < .001
assert min(eigenvalues) > 0 and 2 * max(eigenvalues) <= sum(eigenvalues)

# Mass, COM and TCP use the manufacturer manual's original-coupling assembly.
# Its inertia reference point is unspecified; use an explicitly identified
# approximation from the vendor's 2F-85 component model about its aggregate COM.
# Translating that COM does not change a centroidal tensor. This is an initial
# model, not a load identification or a calibration of the installed assembly.
profile = {
    "id": "robotiq-2f85-original-no-camera",
    "name": "Robotiq 2F-85 original coupling (no camera, reference)",
    "color": "#3F51B5",
    "mass": .921,
    "centerOfMass": [0., 0., .060],
    "transformation": [1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., .174, 1.],
    "inertia": inertia.flatten(order="F").tolist(),
}
(ROOT / "endeffector-config.json").write_text(json.dumps(profile, indent=2) + "\n")
provenance = {
    "profile_kind": "initial_reference_not_calibrated",
    "mount": "user-confirmed original Robotiq coupling, standard normal orientation",
    "d405_installed": False,
    "d405_mass_included": False,
    "payload_for_empty_gripper_kg": 0,
    "manual": {
        "url": "https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_General_PDF_20210623.pdf",
        "page": 82,
        "section": "5.2.3",
        "mass_kg": .921, "com_m": [0, 0, .06], "tcp_m": [0, 0, .174],
        "inertia_kg_m2_as_printed": [.00418, .00508, .00125],
        "manual_inertia_used": False,
        "reason": "The manual does not specify the inertia reference point required by Franka."
    },
    "inertia_estimate": {
        "source": "https://github.com/robotiq/robotiq-ros/blob/f95169e1b085254e0a98159ca93bdd1aff7a681f/grippers/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro",
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "pose": "fully open, all gripper joints at zero",
        "method": "sum rotated component COM tensors with the parallel-axis theorem at aggregate COM",
        "component_count": len(parts),
        "component_model_mass_kg": mass,
        "component_model_com_in_urdf_base_m": center.tolist(),
        "inertia_kg_m2_column_major": profile["inertia"],
        "principal_moments_kg_m2": eigenvalues.tolist(),
        "limitation": "Vendor URDF estimate; not a measured inertia for this coupling/fingertip assembly. Manual mass/COM/TCP take precedence. No camera, camera bracket, workpiece or cable force is modeled."
    },
    "desk_import_spec": "https://download.franka.de/ride-cli-doc-v1.1.1/end_effector_commissioning.html",
    "activated_in_desk": False,
    "live_state_verified": False,
    "motion_executed": False,
}
(ROOT / "provenance.json").write_text(json.dumps(provenance, indent=2) + "\n")
archive = ROOT / "robotiq-2f85-original-no-camera.tar.gz"
with tarfile.open(archive, "w:gz") as tar:
    tar.add(ROOT / "endeffector-config.json", arcname="endeffector-config.json")
with tarfile.open(archive) as tar:
    assert tar.getnames() == ["endeffector-config.json"]
    assert json.load(tar.extractfile("endeffector-config.json")) == profile
print(json.dumps({"profile": profile, "inertia_principal_moments": eigenvalues.tolist(), "archive": str(archive)}, indent=2))
