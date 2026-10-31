"""Build the nominal stock 2F-85 + D405/R2 Desk reference; no hardware access."""
import hashlib
import json
from pathlib import Path
import tarfile

import numpy as np

ROOT = Path(__file__).resolve().parent


def combine(parts):
    mass = sum(p['mass_kg'] for p in parts)
    center = sum(p['mass_kg'] * np.array(p['center_in_flange_m']) for p in parts) / mass
    inertia = np.zeros((3, 3))
    for part in parts:
        d = np.array(part['center_in_flange_m']) - center
        tensor = np.array(part['inertia_at_com_in_flange_kg_m2']).reshape(3, 3)
        inertia += tensor + part['mass_kg'] * (d @ d * np.eye(3) - np.outer(d, d))
    return mass, center, inertia


def build():
    inputs = json.loads((ROOT / 'assembly-inputs.json').read_text())
    geometry = json.loads((ROOT / 'geometry-derived.json').read_text())
    baseline_path = ROOT / inputs['baseline_profile']
    baseline = json.loads(baseline_path.read_text())
    assert inputs['original_coupling_and_fingertips_unchanged'] is True
    assert inputs['mount']['matches_installed_part'] is True
    assert inputs['finger_assembly']['yellow_TPU_installed'] is False
    for source in inputs['mount']['sources'].values():
        assert hashlib.sha256((ROOT / source['path']).read_bytes()).hexdigest() == source['sha256']

    # Keep the existing standard, unflipped, identity-TCP mounting convention.
    # CAD X: jaw opening; CAD Y: tips; CAD +Z: photographed camera side.
    # Nominal mapping: (x,y,z)_CAD -> (x,-z,y+11.2mm)_flange.
    # 11.2mm is inferred from manual nominal 174mm coupled TCP minus
    # 162.8mm closed bare-gripper height, not measured from the photograph.
    rotation = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
    offset = np.array([0., 0., 0.0112])
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(rotation), 1.)
    parts = [{
        'name': 'stock_2f85_including_original_coupling_and_fingers',
        'mass_kg': baseline['mass'],
        'center_in_flange_m': baseline['centerOfMass'],
        'inertia_at_com_in_flange_kg_m2': np.array(baseline['inertia']).reshape(3, 3, order='F').tolist(),
        'basis': 'Previous reference profile: manual mass/COM/TCP, vendor URDF open-gripper inertia estimate.',
    }]

    def add_part(name, mass, center, tensor, basis):
        parts.append({'name': name, 'mass_kg': mass,
                      'center_in_flange_m': (rotation @ np.array(center) + offset).tolist(),
                      'inertia_at_com_in_flange_kg_m2': (rotation @ tensor @ rotation.T).tolist(),
                      'basis': basis})

    mount = geometry['mount']
    mount_mass = inputs['mount']['printed_part_mass_kg']
    add_part('PETG_R2_mount', mount_mass, mount['solid_centroid_m'],
             np.array(mount['solid_inertia_per_kg_m2_column_major']).reshape(3, 3, order='F') * mount_mass,
             'Full CAD solid at PETG reference density 1270 kg/m3; actual infill distribution not modeled.')
    camera = geometry['camera_envelope']
    add_part('D405', camera['nominal_mass_kg'], camera['geometric_center_m'],
             np.array(camera['uniform_box_inertia_kg_m2_column_major']).reshape(3, 3, order='F'),
             '60g nominal, 42x42x23mm uniform box at its geometric center; not calibrated mass properties.')
    cad_T_camera = np.array(camera['design_T_front_column_major']).reshape(4, 4, order='F')
    for x in [-0.010, 0.010]:
        center = cad_T_camera[:3, :3] @ [x, 0., -0.0255] + cad_T_camera[:3, 3]
        add_part(f'camera_screw_and_washer_at_x_{x}', inputs['mount']['added_fasteners_mass_kg'] / 2,
                 center, np.zeros((3, 3)),
                 '1g point-mass allowance at the rear plate screw axis. Existing gripper screws not counted twice.')

    mass, center, inertia = combine(parts)
    center[np.abs(center) < 1e-9] = 0.0  # Remove STL rounding noise on the symmetry plane.
    moments = np.linalg.eigvalsh(inertia)
    assert mass > 0 and np.isfinite(inertia).all() and np.isfinite(center).all()
    np.testing.assert_allclose(inertia, inertia.T, atol=1e-12)
    assert moments.min() > 0 and 2 * moments.max() <= moments.sum()
    profile = {
        'id': 'robotiq-2f85-d405-r2-reference',
        'name': 'Robotiq 2F-85 + D405 + R2 PETG (estimated reference)',
        'color': '#3F51B5', 'mass': mass, 'centerOfMass': center.tolist(),
        'transformation': baseline['transformation'],
        'inertia': inertia.flatten(order='F').tolist(),
    }
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = offset
    report = {
        'profile_kind': 'nominal_engineering_reference_not_load_identified',
        'baseline_sha256': hashlib.sha256(baseline_path.read_bytes()).hexdigest(),
        'mount_repository_commit': inputs['mount']['commit'],
        'flange_T_design_column_major': transform.flatten(order='F').tolist(),
        'registration_basis': 'Prior standard unflipped mounting convention; stock fingers and R2 side mount corroborated by user photographs. Rx(+90deg) CAD-to-flange nominal mapping. Effective axial offset 11.2mm inferred from manual pages 65/82, not image metrology.',
        'registration_is_measured': False,
        'camera_is_hand_eye_calibrated': False,
        'components': parts,
        'total_mass_kg': mass,
        'principal_moments_kg_m2': moments.tolist(),
        'TCP_unchanged': True,
        'payload_for_empty_gripper_kg': 0,
        'activated_in_desk': False,
        'live_state_verified': False,
        'motion_executed': False,
        'limitations': [
            'PETG solid-density reference overestimates a partly hollow print; exact brand/density and sliced distribution are unknown.',
            'Camera inertia/COM and fasteners are geometric estimates. USB connector/cable mass and forces are not separately modeled.',
            'Flange registration inherits the old nominal mounting convention; photos corroborate assembly, not calibrated axes or millimeter distances.',
            'Original gripper inertia is the prior vendor-URDF fully-open estimate; finger motion changes the real inertia.',
            'This is an inertial/TCP configuration, not a camera collision envelope or optical hand-eye calibration.',
        ],
    }
    for filename, value in [('endeffector-config.json', profile), ('provenance.json', report)]:
        (ROOT / filename).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    archive = ROOT / 'robotiq-2f85-d405-r2-reference.tar.gz'
    with tarfile.open(archive, 'w:gz') as tar:
        tar.add(ROOT / 'endeffector-config.json', arcname='endeffector-config.json')
    with tarfile.open(archive) as tar:
        assert tar.getnames() == ['endeffector-config.json']
        assert json.load(tar.extractfile('endeffector-config.json')) == profile
    inputs['status'] = 'reference_profile_generated_not_applied_in_desk'
    inputs['mount']['flange_T_design_column_major'] = transform.flatten(order='F').tolist()
    inputs['mount']['flange_registration_basis'] = report['registration_basis']
    inputs['combined_desk_parameters'] = {k: profile[k] for k in ['mass', 'centerOfMass', 'inertia', 'transformation']}
    inputs['pending'] = ['Desk activation and live readback; physical commissioning/calibration are not implied by generation']
    (ROOT / 'assembly-inputs.json').write_text(json.dumps(inputs, indent=2, ensure_ascii=False) + '\n')
    print(json.dumps(profile, indent=2))


if __name__ == '__main__':
    build()
