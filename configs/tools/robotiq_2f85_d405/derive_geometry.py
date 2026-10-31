"""Derive R2 geometry in its CAD frame, never connect to or configure a robot.

Requires NumPy. Solid CAD centroid/inertia are geometric approximations to a
printed part, not a measured distribution for the specified gyroid infill.
"""
import hashlib
import json
from pathlib import Path
import struct

import numpy as np

ROOT = Path(__file__).resolve().parent


def solid_properties(triangles):
    """Integrate signed tetrahedra (origin, a, b, c); return V, COM, I/m."""
    a, b, c = triangles.transpose(1, 0, 2)
    volumes = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6
    volume = volumes.sum()
    if not np.isfinite(volume) or volume <= 0:
        raise ValueError('Mesh must have positive signed volume')
    sums = a + b + c
    center = (volumes[:, None] * sums / 4).sum(axis=0) / volume
    seconds = np.einsum('ni,nj->nij', sums, sums)
    for vertex in (a, b, c):
        seconds += np.einsum('ni,nj->nij', vertex, vertex)
    covariance = (volumes[:, None, None] * seconds / 20).sum(axis=0) / volume
    covariance -= np.outer(center, center)
    inertia_per_mass = np.trace(covariance) * np.eye(3) - covariance
    return float(volume), center, inertia_per_mass


def derive():
    path = ROOT / 'sources/D405_wrist_mount_R2_PRINT.stl'
    manifest = json.loads((ROOT / 'sources/github-SHA256.json').read_text())
    raw = path.read_bytes()
    if hashlib.sha256(raw).hexdigest() != manifest[path.name]:
        raise ValueError('R2 STL checksum differs from the source release')
    count = struct.unpack_from('<I', raw, 80)[0]
    if len(raw) != 84 + count * 50:
        raise ValueError('Unexpected binary STL size')
    dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    triangles = np.frombuffer(raw, dtype=dtype, count=count, offset=84)['vertices'].astype(float)
    pitch = np.deg2rad(30)
    # Actual source operations: Ry(180deg) @ Rx(pitch - 90deg).
    # The source's descriptive frame comment has a reflected Y axis; use the
    # executable rigid rotation, not that comment. Not an optical calibration.
    rotation = np.array([[-1, 0, 0], [0, np.sin(pitch), np.cos(pitch)],
                         [0, np.cos(pitch), -np.sin(pitch)]])
    front = np.array([0, 70, 64]) * 1e-3
    triangles = (triangles - [0, 0, 28]) * 1e-3 @ rotation.T + front
    volume, center, inertia = solid_properties(triangles)
    checks = json.loads((ROOT / 'sources/mechanical_checks.json').read_text())
    np.testing.assert_allclose(volume, checks['stl']['volume_mm3'] * 1e-9, rtol=1e-9)
    np.testing.assert_allclose([triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))],
                               np.array(checks['world_bounds_mm']) * 1e-3, atol=5e-5)
    np.testing.assert_allclose(rotation @ rotation.T, np.eye(3), atol=1e-12)
    assert np.isclose(np.linalg.det(rotation), 1)
    moments = np.linalg.eigvalsh(inertia)
    assert moments.min() > 0 and 2 * moments.max() <= moments.sum()
    # Camera body modeled only as a uniform nominal bounding box.
    mass = 0.060
    x, y, z = [0.042, 0.042, 0.023]
    camera_inertia = rotation @ (mass / 12 * np.diag([y*y+z*z, x*x+z*z, x*x+y*y])) @ rotation.T
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = front
    result = {
        'frame': 'Robotiq vendor closed CAD frame; not Franka flange',
        'mount': {'volume_m3': volume, 'solid_centroid_m': center.tolist(),
                  'solid_inertia_per_kg_m2_column_major': inertia.flatten(order='F').tolist(),
                  'printed_mass_kg': None,
                  'note': 'Multiply unit-mass tensor by measured/estimated printed mass only as a uniform-density approximation. CAD volume times 40% is not the sliced volume.'},
        'camera_envelope': {'nominal_mass_kg': mass,
                            'geometric_center_m': (front + rotation @ [0, 0, -0.0115]).tolist(),
                            'uniform_box_inertia_kg_m2_column_major': camera_inertia.flatten(order='F').tolist(),
                            'design_T_front_column_major': transform.flatten(order='F').tolist(),
                            'note': 'Geometric center and uniform box inertia are estimates, not vendor mass properties or hand-eye calibration.'},
        'checks': {'source_hash': True, 'volume_matches_release': True,
                   'bounds_match_STEP_report': True, 'proper_rotation': True,
                   'physical_solid_inertia': True},
        'desk_import_generated': False,
    }
    (ROOT / 'geometry-derived.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    derive()
