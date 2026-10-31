# Combined responsiveness trial — 2026-10-31

> 历史试用记录：本文描述当时的候选状态。当前 main 的配置、阈值和数采兼容检查以 [发布检查](RELEASE_20261031.md) 为准。

The operator reported that aperture opening/closing was satisfactory but wrist
rotation still trailed GELLO. `responsive` combines the changes below in one
candidate, without an ablation campaign. It is not yet validated on hardware.

| Setting | Previous `feedforward075_j2_22` | `responsive` |
| --- | --- | --- |
| J1–J4 reference speed | 0.2 rad/s | 0.3 rad/s |
| J5–J7 reference speed | 0.2 rad/s | 0.5 rad/s |
| Reference velocity feedforward | 0.75 | 1.0 |
| J7 tracking position gain | 60 | 90 |
| J1–J6 position gains / all damping gains | Original | Original |
| Home speed / home gains | Original | Original |
| Joint / Cartesian collision thresholds | J2 22 Nm profile | Unchanged |
| Control / dataset sampling | 100 / 15 Hz | 100 / 15 Hz |

These are application reference-speed limits, not manufacturer joint limits or
a guarantee of actual measured speed. The existing torque controller slews its
position reference; its reference is not an acceleration/jerk-limited trajectory.
The feedforward velocity retains its existing acceleration ramp. Holding retains
the active tracking stiffness and cancels the remote target; home retains original
gains, and feedforward is disabled in home, hold and stop.

## Communication and scheduling

The launcher checks both exact USB device identities. It requests sudo only when
the buffers need updating to 1 ms; the setup helper takes the collection lock,
refuses active bridges, saves previous settings under `/var/backups`, verifies
writes and restores earlier values if setup fails. It opens no serial device and
installs no persistent configuration. Replug/reboot may reset the values.

Both values were read back as **1 ms** after applying the helper on 4090.
Previous values were saved to
`/var/backups/robolab-usb-latency-20261031-195037.json`.
The GELLO bus baud rate is unchanged; a 100 Hz target loop does not imply 100 Hz
fresh GELLO measurements.

Only this profile sets BLAS/OpenMP thread counts to 1, confines bridge workers to
CPUs 14–19 and collection workers to 13–19, then pins the Python control thread
to CPU 13 after creating GELLO's reader. The Franka callback remains on CPU 12.
The Python GIL switch interval is 1 ms. These settings reduce potential scheduling
contention; CPU 13 is not reserved from other applications, and improved physical
jitter has not been measured yet.

The session checks actual bridge speed and gain settings before claiming control,
and records them with runtime scheduling settings in diagnostic events.

## Start on 4090

```bash
cd /home/heinrich/robolab-control-system
./scripts/try_latency.sh responsive --execute --teleop-only
```

No cameras or training data; diagnostic logs are retained. Enter starts devices,
then home, then following; Enter stops. The previous mode remains available:

```bash
./scripts/try_latency.sh feedforward075_j2_22 --execute --teleop-only
```

## Verification

- Gain regression failed first with `configured J7 tracking gain not applied`,
  then passed after implementation. Hold continuity, unchanged home gains and
  gain bounds are covered by the same C++ test.
- Real libfranka/Robotiq build succeeded; **2/2 CTest tests passed**.
- **105 Python tests passed** with the new launcher's single-thread BLAS settings.
  An earlier run with inherited BLAS settings had one existing PTY startup test
  time out with an empty transcript; this does not establish its root cause.
- Actual C++ mock bridge + actual session loaded the new profile, followed all
  seven targets, held on stop, used main CPU 13 / 1 ms GIL, and wrote no dataset.
  The first ad-hoc harness omitted the initial hold handshake; the corrected
  harness waits for `hold_ready` before requesting follow.
- Offline actual reference-generator check: for a 30-degree, one-second input,
  J5/J6/J7 reference arrival is **1.048 s**, previously **2.618 s**. This does not
  simulate robot dynamics or measure physical tracking latency.
- Shell syntax, profile inspection and preserved home/protection settings checked.

Evidence: [reference generator](latency/responsive/reference_probe.json),
[mock integration](latency/responsive/mock_integration.json).

No physical motion, Desk recovery, original pipeline edits or merge to main was
performed. Keep this candidate pending operator assessment of free-space tracking
and contact behavior.
