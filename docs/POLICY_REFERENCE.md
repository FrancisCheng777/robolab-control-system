# Measured-motion policy reference adapter

`pi05_demo.policy_reference` is an opt-in adapter for model actions encoded as
`(q[t+1] - q[t]) / 0.05` on a 15Hz measured-state grid. It is not for GELLO's
absolute leader targets and does not modify collection, torque gains or limits.

A torque servo may not realize a tiny displacement within one policy tick.
Repeatedly commanding `measured_q + delta` discards the untracked displacement
and can stall before grasping. The adapter advances a retained reference once
per new model action. It bounds reference lead by the smaller of the supplied
tracking limits and an explicit cap no larger than 0.05rad. Saturated excess is
discarded, not carried as hidden windup. The caller checks joint limits and
resets the reference to None after hold/home or an episode boundary. Heartbeat
refreshes use `bound_joint_reference` without adding another delta.

The π0.5 runtime owns its 15Hz scheduling, 100Hz refresh, hold and watchdog. The
shared adapter itself performs no I/O. Legacy collection remains on absolute
leader-target control. Unit tests cover stalled feedback, reversal, reset,
perfect tracking, invalid input and smaller calibrated bounds; physical success
still needs task-specific verification. The related π0.5 test reached grasp,
placement and insertion attempt after enabling this adapter, as reported by the
operator; successful insertion has not been established.
