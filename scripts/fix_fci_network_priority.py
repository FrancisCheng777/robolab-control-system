#!/usr/bin/env python3
"""Temporary, reversible network softirq priority experiment for the FCI NIC.

No robot commands, affinity changes, driver changes, or persistence across reboot.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import pwd
import stat
import time

NIC = 'enp5s0'


def single_cpu(text):
    text = text.strip()
    if not text.isdigit():
        raise RuntimeError('Expected exactly one IRQ CPU; refusing to guess: ' + text)
    return int(text)


def starttime(pid):
    return (Path('/proc') / str(pid) / 'stat').read_text().rsplit(')', 1)[1].split()[19]


def discover():
    entries = list((Path('/sys/class/net') / NIC / 'device/msi_irqs').iterdir())
    if len(entries) != 1:
        raise RuntimeError('Expected this NIC to have exactly one MSI interrupt')
    irq = int(entries[0].name)
    cpu = single_cpu((Path('/proc/irq') / str(irq) / 'effective_affinity_list').read_text())
    name = 'ksoftirqd/' + str(cpu)
    found = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if (proc / 'comm').read_text().strip() == name:
                found.append(int(proc.name))
        except (FileNotFoundError, ProcessLookupError):
            pass
    if len(found) != 1:
        raise RuntimeError('Expected a unique kernel network softirq thread')
    pid = found[0]
    return dict(nic=NIC, irq=irq, cpu=cpu, pid=pid, comm=name, start=starttime(pid),
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                policy=os.sched_getscheduler(pid), priority=os.sched_getparam(pid).sched_priority)


def apply_priority(state):
    try:
        os.sched_setscheduler(state['pid'], os.SCHED_FIFO, os.sched_param(50))
        actual = discover()
        if actual['pid'] != state['pid'] or actual['policy'] != os.SCHED_FIFO or actual['priority'] != 50:
            raise RuntimeError('IRQ/thread verification failed')
    except BaseException:
        os.sched_setscheduler(state['pid'], state['policy'], os.sched_param(state['priority']))
        raise


def collection_lock(path, uid, gid):
    """Create the coordination lock on first use, without trusting symlinks."""
    try:
        fd = os.open(path, os.O_RDWR | os.O_NOFOLLOW | os.O_CREAT | os.O_EXCL, 0o660)
    except FileExistsError:
        fd = os.open(path, os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK)
    else:
        try:
            if os.geteuid() == 0:
                os.fchown(fd, uid, gid)
        except BaseException:
            os.close(fd)
            raise
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != uid:
            raise RuntimeError('Unexpected collection lock owner/type')
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('Collection/control is active; exit it first') from exc
    except BaseException:
        os.close(fd)
        raise
    return fd


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--apply', action='store_true')
    modes.add_argument('--rollback', type=Path)
    args = parser.parse_args()
    if args.apply or args.rollback:
        if os.geteuid() != 0:
            raise SystemExit('sudo is required to change a kernel thread priority')
        user = pwd.getpwnam('heinrich')
        fd = collection_lock('/tmp/pi05-robolab-trial-20261030.lock', user.pw_uid, user.pw_gid)
    if args.rollback:
        folder = args.rollback.resolve()
        if folder.parent != Path('/var/backups') or folder.stat().st_uid != 0:
            raise SystemExit('Expected root-owned backup under /var/backups')
        saved = json.loads((folder / 'network_priority.json').read_text())
        if saved['boot_id'] != Path('/proc/sys/kernel/random/boot_id').read_text().strip():
            raise SystemExit('Host rebooted; this temporary priority setting has already expired')
        if starttime(saved['pid']) != saved['start']:
            raise SystemExit('Thread identity changed; refusing to modify another process')
        if os.sched_getscheduler(saved['pid']) != os.SCHED_FIFO or os.sched_getparam(saved['pid']).sched_priority != 50:
            raise SystemExit('Priority changed since experiment; inspect before rollback')
        os.sched_setscheduler(saved['pid'], saved['policy'], os.sched_param(saved['priority']))
        print('Restored prior kernel thread scheduling policy:', saved)
        return
    state = discover()
    print(json.dumps(state, indent=2))
    if not args.apply:
        print('Dry run: proposed SCHED_FIFO 50 for this one network softirq thread only.')
        return
    if state['policy'] == os.SCHED_FIFO and state['priority'] == 50:
        print('Network FIFO 50 is already applied; no changes needed.')
        os.close(fd)
        return
    if state['policy'] != os.SCHED_OTHER or state['priority'] != 0:
        raise SystemExit('Thread is not ordinary SCHED_OTHER; inspect instead of overwriting')
    folder = Path('/var/backups') / ('pi05-network-priority-' + time.strftime('%Y%m%d-%H%M%S'))
    folder.mkdir(mode=0o755)
    (folder / 'network_priority.json').write_text(json.dumps(state, indent=2) + '\n')
    apply_priority(state)
    print('Applied temporary FIFO 50; backup / rollback:', folder)
    print('No persistent configuration installed. Keep collection stopped for validation.')


if __name__ == '__main__':
    main()
