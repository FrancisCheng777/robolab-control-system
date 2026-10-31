#!/usr/bin/env python3
"""Repair the observed PAM priority=99 typo; never connect to robot hardware."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import pwd
import re
import stat
import tempfile
import time

LIMITS = Path('/etc/security/limits.conf')
LOCK = Path('/tmp/pi05-robolab-trial-20261030.lock')


def corrected_limits(text):
    pattern = r'(?m)^(@realtime\s+(?:soft|hard|-)\s+priority\s+)99(\s*(?:#[^\n]*)?)$'
    result, count = re.subn(pattern, r'\g<1>0\2', text)
    if count == 0:
        raise RuntimeError('Expected @realtime priority=99 entries not found; inspect instead of overwriting')
    return result


def replace_file(path, text):
    info = path.stat()
    fd, name = tempfile.mkstemp(prefix='.pi05-priority-', dir=path.parent)
    try:
        with os.fdopen(fd, 'w') as out:
            out.write(text)
            out.flush()
            os.fsync(out.fileno())
        os.chmod(name, stat.S_IMODE(info.st_mode))
        os.chown(name, info.st_uid, info.st_gid)
        os.replace(name, path)
    finally:
        if os.path.exists(name):
            os.unlink(name)


def process_info(pid):
    path = Path('/proc') / str(pid)
    fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
    return {'pid': pid, 'ppid': int(fields[1]), 'start': fields[19],
            'uid': path.stat().st_uid, 'nice': os.getpriority(os.PRIO_PROCESS, pid)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--apply', action='store_true')
    mode.add_argument('--rollback', type=Path)
    args = parser.parse_args()
    uid = pwd.getpwnam('heinrich').pw_uid
    if args.apply or args.rollback:
        if os.geteuid() != 0:
            raise SystemExit('Run with sudo; no password is read or stored by this script.')
        # Same exclusive lock as collection; never alter priorities during a run.
        fd = os.open(LOCK, os.O_RDWR | os.O_NOFOLLOW)
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != uid:
            raise SystemExit('Unexpected collection lock owner/type')
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit('Collection is active. Exit with q before applying this repair.')
    if args.rollback:
        folder = args.rollback.resolve()
        if folder.parent != Path('/var/backups') or folder.stat().st_uid != 0:
            raise SystemExit('Rollback requires this tool’s root-owned /var/backups directory')
        record = json.loads((folder / 'record.json').read_text())
        current = LIMITS.read_text()
        if hashlib.sha256(current.encode()).hexdigest() != record['after_sha256']:
            raise SystemExit('limits.conf changed since repair; refusing to overwrite later edits')
        replace_file(LIMITS, (folder / 'limits.conf.before').read_text())
        for saved in record['processes']:
            try:
                actual = process_info(saved['pid'])
                if actual['start'] == saved['start'] and actual['uid'] == uid and actual['nice'] == 0:
                    os.setpriority(os.PRIO_PROCESS, saved['pid'], saved['nice'])
            except ProcessLookupError:
                pass
            except FileNotFoundError:
                pass
        print('Rolled back priority repair:', folder)
        return
    before = LIMITS.read_text()
    after = corrected_limits(before)
    print('PAM change: @realtime priority 99 -> 0; rtprio and memlock unchanged.')
    if not args.apply:
        print('Dry run only. Apply: sudo python3 scripts/fix_realtime_priority.py --apply')
        return
    folder = Path('/var/backups') / ('pi05-priority-' + time.strftime('%Y%m%d-%H%M%S'))
    folder.mkdir(mode=0o755)
    (folder / 'limits.conf.before').write_text(before)
    processes = []
    pid = os.getppid()
    while pid > 1:
        item = process_info(pid)
        if item['uid'] == uid and item['nice'] == 19:
            processes.append(item)
        pid = item['ppid']
    record = {'after_sha256': hashlib.sha256(after.encode()).hexdigest(), 'processes': processes}
    (folder / 'record.json').write_text(json.dumps(record, indent=2) + '\n')
    replace_file(LIMITS, after)
    for item in processes:
        os.setpriority(os.PRIO_PROCESS, item['pid'], 0)
    print('Applied. Current terminal ancestors restored to nice=0; future logins inherit nice=0.')
    print('Backup / rollback:', folder)
    print('Restart collection from THIS terminal after deployment is verified.')


if __name__ == '__main__':
    main()
