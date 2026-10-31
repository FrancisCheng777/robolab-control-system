#!/usr/bin/env python3
"""Inspect/apply temporary 1 ms FTDI buffering, without opening either serial port."""
import argparse
import json
import os
from pathlib import Path
import pwd
import re
import time

from fix_fci_network_priority import collection_lock

DEVICES = {
    'gello': 'usb-FTDI_USB__-__Serial_Converter_FTBYPQ5E-if00-port0',
    'robotiq': 'usb-FTDI_USB_TO_RS-485_DAAL8Y3V-if00-port0',
}


def discover():
    result = {}
    for name, identity in DEVICES.items():
        device = (Path('/dev/serial/by-id') / identity).resolve(strict=True)
        if device.parent != Path('/dev') or not re.fullmatch(r'ttyUSB[0-9]+', device.name):
            raise RuntimeError('Unexpected USB device: ' + str(device))
        attribute = Path('/sys/bus/usb-serial/devices') / device.name / 'latency_timer'
        result[name] = dict(device=str(device), attribute=str(attribute),
                            latency_ms=int(attribute.read_text()))
    return result


def apply_latency(devices):
    changed = []
    try:
        for info in devices.values():
            if info['latency_ms'] == 1:
                continue
            path = Path(info['attribute'])
            changed.append(info)
            path.write_text('1\n')
            if int(path.read_text()) != 1:
                raise RuntimeError('USB latency verification failed: ' + str(path))
    except BaseException as exc:
        errors = []
        for info in reversed(changed):
            try:
                Path(info['attribute']).write_text(str(info['latency_ms']) + '\n')
            except OSError as rollback_error:
                errors.append(str(rollback_error))
        if errors:
            raise RuntimeError('USB setup failed; rollback also failed: ' + '; '.join(errors)) from exc
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--check', action='store_true')
    group.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    if not args.apply:
        devices = discover()
        print(json.dumps(devices, indent=2))
        if args.check and any(d['latency_ms'] != 1 for d in devices.values()):
            raise SystemExit(1)
        return
    if os.geteuid() != 0:
        raise SystemExit('sudo is required for temporary USB latency setup')
    user = pwd.getpwnam('heinrich')
    fd = collection_lock('/tmp/pi05-robolab-trial-20261030.lock', user.pw_uid, user.pw_gid)
    try:
        for process in Path('/proc').iterdir():
            if not process.name.isdigit():
                continue
            try:
                argv = (process/'cmdline').read_bytes().split(b'\0')
            except (FileNotFoundError, ProcessLookupError):
                continue
            if argv and Path(os.fsdecode(argv[0])).name == 'pi05_franka_bridge':
                raise RuntimeError('Exit the active bridge before changing USB latency')
        devices = discover()
        if any(d['latency_ms'] != 1 for d in devices.values()):
            backup = Path('/var/backups') / ('robolab-usb-latency-' + time.strftime('%Y%m%d-%H%M%S') + '.json')
            with backup.open('x') as stream:
                json.dump(devices, stream, indent=2)
            print('Previous values saved to:', backup)
            apply_latency(devices)
        print(json.dumps(discover(), indent=2))
        print('Temporary settings only. No device opened or motion command sent.')
    finally:
        os.close(fd)


if __name__ == '__main__':
    main()
