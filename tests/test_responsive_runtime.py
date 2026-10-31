import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

from pi05_demo.teleop_runtime import configure_runtime

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
spec = importlib.util.spec_from_file_location('usb_latency', ROOT/'scripts/set_teleop_usb_latency.py')
usb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(usb)


class ResponsiveRuntimeTests(unittest.TestCase):
    def test_default_does_not_change_scheduling(self):
        with patch('os.sched_setaffinity', create=True) as affinity, patch('sys.setswitchinterval') as gil:
            configure_runtime({})
        affinity.assert_not_called()
        gil.assert_not_called()

    def test_opt_in_pins_only_calling_thread(self):
        with patch('os.sched_getaffinity', return_value={13,14}, create=True), \
             patch('os.sched_setaffinity', create=True) as affinity, patch('sys.setswitchinterval') as gil:
            configure_runtime({'teleop_main_cpu':13, 'teleop_gil_switch_interval_s':.001})
        affinity.assert_called_once_with(0, {13})
        gil.assert_called_once_with(.001)

    def test_reserved_or_unavailable_cpu_rejected_without_changes(self):
        for cpu in (11,12,99):
            with patch('os.sched_getaffinity', return_value={13,14}, create=True), \
                 patch('os.sched_setaffinity', create=True) as affinity, patch('sys.setswitchinterval') as gil:
                with self.assertRaises(ValueError):
                    configure_runtime({'teleop_main_cpu':cpu, 'teleop_gil_switch_interval_s':.001})
                affinity.assert_not_called()
                gil.assert_not_called()

    def test_usb_apply_and_repeat_are_idempotent(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'latency';path.write_text('16\n')
            devices={'gello':dict(attribute=str(path),latency_ms=16)}
            usb.apply_latency(devices)
            self.assertEqual(path.read_text(),'1\n')
            devices['gello']['latency_ms']=1
            with patch.object(Path,'write_text',side_effect=AssertionError('already configured')):
                usb.apply_latency(devices)

    def test_partial_usb_failure_restores_previous_values(self):
        with tempfile.TemporaryDirectory() as directory:
            one=Path(directory)/'one';two=Path(directory)/'two'
            one.write_text('16\n');two.write_text('8\n')
            devices={k:dict(attribute=str(p),latency_ms=v) for k,p,v in [('gello',one,16),('robotiq',two,8)]}
            original=Path.write_text
            def write(path, text, *args, **kwargs):
                if path == two and text == '1\n': raise OSError('injected write failure')
                return original(path,text,*args,**kwargs)
            with patch.object(Path,'write_text',write), self.assertRaises(OSError):
                usb.apply_latency(devices)
            self.assertEqual(one.read_text(),'16\n')
            self.assertEqual(two.read_text(),'8\n')
