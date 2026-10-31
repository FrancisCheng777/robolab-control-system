import importlib.util
from pathlib import Path
import unittest

path = Path(__file__).resolve().parents[1] / 'scripts/fix_realtime_priority.py'
spec = importlib.util.spec_from_file_location('repair', path)
repair = importlib.util.module_from_spec(spec)
spec.loader.exec_module(repair)


class PriorityRepairTests(unittest.TestCase):
    def test_repairs_nice_without_changing_realtime_or_memlock(self):
        text = ('# priority 99 is wrong\n@realtime soft rtprio 99\n'
                '@realtime soft priority 99\n@realtime hard priority 99 # old\n'
                '@realtime hard rtprio 99\n@realtime hard memlock 102400\n'
                'someone - priority 10\n')
        expected = text.replace('soft priority 99', 'soft priority 0').replace('hard priority 99', 'hard priority 0')
        self.assertEqual(repair.corrected_limits(text), expected)

    def test_refuses_unknown_or_already_repaired_config(self):
        for text in ('@realtime - priority 0\n', '@realtime - rtprio 99\n'):
            with self.assertRaises(RuntimeError):
                repair.corrected_limits(text)


if __name__ == '__main__':
    unittest.main()
