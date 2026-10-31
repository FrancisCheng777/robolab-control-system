import importlib.util
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location('network_priority', Path(__file__).resolve().parents[1]/'scripts/fix_fci_network_priority.py')
network = importlib.util.module_from_spec(spec)
spec.loader.exec_module(network)


class NetworkStartupTests(unittest.TestCase):
    def test_creates_missing_lock_and_rejects_concurrent_holder(self):
        with tempfile.TemporaryDirectory() as root:
            path = Path(root)/'collection.lock'
            fd = network.collection_lock(path, os.getuid(), os.getgid())
            try:
                self.assertTrue(path.is_file())
                self.assertEqual(path.stat().st_uid, os.getuid())
                with self.assertRaisesRegex(RuntimeError, 'active'):
                    network.collection_lock(path, os.getuid(), os.getgid())
            finally:
                os.close(fd)

    def test_refuses_symlink_lock(self):
        with tempfile.TemporaryDirectory() as root:
            target=Path(root)/'target';target.touch()
            link=Path(root)/'lock';link.symlink_to(target)
            with self.assertRaises(OSError):
                network.collection_lock(link,os.getuid(),os.getgid())

    def test_already_applied_is_success_without_changing_scheduler(self):
        with tempfile.TemporaryFile() as fd:
            with patch('sys.argv',['fix_fci_network_priority.py','--apply']), \
                 patch.object(network.os,'geteuid',return_value=0), \
                 patch.object(network.pwd,'getpwnam') as user, \
                 patch.object(network,'collection_lock',return_value=os.dup(fd.fileno())), \
                 patch.object(network,'discover',return_value={'policy':os.SCHED_FIFO,'priority':50}), \
                 patch.object(network,'apply_priority',side_effect=AssertionError('must not reapply')):
                user.return_value.pw_uid=os.getuid();user.return_value.pw_gid=os.getgid()
                network.main()


if __name__ == '__main__': unittest.main()
