import tempfile,unittest
from contextlib import nullcontext
from unittest.mock import patch
from test_y42_calibration_flow import isolated_web_module

class ScanStateTests(unittest.TestCase):
    def test_probe_does_not_change_operational_port_or_baud(self):
        with tempfile.TemporaryDirectory() as folder:
            w=isolated_web_module(folder)
            w.STATE.update(port='COM5',baud=921600,auto='idle')
            with patch.object(w.direct,'open_serial',return_value=nullcontext(object())),patch.object(w.direct,'scan',return_value=[1]),patch.object(w,'active_auto',return_value=False):
                result=w.execute(dict(action='scan',port='COM8',baud=115200,start=1,end=10))
            self.assertEqual(result['found'],[1])
            self.assertEqual(w.STATE['baud'],921600)
            self.assertEqual(w.STATE['port'],'COM5')

if __name__=='__main__':unittest.main()
