"""Pure preview generation tests; no HTTP server or motor connection."""
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import y42_twin_server as server


class HotReloadTests(unittest.TestCase):
    def test_generation_reuses_unchanged_code_and_rolls_back_failed_load(self):
        names=('y42_exact_path','y42_spool_geometry','y42_twin_model')
        previous={name:sys.modules.get(name) for name in names}
        old_cache=server.MODEL_CACHE.copy()
        try:
            with tempfile.TemporaryDirectory() as directory:
                root=Path(directory);(root/'tools').mkdir()
                for name in names:
                    (root/'tools'/f'{name}.py').write_text('value = 1\n',encoding='utf-8')
                with patch.object(server,'ROOT',root):
                    server.MODEL_CACHE.clear()
                    first,v1=server.simulation_model()
                    self.assertIs(server.simulation_model()[0],first)
                    source=root/'tools/y42_twin_model.py'
                    source.write_text('value = 2\n',encoding='utf-8')
                    second,v2=server.simulation_model()
                    self.assertNotEqual(v1,v2)
                    self.assertEqual(first.value,1)
                    self.assertEqual(second.value,2)
                    source.write_text('invalid python !\n',encoding='utf-8')
                    with self.assertRaises(SyntaxError):server.simulation_model()
                    self.assertIs(sys.modules['y42_twin_model'],second)
                    self.assertEqual(server.MODEL_CACHE['version'],v2)
        finally:
            server.MODEL_CACHE.clear();server.MODEL_CACHE.update(old_cache)
            for name,module in previous.items():
                if module is None:sys.modules.pop(name,None)
                else:sys.modules[name]=module


if __name__=='__main__':unittest.main()
