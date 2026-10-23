import copy
from contextlib import redirect_stdout, redirect_stderr
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest

from lunar_rover.cli import main

ROOT = Path(__file__).resolve().parents[1]
SCENARIO = ROOT/'configs/scenarios/synthetic_obstacles.json'


class CliTests(unittest.TestCase):
    def run_cli(self, *arguments):
        with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            return main(list(arguments))

    def test_complete_run_has_figures_and_verifiable_checksums(self):
        with tempfile.TemporaryDirectory() as work:
            output = Path(work)/'run'
            self.assertEqual(self.run_cli('--scenario',str(SCENARIO),'--output',str(output)),0)
            for filename in ('scenario.json','metrics.json','trajectory.csv','planned_route.csv','overview.png','overview.svg','manifest.json'):
                self.assertTrue((output/filename).is_file(),filename)
            manifest = json.loads((output/'manifest.json').read_text(encoding='utf-8'))
            self.assertEqual(manifest['data_type'],'synthetic')
            for filename,expected in manifest['output_sha256'].items():
                self.assertEqual(hashlib.sha256((output/filename).read_bytes()).hexdigest(),expected)
            original = (output/'metrics.json').read_bytes()
            self.assertEqual(self.run_cli('--scenario',str(SCENARIO),'--output',str(output)),1)
            self.assertEqual((output/'metrics.json').read_bytes(),original)

    def test_no_path_records_failure_without_fake_trajectory(self):
        raw = json.loads(SCENARIO.read_text(encoding='utf-8'))
        raw['grid']['obstacle_rectangles_rc'] = [[0,24,47,27]]
        with tempfile.TemporaryDirectory() as work:
            scenario,output = Path(work)/'blocked.json',Path(work)/'result'
            scenario.write_text(json.dumps(raw),encoding='utf-8')
            self.assertEqual(self.run_cli('--scenario',str(scenario),'--output',str(output)),2)
            self.assertEqual(json.loads((output/'metrics.json').read_text())['status'],'no_path')
            self.assertTrue((output/'manifest.json').exists())
            self.assertFalse((output/'trajectory.csv').exists())

    def test_invalid_input_returns_actionable_error(self):
        with tempfile.TemporaryDirectory() as work:
            self.assertEqual(self.run_cli('--scenario',str(Path(work)/'missing.json'),
                                         '--output',str(Path(work)/'result')),1)
