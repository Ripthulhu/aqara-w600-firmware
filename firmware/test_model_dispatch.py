"""Dispatch-only synthetic self-checks; never execute a firmware scenario."""
import ast
from concurrent.futures import ProcessPoolExecutor
from contextlib import redirect_stdout
import hashlib
import io
from itertools import repeat
import json
from multiprocessing import get_context
import operator
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

HERE = Path(__file__).resolve().parent
RUN_HASHES = {
    "application_thumb.py": "3212262d1a3db2a2adf60939bbe466fd3d2f57343d9a2f0bb26c5cb5f4d738ed",
    "test_continuous_home_thumb.py": "971571095304910ef102992e83d4885ecb31e78d21f32b76431f816ca310b93f",
    "test_confirmation_fault_thumb.py": "83a15e30f3d74f8fefce860615cea5ee0c95749e2399a4b4d22dbfb50ecaff2a",
}


class Dispatch(unittest.TestCase):
    def test_run_bodies_are_byte_identical(self):
        for name, expected in RUN_HASHES.items():
            text = (HERE / name).read_text()
            node = next(node for node in ast.parse(text).body if isinstance(node, ast.FunctionDef) and node.name == "run")
            self.assertEqual(hashlib.sha256(ast.get_source_segment(text, node).encode()).hexdigest(), expected, name)

    def main_fixture(self, name, fail=False, args=()):
        calls = []
        class Pool:
            def __init__(inner, max_workers, mp_context):
                self.assertEqual(max_workers, 3)
                self.assertEqual(mp_context.get_start_method(), "spawn")
            def __enter__(inner): return inner
            def __exit__(inner, *_): return False
            def map(inner, function, *iterables): return map(function, *iterables)
        def worker(*values):
            calls.append(values[-1])
            if fail and len(calls) == 2: raise RuntimeError("synthetic worker failure")
            return {"scenario": values[-1]}
        with tempfile.TemporaryDirectory() as folder:
            directory = Path(folder)
            image = b"synthetic dispatch-only input"
            (directory / "replacement-core.bin").write_bytes(image)
            manifest = dict(bin_sha256=hashlib.sha256(image).hexdigest(), flash_application=True,
                            automatic_startup_motion=False, code_address=0x08002000, symbols={},
                            test_confirmation_layout=[1] + [0] * 22)
            (directory / "manifest.json").write_text(json.dumps(manifest))
            module = SimpleNamespace(__file__=str(HERE / name))
            service = SimpleNamespace(__file__=module.__file__, display=module, probe=module)
            app = SimpleNamespace(__file__=module.__file__, service=service, base=module)
            namespace = dict(__file__=module.__file__, Path=Path, hashlib=hashlib, json=json,
                             sys=SimpleNamespace(argv=[name, folder, *args]), SCENARIOS=("first", "second", "third"),
                             ProcessPoolExecutor=Pool, get_context=get_context, repeat=repeat,
                             run=worker, service=service, base=module, app=app)
            node = next(node for node in ast.parse((HERE / name).read_text()).body
                        if isinstance(node, ast.FunctionDef) and node.name == "main")
            exec(compile(ast.Module(body=[node], type_ignores=[]), name, "exec"), namespace)
            with redirect_stdout(io.StringIO()):
                if fail:
                    with self.assertRaisesRegex(RuntimeError, "synthetic worker failure"):
                        namespace["main"]()
                else: namespace["main"]()
            files = [path for path in directory.glob("*results.json")]
            result = json.loads(files[0].read_text()) if files else None
            if fail: self.assertTrue(result is None or result["complete"] is False)
            return calls, result

    def test_three_worker_bound_order_and_parent_output(self):
        scenarios = {
            "application_thumb.py": ("first", "second", "third"),
            "test_continuous_home_thumb.py": ("steady", "slow_start", "fallback", "both_ambiguous", "early_endpoint"),
            "test_confirmation_fault_thumb.py": ("candidate_rest", "backoff_rest", "confirmed_home"),
        }
        for name, expected in scenarios.items():
            calls, result = self.main_fixture(name)
            self.assertEqual(tuple(calls), expected)
            rows = result["cases"]
            self.assertEqual(tuple(rows) if isinstance(rows, dict) else tuple(row["scenario"] for row in rows), expected)
            self.assertTrue(result["complete"])

    def test_worker_exception_cannot_become_complete(self):
        for name in RUN_HASHES:
            with self.subTest(name=name): self.main_fixture(name, fail=True)

    def test_application_cli_selection_still_runs_each_requested_case(self):
        calls, result = self.main_fixture("application_thumb.py", args=("third", "first", "third"))
        self.assertEqual(calls, ["third", "first", "third"])
        self.assertFalse(result["complete"])

    def test_actual_spawn_pool_keeps_input_order_and_propagates_failure(self):
        with ProcessPoolExecutor(max_workers=3, mp_context=get_context("spawn")) as pool:
            self.assertEqual(list(pool.map(operator.neg, [1, 2, 3])), [-1, -2, -3])
            with self.assertRaises(ZeroDivisionError):
                list(pool.map(operator.floordiv, [1, 1, 1], [1, 0, 1]))


if __name__ == "__main__":
    unittest.main()
