from __future__ import annotations

import json
import shutil
import unittest
from pathlib import Path

from scripts.bump_release_version import bump_release_version


class BumpReleaseVersionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.workspace = Path(__file__).resolve().parent / ".tmp"
        self.workspace.mkdir(exist_ok=True)

    def tearDown(self) -> None:
        shutil.rmtree(self.workspace, ignore_errors=True)

    def _write_version_file(self, directory: Path, version: str) -> Path:
        directory.mkdir(parents=True, exist_ok=True)
        version_file = directory / "version.json"
        version_file.write_text(
            json.dumps({"version": version}, indent=2) + "\n",
            encoding="utf-8",
        )
        return version_file

    def test_accepts_explicit_semver(self) -> None:
        version_file = self._write_version_file(self.workspace / "explicit", "0.1.0")

        version = bump_release_version(version_file=version_file, version="1.2.3")

        self.assertEqual(version, "1.2.3")
        self.assertEqual(
            json.loads(version_file.read_text(encoding="utf-8"))["version"],
            "1.2.3",
        )

    def test_normalizes_v_prefix(self) -> None:
        version_file = self._write_version_file(self.workspace / "prefixed", "0.1.0")

        version = bump_release_version(version_file=version_file, version="v2.0.1")

        self.assertEqual(version, "2.0.1")
        self.assertEqual(
            json.loads(version_file.read_text(encoding="utf-8"))["version"],
            "2.0.1",
        )

    def test_increments_patch_when_version_is_omitted(self) -> None:
        version_file = self._write_version_file(self.workspace / "implicit", "3.4.5")

        version = bump_release_version(version_file=version_file, version="")

        self.assertEqual(version, "3.4.6")
        self.assertEqual(
            json.loads(version_file.read_text(encoding="utf-8"))["version"],
            "3.4.6",
        )

    def test_rejects_invalid_explicit_version(self) -> None:
        version_file = self._write_version_file(self.workspace / "invalid", "0.1.0")

        with self.assertRaises(ValueError):
            bump_release_version(version_file=version_file, version="invalid")


if __name__ == "__main__":
    unittest.main()
