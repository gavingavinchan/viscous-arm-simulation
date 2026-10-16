"""Package the portable runtime assets after successful current validation."""
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

ROOT = Path(__file__).resolve().parent


def main():
    report = json.loads((ROOT / "inspection/validation_report.json").read_text())
    urdf = ROOT / "model/star_arm_102_hd.urdf"
    assert report["passed"] and report["urdf_sha256"] == hashlib.sha256(urdf.read_bytes()).hexdigest()
    manifest_path = ROOT / "model/model_manifest.json"
    assert report["manifest_sha256"] == hashlib.sha256(manifest_path.read_bytes()).hexdigest()
    for part in json.loads(manifest_path.read_text())["parts"]:
        assert part["mesh_sha256"] == hashlib.sha256((ROOT / "model" / part["mesh"]).read_bytes()).hexdigest()
    paths = [ROOT / "README.md", ROOT / "inspection/validation_report.json",
             ROOT / "source/intake.json", ROOT / "source/LICENSE-scope.md"]
    paths += [p for p in (ROOT / "model").rglob("*") if p.is_file()]
    archive = ROOT / "star_arm_102_hd_model.zip"
    with ZipFile(archive, "w", ZIP_DEFLATED, compresslevel=6) as output:
        for path in sorted(paths):
            output.write(path, path.relative_to(ROOT))
    with ZipFile(archive) as output:
        assert output.testzip() is None
    print(f"{archive}: {len(paths)} files, {archive.stat().st_size:,} bytes")


if __name__ == "__main__":
    main()
