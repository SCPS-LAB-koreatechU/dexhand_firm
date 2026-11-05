"""Package the current full PC SDK and both complete upload folders, without caches or old ZIPs.

Does not run hardware or certify tests. Existing manifest records verification;
source SHA256s in it must match before packaging. Run verify.py first.
"""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "dexhand_control/firmware/안내사항/BUILD_MANIFEST_20261104.json"
OUTPUT = ROOT / "UpperBody_SDK_0.2.0_20261104.zip"
EXCLUDE = {"__pycache__", ".pytest_cache", ".git", "build", "install", "log", ".venv", ".cache"}


def files():
    selected = [ROOT / "README.md"]
    for folder in ("humanoid_sdk-main", "dexhand_control"):
        for path in (ROOT / folder).rglob("*"):
            if (path.is_file() and not EXCLUDE.intersection(path.relative_to(ROOT).parts) and
                    path.suffix.lower() not in {".zip", ".pyc", ".pyo", ".exe", ".o"}):
                selected.append(path)
    return sorted(selected)


def main():
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    revision = manifest.get("right_wiring_revision", 1)
    output = OUTPUT if revision == 1 else OUTPUT.with_name(OUTPUT.stem + f"_pinrev{revision}.zip")
    if manifest.get("hand_demo_revision"):
        output = output.with_name(output.stem + f"_handdemo{manifest['hand_demo_revision']}.zip")
    if manifest.get("hand_gui_revision"):
        output = output.with_name(output.stem + f"_gui{manifest['hand_gui_revision']}.zip")
    if manifest.get("can_wiring") == {"tx_gpio": 1, "rx_gpio": 3}:
        output = output.with_name(output.stem + "_can13.zip")
    if manifest.get("sdk_version") != "0.2.0":
        raise SystemExit("Update the versioned verification manifest first")
    for relative, expected in manifest["source_sha256"].items():
        if hashlib.sha256((ROOT / relative).read_bytes()).hexdigest() != expected:
            raise SystemExit("Manifest source mismatch: " + relative)
    selected = files()
    expected_files = {p.relative_to(ROOT).as_posix() for p in selected if p != MANIFEST}
    if expected_files != set(manifest["source_sha256"]):
        raise SystemExit("Manifest must list exactly all release source files")
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in selected:
            archive.write(path, path.relative_to(ROOT).as_posix())
    with zipfile.ZipFile(output) as archive:
        if archive.testzip() is not None:
            raise RuntimeError("ZIP integrity check failed")
        for relative, expected in manifest["source_sha256"].items():
            if hashlib.sha256(archive.read(relative)).hexdigest() != expected:
                raise RuntimeError("ZIP hash mismatch: " + relative)
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_suffix(".zip.sha256").write_text(digest + "  " + output.name + "\n", encoding="ascii")
    print(f"PASS: {len(selected)} files, {output.stat().st_size} bytes, {output}")
    print("SHA256: " + digest)


if __name__ == "__main__":
    main()
