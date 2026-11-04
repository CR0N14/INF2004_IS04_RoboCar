"""Format, check, compile and replay the private Buddy 3 modules.

Requires Python 3, Clang and clang-format on PATH. No hardware or SDK needed.
"""
from pathlib import Path
import argparse
import hashlib
import re
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT.parents[1] / "app_program/subsystem3"
SOURCES = [CORE / name for name in
           ("code39.c", "ir_frame.c", "ir_processing.c", "barcode_decoder.c",
            "ir_transport.c")]
TEST = ROOT / "host/test_main.c"
FLAGS = ["-std=c99", "-Wall", "-Wextra", "-Werror", "-Wconversion",
         "-Wshadow", "-Wstrict-prototypes", "-Wmissing-prototypes", "-pedantic"]


def command(arguments, transcript):
    transcript.append("$ " + " ".join(map(str, arguments)))
    result = subprocess.run(arguments, cwd=ROOT, text=True,
                            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    transcript.append(result.stdout.rstrip())
    if result.returncode:
        raise RuntimeError(f"Command failed ({result.returncode})")
    return result.stdout


def style_check(files):
    for path in files:
        text = path.read_text(encoding="utf-8")
        for number, line in enumerate(text.splitlines(), 1):
            if len(line) > 80 or "\t" in line or line.rstrip() != line:
                raise RuntimeError(f"Whitespace: {path.name}:{number}")
        if not text.endswith("/* End of file. */\n\n"):
            raise RuntimeError(f"Missing end marker: {path}")
        code = re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)
        if re.search(r"\b(auto|register|goto|continue|abort|exit|setjmp|longjmp)\b",
                     code):
            raise RuntimeError(f"Forbidden token: {path}")
        if path.suffix == ".c" and not path.with_suffix(".h").exists():
            raise RuntimeError(f"Missing companion header: {path}")
        functions = re.finditer(
            r"(?m)^(?:static )?\w+(?:\s+\w+)?\s+(\w+) \([^;]*?\)\n\{",
            text)
        for match in functions:
            end = text.index("{", match.start()) + 1
            depth = 1
            while depth:
                depth += (text[end] == "{") - (text[end] == "}")
                end += 1
            if text[match.start():end].count("\n") >= 100:
                raise RuntimeError(f"Function exceeds 100 lines: {match[1]}")
    # This scan is a subset of Barr-C, not a compliance certification.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, default=ROOT / "build/test.log")
    parser.add_argument("--application", action="store_true",
                        help="also test the adapter and compile all app sources")
    parser.add_argument("--arm", action="store_true",
                        help="also compile freestanding Cortex-M0+ objects")
    args = parser.parse_args()
    transcript = ["Buddy 3 synthetic replay; no physical hardware evidence."]
    build = ROOT / "build"
    build.mkdir(exist_ok=True)
    status = 0
    try:
        for tool in ("clang", "clang-format"):
            if not shutil.which(tool):
                raise RuntimeError(f"Required tool missing: {tool}")
        files = sorted(CORE.glob("*.c")) + sorted(CORE.glob("*.h"))
        files = [path for path in files if path.stem not in
                 ("subsystem3", "subsystem3_test")]
        files += sorted((ROOT / "host").glob("*.c"))
        files += sorted((ROOT / "host").glob("*.h"))
        command(["clang-format", "-i", *map(str, files)], transcript)
        for path in files:
            text = path.read_text(encoding="utf-8")
            # Barr 8.2.a aligns break with case, unlike clang-format.
            text = re.sub(r"(?m)^            break;", "        break;", text)
            path.write_text(text.rstrip() + "\n\n", encoding="utf-8",
                            newline="\n")
        style_check(files)
        transcript.append("PASS automated Barr subset; manual review required.")
        command(["clang", "--version"], transcript)
        for source in SOURCES:
            command(["clang", "--analyze", *FLAGS, str(source), "-o",
                     str(build / (source.stem + ".plist"))], transcript)
        executable = build / ("host_tests.exe" if sys.platform == "win32"
                              else "host_tests")
        command(["clang", *FLAGS, "-I", str(CORE), *map(str, SOURCES), str(TEST),
                 "-o", str(executable)], transcript)
        command([str(executable)], transcript)
        for module in ("frame", "line", "barcode", "transport", "guards"):
            transcript.append("Independent module: " + module)
            command([str(executable), module], transcript)
        code39_test = build / ("code39_tests.exe" if sys.platform == "win32"
                                else "code39_tests")
        command(["clang", *FLAGS, "-I", str(CORE), *map(str, SOURCES),
                 str(ROOT / "host/code39_main.c"), "-o", str(code39_test)],
                transcript)
        command([str(code39_test)], transcript)
        if args.arm:
            objects = []
            for source in SOURCES:
                target = build / (source.stem + "_arm.o")
                command(["clang", "--target=arm-none-eabi",
                         "-mcpu=cortex-m0plus", "-mthumb", "-ffreestanding",
                         "-Os", "-fstack-usage", *FLAGS, "-c", str(source),
                         "-o", str(target)], transcript)
                objects.append(target)
            if shutil.which("llvm-size"):
                command(["llvm-size", *map(str, objects)], transcript)
            transcript.append("ARM objects only: no SDK, RTOS link or UF2.")
        if args.application:
            command([sys.executable, str(ROOT / "host/check_application.py")],
                    transcript)
        transcript.append("Source SHA256:")
        for path in files:
            transcript.append(f"{path.relative_to(ROOT.parents[1])} " +
                              hashlib.sha256(path.read_bytes()).hexdigest())
    except (RuntimeError, OSError) as error:
        transcript.append("FAIL " + str(error))
        status = 1
    args.log.parent.mkdir(parents=True, exist_ok=True)
    args.log.write_text("\n".join(transcript) + "\n", encoding="utf-8")
    print("\n".join(transcript))
    return status


if __name__ == "__main__":
    sys.exit(main())
