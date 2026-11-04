"""Test the S3 adapter and compile/link every application source in 8 profiles.

This is not an SDK/kernel firmware link or a hardware test. The ARM compile
uses real project headers and one standard memset declaration because this
Clang installation does not ship the Arm C library headers.
"""
from pathlib import Path
import hashlib
import itertools
import shutil
import subprocess
import sys

from run_tests import command, style_check

PROJECT = Path(__file__).resolve().parents[3]
CORE = PROJECT / "app_program/subsystem3"
TESTS = PROJECT / "testing/subsystem3/host"
BUILD = PROJECT / "testing/subsystem3/build/application"


def main():
    transcript = ["Application integration: host doubles and ARM partial links."]
    BUILD.mkdir(parents=True, exist_ok=True)
    status = 0
    try:
        sources = sorted((PROJECT / "app_program").glob("*.c"))
        sources += sorted((PROJECT / "app_program").glob("*/*.c"))
        changed = sorted(CORE.glob("*.c")) + sorted(CORE.glob("*.h"))
        changed += [TESTS / "adapter_main.c", TESTS / "adapter_main.h"]
        command(["clang-format", "-i", *map(str, changed)], transcript)
        import re
        for path in changed:
            text = path.read_text()
            text = re.sub(r"(?m)^            break;", "        break;", text)
            path.write_text(text.rstrip() + "\n\n", newline="\n")
        style_check(changed)
        includes = ["-I" + str(TESTS / "host_include")]
        for path in ("include", "config", "kernel/knlinc"):
            includes += ["-isystem", str(PROJECT / path)]
        includes += ["-I" + str(CORE)]
        common = ["-std=c99", "-D_PICO_RP2040_", "-DCNF_SMP=1", "-DAPP_TEST=0"]
        production = [CORE / name for name in
                      ("subsystem3.c", "ir_frame.c", "ir_processing.c",
                       "barcode_decoder.c", "code39.c")]
        executable = BUILD / ("adapter_tests.exe" if sys.platform == "win32"
                              else "adapter_tests")
        command(["clang", *common, *includes, *map(str, production),
                 str(PROJECT / "app_program/subsystem1/net_codec.c"),
                 str(PROJECT / "app_program/subsystem1/bitpack.c"),
                 str(TESTS / "adapter_main.c"), "-o", str(executable)],
                transcript)
        command([str(executable)], transcript)
        command([str(executable), "default"], transcript)
        navigation = BUILD / ("navigation_tests.exe" if sys.platform == "win32"
                              else "navigation_tests")
        command(["clang", *common, *includes,
                 str(PROJECT / "app_program/subsystem1/net_codec.c"),
                 str(PROJECT / "app_program/subsystem1/bitpack.c"),
                 str(PROJECT / "app_program/subsystem1/sm_logic.c"),
                 str(TESTS / "navigation_main.c"), "-o", str(navigation)],
                transcript)
        command([str(navigation)], transcript)
        # Only the C library declaration is substituted in ARM compile checks.
        headers = BUILD / "standard_headers"
        headers.mkdir(exist_ok=True)
        (headers / "string.h").write_text(
            "#ifndef S3_CHECK_STRING_H\n#define S3_CHECK_STRING_H\n"
            "#include <stddef.h>\nvoid *memset(void *, int, size_t);\n#endif\n")
        target_flags = ["--target=arm-none-eabi", "-mcpu=cortex-m0plus",
                        "-mthumb", "-ffreestanding", "-std=gnu11",
                        "-D_PICO_RP2040_"]
        target_includes = []
        for path in ("include", "config", "kernel/knlinc",
                     "lib/libwifi/sysdepend/pico_rp2040",
                     "lib/liblwip/sysdepend/pico_rp2040"):
            target_includes += ["-isystem", str(PROJECT / path)]
        target_includes += ["-I", str(headers)]
        linker = shutil.which("ld.lld")
        if not linker:
            raise RuntimeError("ld.lld is required for application partial links")
        for smp, test, mqtt in itertools.product((0, 1), repeat=3):
            label = f"smp{smp}_test{test}_mqtt{mqtt}"
            output = BUILD / label
            output.mkdir(exist_ok=True)
            objects = []
            profile = [f"-DCNF_SMP={smp}", f"-DAPP_TEST={test}",
                       f"-DTM_MQTT={mqtt}"]
            for source in sources:
                target = output / (source.stem + ".o")
                warnings = []
                if source.parent == CORE and source.stem != "subsystem3_test":
                    warnings = ["-Wall", "-Wextra", "-Werror", "-Wconversion",
                                "-Wshadow", "-Wstrict-prototypes",
                                "-Wmissing-prototypes", "-pedantic"]
                command(["clang", *target_flags, *profile, *target_includes,
                         *warnings, "-c", str(source), "-o", str(target)],
                        transcript)
                objects.append(target)
            combined = output / "application.o"
            command([linker, "-r", *map(str, objects), "-o", str(combined)],
                    transcript)
            symbols = command(["llvm-nm", str(combined)], transcript)
            if len(re.findall(r"(?m)^\S+ T s3_get_tlm$", symbols)) != 1:
                raise RuntimeError("Expected exactly one strong S3 telemetry getter")
            undefined = [line.split()[-1] for line in symbols.splitlines()
                         if " U " in line]
            prefixes = ("subsystem", "s3_", "ir_", "barcode_", "sm_",
                        "net_", "bitpack_", "bitread_", "app_start_task")
            unresolved = [name for name in undefined if name.startswith(prefixes)]
            if unresolved:
                raise RuntimeError(f"Unresolved application symbols: {unresolved}")
            transcript.append(f"PASS {label}: {len(sources)} sources, "
                              "partial link, strong S3 getter, app symbols resolved")
        transcript.append("PASS 8 application profiles; kernel/SDK link not run.")
        transcript.append("Application and integration-test SHA256:")
        for source in sources + [TESTS / "adapter_main.c"]:
            transcript.append(f"{source.relative_to(PROJECT)} " +
                              hashlib.sha256(source.read_bytes()).hexdigest())
    except (OSError, RuntimeError) as error:
        transcript.append("FAIL " + str(error))
        status = 1
    (BUILD / "verification.log").write_text("\n".join(transcript) + "\n")
    print("\n".join(transcript))
    return status


if __name__ == "__main__":
    sys.exit(main())
