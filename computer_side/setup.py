import os
import shlex
import shutil
import subprocess
import sys
import sysconfig
from pathlib import Path

from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    def __init__(self, name):
        super().__init__(name, sources=[])


class CMakeBuild(build_ext):
    def _cmake_executable(self):
        candidates = [
            os.environ.get("CMAKE_EXECUTABLE"),
            shutil.which("cmake"),
            "/usr/bin/cmake",
        ]

        for candidate in candidates:
            if not candidate:
                continue

            try:
                subprocess.check_call(
                    [candidate, "--version"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                )
                return candidate
            except (OSError, subprocess.CalledProcessError):
                continue

        raise RuntimeError("Could not find a working cmake executable")

    def build_extension(self, ext):
        source_dir = Path(__file__).parent.resolve()
        build_dir = source_dir / "build" / f"pip-{sysconfig.get_python_version()}"
        ext_dir = Path(self.get_ext_fullpath(ext.name)).parent.resolve()
        cmake = self._cmake_executable()

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={ext_dir}",
            "-DBUILD_PYTHON_BINDINGS=ON",
            "-DCMAKE_BUILD_TYPE=Release",
            f"-DPython_EXECUTABLE={sys.executable}",
            f"-DPython3_EXECUTABLE={sys.executable}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
        ]

        try:
            import pybind11

            cmake_args.append(f"-Dpybind11_DIR={pybind11.get_cmake_dir()}")
        except ImportError:
            pass

        cmake_args += shlex.split(os.environ.get("CMAKE_ARGS", ""))

        for stale_module in ext_dir.glob("kuka_fri_py*.so"):
            stale_module.unlink()

        build_dir.mkdir(parents=True, exist_ok=True)
        subprocess.check_call([cmake, "-S", str(source_dir), "-B", str(build_dir), *cmake_args])
        subprocess.check_call([cmake, "--build", str(build_dir), "--target", "kuka_fri_py", "-j4"])

        built_modules = list(ext_dir.glob("kuka_fri_py*.so"))
        if not built_modules:
            raise RuntimeError("CMake finished, but kuka_fri_py shared module was not produced")


setup(
    name="kuka-fri-py",
    version="0.1.0",
    description="Python bindings for the KUKA FRI controller",
    long_description=(Path(__file__).parent / "README.md").read_text(encoding="utf-8"),
    long_description_content_type="text/markdown",
    python_requires=">=3.10",
    install_requires=["numpy"],
    ext_modules=[CMakeExtension("kuka_fri_py")],
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
)
