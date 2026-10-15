from glob import glob
from pathlib import Path
import subprocess
from setuptools import setup
from setuptools.command.build_py import build_py

package_name = "uav_bc_recorder"


class BuildPy(build_py):
    """Keep source revision available when installed without source symlinks."""

    def run(self):
        super().run()
        try:
            result = subprocess.run(
                ["git", "-C", str(Path(__file__).resolve().parent), "rev-parse", "HEAD"],
                capture_output=True, text=True, check=True, timeout=3)
            commit = result.stdout.strip() or None
        except (OSError, subprocess.SubprocessError):
            commit = None
        destination = Path(self.build_lib) / package_name / "_build_info.py"
        destination.write_text(f"GIT_COMMIT = {commit!r}\n", encoding="utf-8")


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    cmdclass={"build_py": BuildPy},
    entry_points={
        "console_scripts": [
            "experiment_recorder = uav_bc_recorder.recorder_node:main",
            "phase2_runner = uav_bc_recorder.phase2_runner:main",
            "phase2_analysis = uav_bc_recorder.phase2_analysis:main",
        ],
    },
)