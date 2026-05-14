"""URDF validation: xacro processes + check_urdf passes."""
import subprocess
from pathlib import Path

PROJECT_ROOT = Path("/home/nguyenvd/workspace/dog_robot")
URDF_XACRO = PROJECT_ROOT / "dog_robot_ws/src/dog_robot_description/urdf/dog_robot.urdf.xacro"


def test_xacro_processes():
    """xacro must process the URDF without errors."""
    result = subprocess.run(
        ["xacro", str(URDF_XACRO)], capture_output=True, text=True
    )
    assert result.returncode == 0, f"xacro failed:\n{result.stderr}"
    assert "<robot" in result.stdout
    assert result.stdout.count("<link") >= 17
    assert result.stdout.count('type="revolute"') == 12


def test_check_urdf_passes():
    """check_urdf must accept the generated URDF."""
    xacro = subprocess.run(["xacro", str(URDF_XACRO)], capture_output=True, text=True)
    urdf_str = xacro.stdout
    tmp = Path("/tmp/dog_robot_test.urdf")
    tmp.write_text(urdf_str)
    result = subprocess.run(["check_urdf", str(tmp)], capture_output=True, text=True)
    assert result.returncode == 0, f"check_urdf failed:\n{result.stderr}"
    assert "Successfully Parsed XML" in result.stdout
