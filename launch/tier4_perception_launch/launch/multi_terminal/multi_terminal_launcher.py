import logging
import os
import shlex
import shutil
import tempfile

from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

logger = logging.getLogger(__name__)

TERMINAL_METHOD_PRIORITY = ('terminator', 'tmux')
# TERMINAL_METHOD_PRIORITY = ('tmux', 'terminator')
GUI_TERMINAL_CANDIDATES = (
    'gnome-terminal',
    'xterm',
    'konsole',
    'xfce4-terminal',
    'mate-terminal',
    'terminator',
)

TERMINATOR_SIGINT_WAIT_SECONDS = 20
TERMINATOR_SIGTERM_WAIT_SECONDS = 8
TERMINATOR_SIGKILL_WAIT_SECONDS = 3

TERMINAL_SIGTERM_TIMEOUT = 25.0
TERMINAL_SIGKILL_TIMEOUT = 10.0


def _first_available_command(candidates: tuple[str, ...]) -> str:
    """Return the first executable available from the candidates."""
    return next((cmd for cmd in candidates if shutil.which(cmd)), '')


def detect_terminal_method() -> str:
    """Detect available terminal method for launching separate terminals."""
    return _first_available_command(TERMINAL_METHOD_PRIORITY) or 'none'


def detect_gui_terminal() -> str:
    """Detect available GUI terminal emulator."""
    if os.environ.get('DISPLAY') is None:
        return ''
    return _first_available_command(GUI_TERMINAL_CANDIDATES)


def format_launch_args_for_command(context, launch_arguments_names: list[str]) -> list[str]:
    """Convert launch arguments to command-line format."""
    launch_args = []
    for name in launch_arguments_names:
        value = LaunchConfiguration(name).perform(context)
        if value is not None:
            launch_args.append(f"{name}:={value}")
    return launch_args


def escape_dollar_signs_for_bash(cmd_str: str) -> str:
    """Escape dollar signs to preserve ROS2 launch substitutions."""
    return cmd_str.replace("$", "\\$")


def _get_workspace_setup_path(launcher_pkg_install_dir: str) -> str:
    """Get workspace setup.bash path."""
    workspace_install_dir = os.path.dirname(os.path.dirname(os.path.dirname(launcher_pkg_install_dir)))
    return os.path.join(workspace_install_dir, 'setup.bash')


def _build_command_string(cmd_parts: list[str], escape_dollars: bool = True) -> str:
    """Build quoted command string from parts."""
    cmd = ' '.join(shlex.quote(p) for p in cmd_parts)
    return escape_dollar_signs_for_bash(cmd) if escape_dollars else cmd


def _write_temp_script(content: str, prefix: str, suffix: str = '.sh') -> str:
    """Write script content to temporary file and return path."""
    fd, script_path = tempfile.mkstemp(suffix=suffix, prefix=prefix)
    with os.fdopen(fd, 'w') as f:
        f.write(content)
    os.chmod(script_path, 0o755)
    return script_path


def create_tmux_launcher_script(launcher_paths: list[str], launch_args_cmd: list[str],
                                launcher_pkg_install_dir: str,
                                launcher_pkg_name: str = 'tier4_perception_launch',
                                session_name: str = 'ros2_launchers',
                                window_name: str = 'launchers',
                                launch_pointcloud_container: bool = False) -> str:
    """Delegate tmux script creation to the tmux helper module."""
    from . import tmux as tmux_module

    return tmux_module.create_tmux_launcher_script(
        launcher_paths=launcher_paths,
        launch_args_cmd=launch_args_cmd,
        launcher_pkg_install_dir=launcher_pkg_install_dir,
        launcher_pkg_name=launcher_pkg_name,
        session_name=session_name,
        window_name=window_name,
        launch_pointcloud_container=launch_pointcloud_container,
    )


def launch_in_tmux(context, launch_arguments_names: list[str], launcher_paths: list[str],
                   launcher_pkg_install_dir: str,
                   launcher_pkg_name: str = 'tier4_perception_launch',
                   session_name: str = 'ros2_launchers',
                   window_name: str = 'launchers',
                   launch_pointcloud_container: bool = False) -> list[ExecuteProcess]:
    """Public wrapper that defers to the tmux helper for launching."""
    from . import tmux as tmux_module

    return tmux_module.launch_in_tmux(
        context=context,
        launch_arguments_names=launch_arguments_names,
        launcher_paths=launcher_paths,
        launcher_pkg_install_dir=launcher_pkg_install_dir,
        launcher_pkg_name=launcher_pkg_name,
        session_name=session_name,
        window_name=window_name,
        launch_pointcloud_container=launch_pointcloud_container,
    )


def create_terminator_launcher_script(launcher_paths: list[str], launch_args_cmd: list[str],
                                      launcher_pkg_install_dir: str,
                                      launcher_pkg_name: str = 'tier4_perception_launch',
                                      layout_name: str = 'ros2_launchers',
                                      launch_pointcloud_container: bool = False,
                                      titles: list[str] = None) -> str:
    """Delegate to the terminator helper module for script creation."""
    from . import terminator as terminator_module

    return terminator_module.create_terminator_launcher_script(
        launcher_paths=launcher_paths,
        launch_args_cmd=launch_args_cmd,
        launcher_pkg_install_dir=launcher_pkg_install_dir,
        launcher_pkg_name=launcher_pkg_name,
        layout_name=layout_name,
        launch_pointcloud_container=launch_pointcloud_container,
        titles=titles,
    )


def launch_in_terminator(context, launch_arguments_names: list[str], launcher_paths: list[str],
                         launcher_pkg_install_dir: str,
                         launcher_pkg_name: str = 'tier4_perception_launch',
                         layout_name: str = 'ros2_launchers',
                         launch_pointcloud_container: bool = False,
                         titles: list[str] = None) -> list[ExecuteProcess]:
    """Public wrapper that defers to the terminator helper for launching."""
    from . import terminator as terminator_module

    return terminator_module.launch_in_terminator(
        context=context,
        launch_arguments_names=launch_arguments_names,
        launcher_paths=launcher_paths,
        launcher_pkg_install_dir=launcher_pkg_install_dir,
        launcher_pkg_name=launcher_pkg_name,
        layout_name=layout_name,
        launch_pointcloud_container=launch_pointcloud_container,
        titles=titles,
    )
