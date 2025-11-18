import os
import shlex

from launch.actions import ExecuteProcess

from . import multi_terminal_launcher as base


def _build_tmux_launch_commands(launcher_paths: list[str], launch_args_cmd: list[str],
                                launcher_pkg_install_dir: str) -> list[tuple[str, str]]:
    """Return (launcher_name, command_part) tuples for tmux scripts."""
    commands: list[tuple[str, str]] = []
    for launcher_path in launcher_paths:
        launcher_name = os.path.basename(launcher_path)
        full_path = os.path.join(launcher_pkg_install_dir, launcher_path)
        cmd_part = base._build_command_string(['ros2', 'launch', full_path] + launch_args_cmd)
        commands.append((launcher_name, cmd_part))
    return commands


def create_tmux_launcher_script(launcher_paths: list[str], launch_args_cmd: list[str],
                                launcher_pkg_install_dir: str,
                                launcher_pkg_name: str = 'tier4_perception_launch',
                                session_name: str = 'ros2_launchers',
                                window_name: str = 'launchers',
                                launch_pointcloud_container: bool = False) -> str:
    """Create a bash script that launches tmux with split panes and runs launchers."""
    setup_bash_path = base._get_workspace_setup_path(launcher_pkg_install_dir)
    launch_commands = _build_tmux_launch_commands(launcher_paths, launch_args_cmd, launcher_pkg_install_dir)
    total_panes = len(launch_commands) + (1 if launch_pointcloud_container else 0)
    window_height, window_width = max(80, total_panes * 5), 200
    gui_terminal = base.detect_gui_terminal()

    lines = [
        "#!/bin/bash", "set -e", "",
        f"SESSION_NAME={shlex.quote(session_name)}",
        f"WINDOW_NAME={shlex.quote(window_name)}",
        f"WORKSPACE_SETUP={shlex.quote(setup_bash_path)}", "",
        "tmux has-session -t $SESSION_NAME 2>/dev/null && tmux kill-session -t $SESSION_NAME || true",
        "command -v tmux &> /dev/null || { echo 'ERROR: tmux not found' >&2; exit 1; }", "",
        f"tmux new-session -d -s $SESSION_NAME -n $WINDOW_NAME -x {window_width} -y {window_height} 'bash' || {{ echo 'ERROR: Failed to create tmux session' >&2; exit 1; }}",
        f"tmux resize-window -t $SESSION_NAME:0 -x {window_width} -y {window_height} 2>/dev/null || true",
        "sleep 0.3",
    ]

    if gui_terminal:
        term_commands = {
            'gnome-terminal': f"gnome-terminal --geometry {window_width}x{window_height}+10+10 -- bash -c \"tmux attach-session -t $SESSION_NAME || bash\"",
            'xterm': f"xterm -geometry {window_width}x{window_height} -e bash -c \"tmux attach-session -t $SESSION_NAME || bash\"",
            'konsole': f"konsole --geometry {window_width}x{window_height} -e bash -c \"tmux attach-session -t $SESSION_NAME || bash\"",
            'xfce4-terminal': f"xfce4-terminal --geometry {window_width}x{window_height} -e \"bash -c \\\"tmux attach-session -t $SESSION_NAME || bash\\\"\"",
            'mate-terminal': f"mate-terminal --geometry {window_width}x{window_height} -e \"bash -c \\\"tmux attach-session -t $SESSION_NAME || bash\\\"\"",
            'terminator': f"terminator -e \"bash -c \\\"tmux attach-session -t $SESSION_NAME || bash\\\"\"",
        }
        gui_cmd = term_commands.get(gui_terminal)
        if gui_cmd:
            lines.extend([f"{gui_cmd} &", "sleep 1"])

    lines.append("LAUNCHERS_SKIPPED=false")

    if launch_pointcloud_container:
        cmd_part = base._build_command_string([
            'ros2', 'run', 'rclcpp_components', 'component_container',
            '--ros-args', '-r', '__node:=pointcloud_container', '--log-level', 'info',
        ])
        lines.extend([
            f"CONTAINER_CMD_PART={shlex.quote(cmd_part)}",
            "CONTAINER_CMD=\"source $WORKSPACE_SETUP && $CONTAINER_CMD_PART\"",
            "echo \"Launching pointcloud container\"",
            "tmux send-keys -t $SESSION_NAME:0.0 \"$CONTAINER_CMD\" Enter",
            "sleep 1",
        ])

    if launch_commands:
        first_name, first_cmd_part = launch_commands[0]

        if not launch_pointcloud_container:
            lines.extend([
                f"LAUNCH_CMD_PART_0={shlex.quote(first_cmd_part)}",
                "LAUNCH_CMD_0=\"source $WORKSPACE_SETUP && $LAUNCH_CMD_PART_0\"",
                f"echo \"Launching: {first_name}\"",
                "tmux send-keys -t $SESSION_NAME:0.0 \"$LAUNCH_CMD_0\" Enter",
                "sleep 1",
            ])
        else:
            lines.extend([
                "tmux split-window -v -t $SESSION_NAME:0 'bash' || { LAUNCHERS_SKIPPED=true; }",
                "[ \"$LAUNCHERS_SKIPPED\" = \"false\" ] && {",
                f"  LAUNCH_CMD_PART_0={shlex.quote(first_cmd_part)}",
                "  LAUNCH_CMD_0=\"source $WORKSPACE_SETUP && $LAUNCH_CMD_PART_0\"",
                f"  echo \"Launching: {first_name}\"",
                "  tmux send-keys -t $SESSION_NAME:0. \"$LAUNCH_CMD_0\" Enter",
                "  sleep 1",
                "}",
            ])

    initial_panes = 1 + (1 if launch_pointcloud_container and launch_commands else 0)
    lines.append(f"SUCCESSFUL_PANES=$(({initial_panes}))")
    lines.append("[ \"$LAUNCHERS_SKIPPED\" != \"true\" ] && {")

    for i, (launcher_name, cmd_part) in enumerate(launch_commands[1:], 1):
        lines.extend([
            "  tmux split-window -v -t $SESSION_NAME:0 'bash' || break",
            f"  LAUNCH_CMD_PART_{i}={shlex.quote(cmd_part)}",
            f"  LAUNCH_CMD_{i}=\"source $WORKSPACE_SETUP && $LAUNCH_CMD_PART_{i}\"",
            f"  echo \"Launching: {launcher_name}\"",
            f"  tmux send-keys -t $SESSION_NAME:0. \"$LAUNCH_CMD_{i}\" Enter",
            "  SUCCESSFUL_PANES=$((SUCCESSFUL_PANES + 1))",
            "  sleep 1",
        ])
    lines.append("}")

    lines.extend([
        "tmux select-layout -t $SESSION_NAME:0 even-vertical",
        f"echo \"Launched $SUCCESSFUL_PANES/{total_panes} panes\"", "",
    ])

    if not gui_terminal:
        lines.append("[ -z \"$TMUX\" ] && [ -t 0 ] && [ -t 1 ] && exec tmux attach-session -t $SESSION_NAME 2>/dev/null || true")

    lines.extend([
        "cleanup_and_exit() {",
        "  if tmux has-session -t $SESSION_NAME 2>/dev/null; then",
        "    for pane in $(tmux list-panes -t $SESSION_NAME -F '#{window_index}.#{pane_index}' 2>/dev/null); do",
        "      tmux send-keys -t $SESSION_NAME:$pane C-c 2>/dev/null || true",
        "    done",
        "    sleep 2",
        "    tmux kill-session -t $SESSION_NAME 2>/dev/null || true",
        "  fi",
        "  exit 0",
        "}",
        "trap cleanup_and_exit SIGINT SIGTERM",
        "while tmux has-session -t $SESSION_NAME 2>/dev/null; do sleep 1; done",
    ])

    return base._write_temp_script('\n'.join(lines), 'tmux_launcher_')


def launch_in_tmux(context, launch_arguments_names: list[str], launcher_paths: list[str],
                   launcher_pkg_install_dir: str,
                   launcher_pkg_name: str = 'tier4_perception_launch',
                   session_name: str = 'ros2_launchers',
                   window_name: str = 'launchers',
                   launch_pointcloud_container: bool = False) -> list[ExecuteProcess]:
    """Launch multiple launchers in a single tmux session with split panes."""
    launch_args_cmd = base.format_launch_args_for_command(context, launch_arguments_names)
    script_path = create_tmux_launcher_script(
        launcher_paths, launch_args_cmd, launcher_pkg_install_dir,
        launcher_pkg_name, session_name, window_name, launch_pointcloud_container,
    )
    return [ExecuteProcess(cmd=['bash', script_path], output='screen')]
