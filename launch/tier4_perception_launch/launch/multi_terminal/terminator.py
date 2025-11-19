import os
import shlex
import tempfile
import textwrap
import uuid

from launch.actions import ExecuteProcess

from . import multi_terminal_launcher as base


def _build_process_supervision_functions(
    sigint_wait: int = base.TERMINATOR_SIGINT_WAIT_SECONDS,
    sigterm_wait: int = base.TERMINATOR_SIGTERM_WAIT_SECONDS,
    sigkill_wait: int = base.TERMINATOR_SIGKILL_WAIT_SECONDS,
) -> str:
    """Return bash helpers for supervising launcher child processes."""
    supervision_block = textwrap.dedent(
        """\
    child_processes_running() {
      for pid_file in $COMMAND_PID_FILES; do
        [ -f "$pid_file" ] || continue
        local child_pid=$(cat "$pid_file" 2>/dev/null)
        [ -n "$child_pid" ] || continue
        if kill -0 "$child_pid" 2>/dev/null; then
          return 0
        fi
      done
      return 1
    }

    get_container_pid() {
      [ -n "$POINTCLOUD_CONTAINER_PID_FILE" ] || return 1
      [ -f "$POINTCLOUD_CONTAINER_PID_FILE" ] || return 1
      local container_pid=$(cat "$POINTCLOUD_CONTAINER_PID_FILE" 2>/dev/null)
      [ -n "$container_pid" ] || return 1
      if kill -0 "$container_pid" 2>/dev/null; then
        echo "$container_pid"
        return 0
      fi
      return 1
    }

    container_is_running() {
      get_container_pid >/dev/null 2>&1
    }

    send_signal_to_children() {
      local sig=$1
      local sent_any=false
      for pid_file in $COMMAND_PID_FILES; do
        [ -f "$pid_file" ] || continue
        local child_pid=$(cat "$pid_file" 2>/dev/null)
        [ -n "$child_pid" ] || continue
        if kill -0 "$child_pid" 2>/dev/null; then
          sent_any=true
          kill -$sig "$child_pid" 2>/dev/null || true
        fi
      done
      [ "$sent_any" = true ]
    }

    wait_for_children_to_exit() {
      local timeout=${1:-0}
      local waited=0
      while child_processes_running; do
        if [ "$timeout" -gt 0 ] && [ "$waited" -ge "$timeout" ]; then
          return 1
        fi
        sleep 1
        waited=$((waited + 1))
      done
      return 0
    }

    wait_for_container_to_exit() {
      local timeout=${1:-0}
      local waited=0
      while container_is_running; do
        if [ "$timeout" -gt 0 ] && [ "$waited" -ge "$timeout" ]; then
          return 1
        fi
        sleep 1
        waited=$((waited + 1))
      done
      return 0
    }

    kill_container_directly() {
      if [ -z "$POINTCLOUD_CONTAINER_PID_FILE" ]; then
        return 1
      fi
      local container_pid=$(cat "$POINTCLOUD_CONTAINER_PID_FILE" 2>/dev/null)
      if [ -z "$container_pid" ]; then
        return 1
      fi
      if kill -0 "$container_pid" 2>/dev/null; then
        local sig=${1:-TERM}
        kill -$sig "$container_pid" 2>/dev/null || true
        return 0
      fi
      return 1
    }

    wait_for_container_death() {
      local timeout=${1:-5}
      local waited=0
      if [ -z "$POINTCLOUD_CONTAINER_PID_FILE" ]; then
        return 0
      fi
      local container_pid=$(cat "$POINTCLOUD_CONTAINER_PID_FILE" 2>/dev/null)
      if [ -z "$container_pid" ]; then
        return 0
      fi
      while [ "$waited" -lt "$timeout" ]; do
        if ! kill -0 "$container_pid" 2>/dev/null; then
          return 0
        fi
        sleep 1
        waited=$((waited + 1))
      done
      return 1
    }

    terminate_child_launchers() {
      [ -n "$COMMAND_PID_FILES" ] || return

      echo "Shutting down ROS2 launchers..."
      send_signal_to_children SIGINT
      sleep 3

      if child_processes_running; then
        send_signal_to_children SIGTERM
        if ! wait_for_children_to_exit 5; then
          send_signal_to_children SIGKILL
          wait_for_children_to_exit 2 || true
        fi
      fi

      echo "Terminating container..."
      if kill_container_directly TERM; then
        if ! wait_for_container_death 8; then
          kill_container_directly KILL
          wait_for_container_death 2 || true
        fi
      fi
    }
    """
    ).strip()

    return (
        supervision_block.replace("__SIGINT_WAIT__", str(sigint_wait))
        .replace("__SIGTERM_WAIT__", str(sigterm_wait))
        .replace("__SIGKILL_WAIT__", str(sigkill_wait))
    )


def _create_command_script(
    index: int, title: str, cmd: str, setup_bash_path: str
) -> tuple[str, str]:
    """Create a wrapper script for a command and return (script_path, pid_file)."""
    pid_fd, pid_file_path = tempfile.mkstemp(suffix=".pid", prefix=f"ros2_cmd_{index}_")
    os.close(pid_fd)

    script_content = "\n".join(
        [
            "#!/bin/bash",
            "set -x",
            f"PID_FILE={shlex.quote(pid_file_path)}",
            'echo $$ > "$PID_FILE"',
            f'echo "Starting launcher: {title} (PID $$)"',
            f"source {shlex.quote(setup_bash_path)}",
            "",
            f"exec {cmd}",
        ]
    )

    script_path = base._write_temp_script(script_content, f"ros2_cmd_{index}_")
    return script_path, pid_file_path


def _generate_terminator_layout(
    commands: list[tuple[str, str]], layout_name: str, script_paths: list[str] = None
) -> str:
    """Generate terminator layout config for given commands."""
    config_lines = [
        "[global_config]",
        "  suppress_multiple_term_dialog = True",
        "",
        "[keybindings]",
        "",
        "[profiles]",
        "  [[default]]",
        "",
    ]

    for i, _ in enumerate(commands, 1):
        profile_name = f"{layout_name}_profile_{i}"
        script_path = (
            script_paths[i - 1] if script_paths and i <= len(script_paths) else f"placeholder_{i}"
        )
        config_lines.extend(
            [
                f"  [[{profile_name}]]",
                "    use_custom_command = True",
                f"    custom_command = {shlex.quote(script_path)}",
                "",
            ]
        )

    config_lines.extend(
        [
            "[layouts]",
            f"  [[{layout_name}]]",
            "    [[[window0]]]",
            "      type = Window",
            '      parent = ""',
            "      size = 1200, 1400",
        ]
    )

    num_terminals = len(commands)
    if num_terminals == 0:
        config_lines.extend(
            [
                "    [[[child1]]]",
                "      type = Terminal",
                "      parent = window0",
                "      profile = default",
            ]
        )
    elif num_terminals == 1:
        title, _ = commands[0]
        config_lines.extend(
            [
                "    [[[child1]]]",
                "      type = Terminal",
                "      parent = window0",
                f"      profile = {layout_name}_profile_1",
                f"      title = {title}",
            ]
        )
    elif num_terminals == 2:
        config_lines.append("    [[[child1]]]")
        config_lines.append("      type = VPaned")
        config_lines.append("      parent = window0")
        config_lines.append("      ratio = 0.5")
        for i, (title, _) in enumerate(commands):
            config_lines.extend(
                [
                    f"    [[[[child{i + 2}]]]]",
                    "      type = Terminal",
                    "      parent = child1",
                    f"      profile = {layout_name}_profile_{i + 1}",
                    f"      title = {title}",
                    f"      order = {i}",
                ]
            )
    else:
        child_counter = 1
        parent_pane = None

        for i, (title, _) in enumerate(commands):
            profile_name = f"{layout_name}_profile_{i + 1}"
            is_first = i == 0
            is_last = i == num_terminals - 1

            # Calculate ratio for equal splits: first terminal gets 1/n, second split is 1/(n-1), etc.
            # For terminal i (0-indexed), ratio should be 1/(num_terminals - i)
            terminals_remaining = num_terminals - i
            split_ratio = 1.0 / terminals_remaining if terminals_remaining > 1 else 0.5

            if is_first:
                config_lines.extend(
                    [
                        f"    [[[child{child_counter}]]]",
                        "      type = VPaned",
                        "      parent = window0",
                        f"      ratio = {split_ratio:.6f}",
                        "      order = 0",
                    ]
                )
                parent_pane = f"child{child_counter}"
                child_counter += 1
                config_lines.extend(
                    [
                        f"    [[[child{child_counter}]]]",
                        "      type = Terminal",
                        f"      parent = {parent_pane}",
                        f"      profile = {profile_name}",
                        f"      title = {title}",
                        "      order = 0",
                    ]
                )
                child_counter += 1
            elif is_last:
                config_lines.extend(
                    [
                        f"    [[[child{child_counter}]]]",
                        "      type = Terminal",
                        f"      parent = {parent_pane}",
                        f"      profile = {profile_name}",
                        f"      title = {title}",
                        "      order = 1",
                    ]
                )
                child_counter += 1
            else:
                config_lines.extend(
                    [
                        f"    [[[child{child_counter}]]]",
                        "      type = VPaned",
                        f"      parent = {parent_pane}",
                        f"      ratio = {split_ratio:.6f}",
                        "      order = 1",
                    ]
                )
                parent_pane = f"child{child_counter}"
                child_counter += 1
                config_lines.extend(
                    [
                        f"    [[[child{child_counter}]]]",
                        "      type = Terminal",
                        f"      parent = {parent_pane}",
                        f"      profile = {profile_name}",
                        f"      title = {title}",
                        "      order = 0",
                    ]
                )
                child_counter += 1

    return "\n".join(config_lines)


def create_terminator_launcher_script(
    launcher_paths: list[str],
    launch_args_cmd: list[str],
    launcher_pkg_install_dir: str,
    launcher_pkg_name: str = "tier4_perception_launch",
    layout_name: str = "ros2_launchers",
    launch_pointcloud_container: bool = False,
    titles: list[str] = None,
) -> str:
    """Create a bash script that launches terminator with a layout and runs launchers."""
    setup_bash_path = base._get_workspace_setup_path(launcher_pkg_install_dir)
    session_group_id = f"autoware-multi-terminal-{uuid.uuid4()}"
    group_env = f"AUTOWARE_MULTI_TERMINAL_GROUP={session_group_id}"

    commands: list[tuple[str, str]] = []
    all_titles: list[str] = []
    escaped_args = [base.escape_dollar_signs_for_bash(arg) for arg in launch_args_cmd]

    if launch_pointcloud_container:
        cmd = base._build_command_string(
            [
                "env",
                group_env,
                "ros2",
                "run",
                "rclcpp_components",
                "component_container",
                "--ros-args",
                "-r",
                "__node:=pointcloud_container",
                "--log-level",
                "info",
            ],
            escape_dollars=False,
        )
        commands.append(("pointcloud_container", cmd))
        all_titles.append("pointcloud_container")

    for launcher_path in launcher_paths:
        launcher_name = os.path.basename(launcher_path)
        full_path = os.path.join(launcher_pkg_install_dir, launcher_path)
        cmd = base._build_command_string(
            ["env", group_env, "ros2", "launch", full_path] + escaped_args, escape_dollars=False
        )
        commands.append((launcher_name, cmd))
        all_titles.append(launcher_name)

    if titles:
        all_titles = titles

    total_panes = len(commands)

    command_scripts = [
        _create_command_script(i, title, cmd, setup_bash_path)
        for i, (title, cmd) in enumerate(commands, 1)
    ]

    pointcloud_container_pid_file = ""
    if launch_pointcloud_container and command_scripts:
        pointcloud_container_pid_file = command_scripts[0][1]

    script_paths = [path for path, _ in command_scripts]
    config_content = _generate_terminator_layout(commands, layout_name, script_paths)

    config_fd, config_path = tempfile.mkstemp(suffix="_terminator_config", prefix="ros2_launcher_")
    with os.fdopen(config_fd, "w") as f:
        f.write(config_content)

    script_lines = [
        "#!/bin/bash",
        "set -e",
        "",
        f"WORKSPACE_SETUP={shlex.quote(setup_bash_path)}",
        f"LAYOUT_NAME={shlex.quote(layout_name)}",
        f"SESSION_GROUP_ID={shlex.quote(session_group_id)}",
        "",
        "command -v terminator &> /dev/null || { echo 'ERROR: terminator not found' >&2; exit 1; }",
        "",
        f'echo "Launching terminator with {total_panes} panes"',
        "",
        'echo "Creating command scripts..."',
    ]

    for title, (script_path, pid_file) in zip(all_titles, command_scripts):
        script_lines.append(
            f'echo "Created script for {title}: {script_path} (PID file: {pid_file})"'
        )

    script_lines.extend(
        [
            f"TERMINATOR_CONFIG={shlex.quote(config_path)}",
            "",
            'echo "About to launch terminator..."',
            'TERMINATOR_PID=""',
            'terminator --config="$TERMINATOR_CONFIG" --layout="$LAYOUT_NAME" &',
            "TERMINATOR_PID=$!",
            'echo "Terminator launched with PID: $TERMINATOR_PID"',
            "sleep 2",
            'if kill -0 "$TERMINATOR_PID" 2>/dev/null; then',
            '  echo "Terminator is still running with PID: $TERMINATOR_PID"',
            "else",
            '  echo "ERROR: Terminator exited immediately (PID: $TERMINATOR_PID)"',
            "  exit 1",
            "fi",
            "",
            f"COMMAND_SCRIPTS=\"{' '.join(shlex.quote(p) for p, _ in command_scripts)}\"",
            f"COMMAND_PID_FILES=\"{' '.join(shlex.quote(p) for _, p in command_scripts)}\"",
            f"POINTCLOUD_CONTAINER_PID_FILE={shlex.quote(pointcloud_container_pid_file)}",
            "",
            "CLEANUP_IN_PROGRESS=false",
            "",
        ]
    )

    process_supervision_block = _build_process_supervision_functions()
    script_lines.extend(process_supervision_block.splitlines())
    script_lines.extend(
        [
            "",
            "cleanup_and_exit() {",
            '  if [ "$CLEANUP_IN_PROGRESS" = true ]; then',
            '    echo "Cleanup already in progress. Waiting for shutdown to finish..."',
            "    return",
            "  fi",
            "  CLEANUP_IN_PROGRESS=true",
            '  echo "Shutting down all processes..."',
            "  terminate_child_launchers",
            "  wait_for_children_to_exit 0 || true",
            '  if [ -n "$TERMINATOR_PID" ] && kill -0 "$TERMINATOR_PID" 2>/dev/null; then',
            '    echo "Terminating terminator (PID: $TERMINATOR_PID)..."',
            '    kill -TERM "$TERMINATOR_PID" 2>/dev/null || true',
            '    kill -TERM -"$TERMINATOR_PID" 2>/dev/null || true',
            "    sleep 1",
            '    if kill -0 "$TERMINATOR_PID" 2>/dev/null; then',
            '      kill -KILL "$TERMINATOR_PID" 2>/dev/null || true',
            '      kill -KILL -"$TERMINATOR_PID" 2>/dev/null || true',
            "    fi",
            "  fi",
            '  if [ -n "$COMMAND_PID_FILES" ]; then',
            "    for pid_file in $COMMAND_PID_FILES; do",
            '      rm -f "$pid_file" 2>/dev/null || true',
            "    done",
            "  fi",
            '  rm -f "$TERMINATOR_CONFIG" 2>/dev/null || true',
            "  for script in $COMMAND_SCRIPTS; do",
            '    rm -f "$script" 2>/dev/null || true',
            "  done",
            '  echo "Cleanup complete."',
            "  exit 0",
            "}",
            "trap cleanup_and_exit SIGINT SIGTERM",
            'echo "Launcher script is running. Press Ctrl+C to exit all sessions."',
            'while kill -0 "$TERMINATOR_PID" 2>/dev/null; do',
            "  sleep 2",
            "done",
            'echo "Terminator exited. Cleaning up..."',
            "cleanup_and_exit",
        ]
    )

    return base._write_temp_script("\n".join(script_lines), "terminator_launcher_")


def launch_in_terminator(
    context,
    launch_arguments_names: list[str],
    launcher_paths: list[str],
    launcher_pkg_install_dir: str,
    launcher_pkg_name: str = "tier4_perception_launch",
    layout_name: str = "ros2_launchers",
    launch_pointcloud_container: bool = False,
    titles: list[str] = None,
) -> list[ExecuteProcess]:
    """Launch multiple launchers in terminator with split panes using a layout file."""
    launch_args_cmd = base.format_launch_args_for_command(context, launch_arguments_names)
    script_path = create_terminator_launcher_script(
        launcher_paths,
        launch_args_cmd,
        launcher_pkg_install_dir,
        launcher_pkg_name,
        layout_name,
        launch_pointcloud_container,
        titles,
    )
    return [
        ExecuteProcess(
            cmd=["bash", script_path],
            output="screen",
            sigterm_timeout=str(base.TERMINAL_SIGTERM_TIMEOUT),
            sigkill_timeout=str(base.TERMINAL_SIGKILL_TIMEOUT),
        )
    ]
