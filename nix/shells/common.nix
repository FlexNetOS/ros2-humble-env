# Common shell configuration shared across all shells
{ pkgs, lib, colconDefaults, isDarwin, system, ... }:

{
  # Common environment variables
  env = {
    COLCON_DEFAULTS_FILE = toString colconDefaults;
    EDITOR = "hx";
    VISUAL = "hx";
  };

  # Base shell hook shared by all shells
  baseShellHook = ''
    # Ensure TMPDIR is valid (fix for Codespaces/devcontainers)
    export TMPDIR=''${TMPDIR:-/tmp}
    [ -d "$TMPDIR" ] || export TMPDIR=/tmp
    mkdir -p "$TMPDIR" 2>/dev/null || true

    # Define stub functions for RoboStack activation scripts
    noa_add_path() { :; }
    export -f noa_add_path 2>/dev/null || true

    # Initialize pixi environment
    if [ -f pixi.toml ]; then
      ${lib.optionalString isDarwin ''
        export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
      ''}
      eval "$(pixi shell-hook 2>/dev/null)" || true
    fi
  '';

  # Default shell hook (minimal output)
  defaultShellHook = isDarwin: system: ''
    # Keep startup fast for non-interactive shells (CI, `nix develop --command ...`).
    if [[ $- == *i* ]]; then
      echo ""
      echo "ROS2 Humble Development Environment"
      echo "=================================="
      echo "  Platform: ${if isDarwin then "macOS" else "Linux"} (${system})"
      echo ""
    fi
  '';

  # Full shell hook (verbose output)
  fullShellHook = isDarwin: system: pythonVersion: ''
    # Initialize direnv
    eval "$(direnv hook bash)"

    # Initialize zoxide
    eval "$(zoxide init bash)"

    # Initialize starship prompt
    eval "$(starship init bash)"

    # ROS2 environment info
    echo ""
    echo "ROS2 Humble Development Environment (Full)"
    echo "==========================================="
    echo "  Platform: ${if isDarwin then "macOS" else "Linux"} (${system})"
    echo "  Python (Nix): ${pythonVersion} (for scripts/tools)"
    echo "  Python (ROS2): 3.11.x via Pixi/RoboStack"
    echo ""
    echo "Quick commands:"
    echo "  cb          - colcon build --symlink-install"
    echo "  ct          - colcon test"
    echo "  ros2-clean  - Clean build artifacts"
    echo "  ros2-ws     - Show workspace info"
    echo ""
  '';
}
