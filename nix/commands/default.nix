# Command wrappers aggregator - imports all command modules
# Usage in flake.nix:
#   commands = import ./nix/commands { inherit pkgs; };
#   then use: commands.core, commands.ai, commands.ros2, etc.
{ pkgs, ... }:

{
  # Core commands (cb, ct, ctr, ros2-env, update-deps)
  core = import ./core.nix { inherit pkgs; };

  # AI and tool execution commands (localai, agixt, aios, sandbox-runtime, etc.)
  ai = import ./ai.nix { inherit pkgs; };

  # ROS2-specific commands (ros2-clean, ros2-ws, ros2-topics, ros2-nodes)
  ros2 = import ./ros2.nix { inherit pkgs; };

  # Infrastructure commands (ipfs-ctl, nats-ctl, prom-ctl)
  infra = import ./infra.nix { inherit pkgs; };

  # Security commands (sbom, vuln-scan, sign-artifact, pki-cert)
  security = import ./security.nix { inherit pkgs; };

  # Development workflow commands (fmt-nix, dev-check, pre-commit, swc)
  dev = import ./dev.nix { inherit pkgs; };

  # Workflow management commands (gh-issues, db-query, temporal-ctl, n8n-ctl)
  workflow = import ./workflow.nix { inherit pkgs; };

  # Convenience: all commands combined for default shell
  defaultShell = (import ./core.nix { inherit pkgs; })
    ++ (import ./ros2.nix { inherit pkgs; })
    ++ (import ./dev.nix { inherit pkgs; });

  # Convenience: all commands combined for full shell
  fullShell = (import ./core.nix { inherit pkgs; })
    ++ (import ./ai.nix { inherit pkgs; })
    ++ (import ./ros2.nix { inherit pkgs; })
    ++ (import ./infra.nix { inherit pkgs; })
    ++ (import ./security.nix { inherit pkgs; })
    ++ (import ./dev.nix { inherit pkgs; })
    ++ (import ./workflow.nix { inherit pkgs; });
}
