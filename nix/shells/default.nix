# Shell definitions aggregator
# Usage in flake.nix:
#   shells = import ./nix/shells { inherit pkgs lib system colconDefaults; };
#   devShells.default = shells.default;
{ pkgs, lib, system, colconDefaults, packages, commands, ... }:

let
  inherit (pkgs.stdenv) isDarwin isLinux;

  common = import ./common.nix {
    inherit pkgs lib colconDefaults isDarwin system;
  };

  # Base shell hook
  baseHook = common.baseShellHook;
in
{
  # Default shell - minimal for fast startup
  default = pkgs.mkShell {
    packages = packages.defaultShell ++ commands.defaultShell;
    inherit (common.env) COLCON_DEFAULTS_FILE EDITOR VISUAL;

    shellHook = baseHook + common.defaultShellHook isDarwin system;
  };

  # Full shell - all tools included
  full = pkgs.mkShell {
    packages = packages.fullShell ++ commands.fullShell;
    inherit (common.env) COLCON_DEFAULTS_FILE EDITOR VISUAL;

    # Layer 3 Isolation Configuration (ARIA P0-001, P0-002)
    DEFAULT_ISOLATION = "firecracker";
    TOOL_ISOLATION = "sandbox-runtime";

    shellHook = baseHook + common.fullShellHook isDarwin system pkgs.python313.version;
  };

  # Identity shell - for Keycloak/Vaultwarden development
  identity = pkgs.mkShell {
    packages = packages.base ++ commands.core ++ packages.linux ++ (with pkgs; [
      keycloak
      vaultwarden
      postgresql_15
      sqlite
      jdk21_headless
      pgcli
    ]);
    inherit (common.env) COLCON_DEFAULTS_FILE EDITOR VISUAL;
    JAVA_HOME = "${pkgs.jdk21_headless}";

    shellHook = baseHook + ''
      echo ""
      echo "Identity & Auth Development Environment"
      echo "======================================="
      echo "  Keycloak, Vaultwarden, PostgreSQL"
      echo ""
    '';
  };
}
