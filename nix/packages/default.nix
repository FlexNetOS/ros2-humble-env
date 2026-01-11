# Package aggregator - imports all package modules
# Usage in flake.nix:
#   packages = import ./nix/packages { inherit pkgs lib; };
#   then use: packages.base, packages.devTools, packages.holochain, etc.
{ pkgs, lib, ... }:

let
  platform = import ./platform.nix { inherit pkgs lib; };
in
{
  # Base packages for fast shell startup
  base = import ./base.nix { inherit pkgs; };

  # Full development tools (optional extras)
  devTools = import ./dev-tools.nix { inherit pkgs; };

  # Holochain P2P packages (requires overlay applied to pkgs)
  holochain = import ./holochain.nix { inherit pkgs; };

  # Platform-specific packages
  linux = platform.linux;
  darwin = platform.darwin;
  platformAll = platform.all;

  # Convenience: all packages combined for default shell
  defaultShell = (import ./base.nix { inherit pkgs; })
    ++ (import ./holochain.nix { inherit pkgs; })
    ++ platform.all;

  # Convenience: all packages combined for full shell
  fullShell = (import ./base.nix { inherit pkgs; })
    ++ (import ./dev-tools.nix { inherit pkgs; })
    ++ (import ./holochain.nix { inherit pkgs; })
    ++ platform.all;
}
