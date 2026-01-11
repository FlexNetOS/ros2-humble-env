# Base packages for fast shell startup (direnv/nix develop)
# Keep this minimal - heavier tools go in dev-tools.nix
{ pkgs, ... }:

with pkgs; [
  pixi
  git
  gh
  jujutsu              # P2-013: Modern Git-compatible VCS (jj command)

  # Python 3.13.x - Latest stable (for non-ROS2 tools)
  # ROS2 uses Python 3.11 via Pixi/RoboStack (separate environment)
  python313
  python313Packages.pip
  python313Packages.virtualenv

  # Nix tools
  nix-output-monitor
  nix-tree
  nixfmt-rfc-style
  nil

  # Useful basics
  curl
  jq
  direnv
  nix-direnv
]
