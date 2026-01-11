# Platform-specific packages (Linux and macOS)
{ pkgs, lib, ... }:

let
  inherit (pkgs.stdenv) isDarwin isLinux;
in
{
  # Linux-specific packages
  linux = lib.optionals isLinux (with pkgs; [
    inotify-tools
    strace
    gdb

    # Container security (sandboxing untrusted ROS2 packages)
    # Usage: docker run --runtime=runsc ...
    # See docs/GITHUB-RESOURCES.md for gVisor integration guide
    gvisor              # OCI runtime for container sandboxing
  ]);

  # macOS-specific packages
  darwin = lib.optionals isDarwin (with pkgs; [
    coreutils
    gnused
    gawk
  ]);

  # Combined (for convenience)
  all = lib.optionals isLinux (with pkgs; [
    inotify-tools
    strace
    gdb
    gvisor
  ]) ++ lib.optionals isDarwin (with pkgs; [
    coreutils
    gnused
    gawk
  ]);
}
