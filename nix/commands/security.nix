# Security command wrappers (SBOM, vulnerability scanning, signing, PKI)
{ pkgs, ... }:

[
  (pkgs.writeShellScriptBin "sbom" ''
    set -e
    TARGET="''${1:-.}"
    FORMAT="''${2:-table}"
    if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
      echo "Usage: sbom [target] [format]"
      echo "Generate SBOM using syft"
      echo "Formats: table, json, spdx-json, cyclonedx-json"
      exit 0
    fi
    echo "Generating SBOM for: $TARGET (format: $FORMAT)" >&2
    syft "$TARGET" -o "$FORMAT"
  '')
  (pkgs.writeShellScriptBin "vuln-scan" ''
    set -e
    TARGET="''${1:-.}"
    TOOL="''${2:-grype}"
    if [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
      echo "Usage: vuln-scan [target] [tool]"
      echo "Scan for vulnerabilities (grype or trivy)"
      exit 0
    fi
    echo "Scanning: $TARGET with $TOOL" >&2
    case "$TOOL" in
      grype) grype "$TARGET" ;;
      trivy) trivy fs "$TARGET" ;;
      *) echo "Unknown tool: $TOOL" >&2; exit 1 ;;
    esac
  '')
  (pkgs.writeShellScriptBin "sign-artifact" ''
    set -e
    if [ -z "$1" ] || [ "$1" = "--help" ] || [ "$1" = "-h" ]; then
      echo "Usage: sign-artifact <target> [--key <keyfile>]"
      echo "Sign container images or files using cosign"
      exit 0
    fi
    TARGET="$1"
    shift
    if [ "$1" = "--key" ] && [ -n "$2" ]; then
      cosign sign --key "$2" "$TARGET"
    else
      echo "Using keyless signing (OIDC)" >&2
      cosign sign "$TARGET"
    fi
  '')
  (pkgs.writeShellScriptBin "pki-cert" ''
    case "''${1:-help}" in
      ca-init)
        echo "Initializing local CA..."
        step ca init --name "ROS2-Dev-CA" --provisioner admin --dns localhost --address ":9000"
        ;;
      create)
        [ -z "$2" ] && { echo "Usage: pki-cert create <name> [san...]" >&2; exit 1; }
        NAME="$2"
        shift 2
        step certificate create "$NAME" "$NAME.crt" "$NAME.key" --san "$NAME" "$@"
        echo "Created: $NAME.crt, $NAME.key"
        ;;
      inspect)
        [ -z "$2" ] && { echo "Usage: pki-cert inspect <cert>" >&2; exit 1; }
        step certificate inspect "$2"
        ;;
      *)
        echo "Usage: pki-cert [ca-init|create|inspect]"
        ;;
    esac
  '')
]
