# Development workflow command wrappers
{ pkgs, ... }:

[
  (pkgs.writeShellScriptBin "fmt-nix" ''
    echo "Formatting Nix files..."
    find . -name "*.nix" -type f ! -path "./.git/*" -exec nixfmt {} +
    echo "Done."
  '')
  (pkgs.writeShellScriptBin "dev-check" ''
    set -e
    FIX=""
    [ "$1" = "--fix" ] && FIX="1"
    echo "Running development checks..."
    echo "=============================="
    FAILED=0

    echo ""
    echo "[1/4] Checking Nix formatting..."
    if [ -n "$FIX" ]; then
      find . -name "*.nix" -type f ! -path "./.git/*" -exec nixfmt {} + && echo "  Formatted."
    else
      if find . -name "*.nix" -type f ! -path "./.git/*" -exec nixfmt --check {} + 2>/dev/null; then
        echo "  OK"
      else
        echo "  FAIL - Run with --fix to auto-format"
        FAILED=1
      fi
    fi

    echo ""
    echo "[2/4] Checking Nix flake..."
    if command -v nix >/dev/null 2>&1; then
      if nix flake check --no-build 2>&1 | head -20; then
        echo "  OK"
      else
        echo "  FAIL"
        FAILED=1
      fi
    else
      echo "  SKIP (nix not available)"
    fi

    echo ""
    echo "[3/4] Checking pixi.lock..."
    if [ -f pixi.toml ]; then
      [ -f pixi.lock ] && echo "  OK" || { echo "  FAIL - Run 'pixi install'"; FAILED=1; }
    else
      echo "  SKIP (no pixi.toml)"
    fi

    echo ""
    echo "[4/4] Checking ROS2 packages..."
    if [ -d "src" ] && command -v colcon >/dev/null 2>&1; then
      if colcon build --packages-select-build-failed 2>&1 | grep -q "No packages"; then
        echo "  OK"
      else
        echo "  WARN - Some packages may have build issues"
      fi
    else
      echo "  SKIP (no src/ or colcon unavailable)"
    fi

    echo ""
    echo "=============================="
    if [ "$FAILED" -eq 0 ]; then
      echo "All checks passed!"
    else
      echo "Some checks failed."
      exit 1
    fi
  '')
  (pkgs.writeShellScriptBin "pre-commit" ''
    echo "Running pre-commit checks..."
    LARGE_FILES=$(git diff --cached --name-only | xargs -I{} sh -c 'test -f "{}" && du -k "{}" | awk "\$1 > 1024 {print \$2}"' 2>/dev/null)
    [ -n "$LARGE_FILES" ] && echo "Warning: Large files staged: $LARGE_FILES" >&2
    if git diff --cached | grep -E "(password|secret|api_key|private_key)\s*[:=]" >/dev/null 2>&1; then
      echo "Warning: Possible secrets detected" >&2
    fi
    STAGED_NIX=$(git diff --cached --name-only --diff-filter=ACM | grep '\.nix$' || true)
    if [ -n "$STAGED_NIX" ]; then
      echo "Formatting staged Nix files..."
      echo "$STAGED_NIX" | xargs nixfmt
      echo "$STAGED_NIX" | xargs git add
    fi
    echo "Pre-commit checks complete."
  '')
  (pkgs.writeShellScriptBin "swc" ''
    exec npx -y @swc/cli@latest "$@"
  '')
]
