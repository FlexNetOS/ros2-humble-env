#!/usr/bin/env bash
set -euo pipefail

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

print_error() { echo -e "${RED}ERROR: $1${NC}" >&2; }
print_success() { echo -e "${GREEN}✓ $1${NC}"; }
print_info() { echo -e "${YELLOW}INFO: $1${NC}"; }

echo "=========================================="
echo "  ROS2 Humble Environment Bootstrap"
echo "=========================================="
echo ""

# Check Nix
if ! command -v nix &> /dev/null; then
    print_error "Nix is not installed"
    echo ""
    echo "Install Nix with flakes enabled:"
    echo "  curl --proto '=https' --tlsv1.2 -sSf -L https://artifacts.nixos.org/experimental-installer | sh -s -- install"
    echo ""
    exit 1
fi

if ! nix eval --expr '1 + 1' &> /dev/null; then
    print_error "Nix flakes are not enabled"
    echo "Add to ~/.config/nix/nix.conf or /etc/nix/nix.conf:"
    echo "  experimental-features = nix-command flakes"
    exit 1
fi

print_success "Nix with flakes is installed"

# Check git
if ! command -v git &> /dev/null; then
    print_error "Git is not installed"
    echo ""
    echo "Install git:"
    echo "  - macOS: brew install git"
    echo "  - Ubuntu/Debian: sudo apt install git"
    echo "  - Fedora: sudo dnf install git"
    echo ""
    exit 1
fi

print_success "Git is installed"

# Check direnv (included in this repo via .envrc)
if command -v direnv &> /dev/null; then
    print_success "direnv is installed"
    print_info "After bootstrap, run: direnv allow"
else
    print_info "direnv not found - you'll need to use 'nom develop' manually"
    echo "Install direnv:"
    echo "  - macOS: brew install direnv"
    echo "  - Ubuntu/Debian: sudo apt install direnv"
    echo "Then add to your shell:"
    echo "  - bash: eval \"\$(direnv hook bash)\""
    echo "  - zsh: eval \"\$(direnv hook zsh)\""
fi

echo ""
print_info "Entering dev shell and running pixi install..."
echo ""

# Use nom develop for better output, fall back to nix develop
if command -v nom &> /dev/null; then
    DEVELOP_CMD="nom develop"
else
    DEVELOP_CMD="nix develop"
fi

$DEVELOP_CMD --command bash <<'EOF'
set -euo pipefail

if ! command -v pixi &> /dev/null; then
    echo "ERROR: pixi not available in dev shell" >&2
    exit 1
fi

echo "Running pixi install..."
pixi install

echo ""
echo "✓ Bootstrap complete!"
EOF

if [ $? -eq 0 ]; then
    echo ""
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "  Next Steps"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo ""
    if command -v direnv &> /dev/null; then
        echo "1. Run: direnv allow"
        echo "2. Environment will activate automatically"
    else
        echo "1. Run: nom develop (or nix develop)"
        echo "2. Or install direnv for auto-activation"
    fi
    echo ""
    print_success "Setup complete!"
else
    print_error "Bootstrap failed"
    exit 1
fi
