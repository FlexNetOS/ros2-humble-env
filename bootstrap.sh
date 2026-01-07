#!/usr/bin/env bash
set -euo pipefail

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored messages
print_error() {
    echo -e "${RED}ERROR: $1${NC}" >&2
}

print_success() {
    echo -e "${GREEN}SUCCESS: $1${NC}"
}

print_info() {
    echo -e "${YELLOW}INFO: $1${NC}"
}

# Check if Nix is installed
check_nix() {
    if ! command -v nix &> /dev/null; then
        print_error "Nix is not installed or not in PATH"
        echo ""
        echo "To install Nix, run one of the following:"
        echo ""
        echo "1. Experimental installer (recommended):"
        echo "   curl --proto '=https' --tlsv1.2 -sSf -L https://artifacts.nixos.org/experimental-installer | sh -s -- install"
        echo ""
        echo "2. Official installer:"
        echo "   sh <(curl -L https://nixos.org/nix/install) --daemon"
        echo ""
        return 1
    fi
    
    # Check if flakes are enabled
    if ! nix eval --expr '1 + 1' &> /dev/null; then
        print_error "Nix is installed but flakes are not enabled"
        echo ""
        echo "To enable flakes, add the following to your Nix configuration:"
        echo "  experimental-features = nix-command flakes"
        echo ""
        echo "Location depends on your system:"
        echo "  - NixOS: /etc/nixos/configuration.nix"
        echo "  - macOS/Linux: ~/.config/nix/nix.conf or /etc/nix/nix.conf"
        echo ""
        return 1
    fi
    
    print_success "Nix is installed and flakes are enabled"
    return 0
}

# Check if Pixi will be available in the dev shell
check_pixi_in_shell() {
    print_info "Pixi will be provided by the Nix dev shell"
    return 0
}

# Main bootstrap process
main() {
    echo "=========================================="
    echo "  ROS2 Humble Environment Bootstrap"
    echo "=========================================="
    echo ""
    
    # Check prerequisites
    if ! check_nix; then
        exit 1
    fi
    
    check_pixi_in_shell
    
    echo ""
    print_info "Entering Nix dev shell and running pixi install..."
    echo ""
    
    # Enter the dev shell and run pixi install
    nix develop --command bash -c '
        set -euo pipefail
        
        # Verify pixi is available
        if ! command -v pixi &> /dev/null; then
            echo -e "'"${RED}"'ERROR: pixi is not available in the dev shell'"${NC}"'" >&2
            exit 1
        fi
        
        echo "Pixi version: $(pixi --version)"
        echo ""
        echo "Running pixi install..."
        pixi install
        
        echo ""
        echo -e "'"${GREEN}"'Bootstrap completed successfully!'"${NC}"'"
        echo ""
        echo "To enter the development environment, run:"
        echo "  nix develop"
        echo ""
        echo "Or use direnv for automatic activation:"
        echo "  direnv allow"
    '
    
    exit_code=$?
    
    if [ $exit_code -eq 0 ]; then
        echo ""
        print_success "Bootstrap completed successfully!"
    else
        echo ""
        print_error "Bootstrap failed with exit code $exit_code"
        exit $exit_code
    fi
}

# Run main function
main "$@"
