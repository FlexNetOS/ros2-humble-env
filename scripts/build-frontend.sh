#!/usr/bin/env bash
# Build frontend assets using esbuild/swc
# BUILDKIT_STARTER_SPEC.md Layer: Build Tools (JavaScript/TypeScript)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Default configuration
BUILD_MODE="${BUILD_MODE:-production}"
OUT_DIR="${OUT_DIR:-$PROJECT_ROOT/dist}"
SRC_DIR="${SRC_DIR:-$PROJECT_ROOT/src}"

usage() {
    cat << EOF
Usage: $(basename "$0") [OPTIONS]

Build frontend assets using esbuild.

Options:
    --mode MODE     Build mode: development or production (default: production)
    --src DIR       Source directory (default: src/)
    --out DIR       Output directory (default: dist/)
    --watch         Watch mode for development
    --minify        Minify output (default in production)
    --sourcemap     Generate source maps
    --help          Show this help message

Examples:
    $(basename "$0")                           # Production build
    $(basename "$0") --mode development        # Development build
    $(basename "$0") --watch                   # Watch mode
    $(basename "$0") --mode production --minify --sourcemap
EOF
}

# Parse arguments
WATCH=false
MINIFY=false
SOURCEMAP=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --mode)
            BUILD_MODE="$2"
            shift 2
            ;;
        --src)
            SRC_DIR="$2"
            shift 2
            ;;
        --out)
            OUT_DIR="$2"
            shift 2
            ;;
        --watch)
            WATCH=true
            shift
            ;;
        --minify)
            MINIFY=true
            shift
            ;;
        --sourcemap)
            SOURCEMAP=true
            shift
            ;;
        --help)
            usage
            exit 0
            ;;
        *)
            log_error "Unknown option: $1"
            usage
            exit 1
            ;;
    esac
done

# Set defaults based on mode
if [[ "$BUILD_MODE" == "production" ]]; then
    MINIFY=true
    SOURCEMAP=true
fi

# Check for esbuild
if ! command -v esbuild &> /dev/null; then
    log_error "esbuild not found. Install with: pixi install"
    exit 1
fi

# Build command
BUILD_CMD="esbuild"

# Add source files
if [[ -d "$SRC_DIR" ]]; then
    BUILD_CMD="$BUILD_CMD $SRC_DIR/*.ts $SRC_DIR/*.tsx $SRC_DIR/*.js $SRC_DIR/*.jsx 2>/dev/null || true"
else
    log_warn "Source directory not found: $SRC_DIR"
    log_info "Creating example source structure..."
    mkdir -p "$SRC_DIR"
    cat > "$SRC_DIR/index.ts" << 'EXAMPLE'
// Example TypeScript entry point
console.log("FlexStack Frontend Loaded");

export function init() {
    console.log("Initializing...");
}

init();
EXAMPLE
    BUILD_CMD="$BUILD_CMD $SRC_DIR/index.ts"
fi

# Add output options
BUILD_CMD="$BUILD_CMD --bundle --outdir=$OUT_DIR"

# Add mode-specific options
if $MINIFY; then
    BUILD_CMD="$BUILD_CMD --minify"
fi

if $SOURCEMAP; then
    BUILD_CMD="$BUILD_CMD --sourcemap"
fi

if $WATCH; then
    BUILD_CMD="$BUILD_CMD --watch"
fi

# Platform options
BUILD_CMD="$BUILD_CMD --platform=browser --target=es2020"

# Create output directory
mkdir -p "$OUT_DIR"

log_info "Building frontend assets..."
log_info "Mode: $BUILD_MODE"
log_info "Source: $SRC_DIR"
log_info "Output: $OUT_DIR"
log_info "Command: $BUILD_CMD"

# Execute build
eval "$BUILD_CMD"

if [[ $? -eq 0 ]] && ! $WATCH; then
    log_info "Build complete!"
    log_info "Output files:"
    ls -la "$OUT_DIR"
fi
