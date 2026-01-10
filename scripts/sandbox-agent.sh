#!/usr/bin/env bash
# Sandbox Agent Executor
# P2-001: Integrate sandbox-runtime for Untrusted Agents
#
# This script executes AI agent code in sandboxed environments based on risk level.
# It loads risk-specific profiles and uses Docker containers for isolation.
#
# Usage:
#   ./sandbox-agent.sh --risk-level <low|medium|high> --language <lang> --code <file> [options]
#
# Examples:
#   ./sandbox-agent.sh --risk-level low --language python --code agent.py
#   ./sandbox-agent.sh --risk-level high --language bash --code script.sh
#   ./sandbox-agent.sh --risk-level medium --language node --code app.js --agent-id my-agent

set -euo pipefail

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
CONFIG_DIR="$PROJECT_ROOT/config/sandbox"
PROFILES_DIR="$CONFIG_DIR/profiles"
LOG_DIR="/var/log/sandbox"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Logging functions
log_info() {
    echo -e "${GREEN}[INFO]${NC} $*" >&2
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $*" >&2
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $*" >&2
}

log_debug() {
    if [[ "${DEBUG:-false}" == "true" ]]; then
        echo -e "${BLUE}[DEBUG]${NC} $*" >&2
    fi
}

# Help text
show_help() {
    cat << EOF
Sandbox Agent Executor - Secure AI Agent Code Execution

Usage: $0 --risk-level <level> --language <lang> --code <file> [options]

Required Arguments:
  --risk-level <level>    - Risk level: low|medium|high
  --language <lang>       - Programming language: python|bash|node|rust|go
  --code <file>           - Code file to execute

Optional Arguments:
  --agent-id <id>         - Agent identifier for logging
  --profile <file>        - Custom profile YAML (overrides risk-level)
  --timeout <seconds>     - Override profile timeout
  --memory <size>         - Override profile memory limit
  --cpu <cores>           - Override profile CPU limit
  --env <KEY=VALUE>       - Additional environment variable
  --volume <host:cont>    - Mount volume (use with extreme caution)
  --output-file <file>    - Save output to file
  --log-level <level>     - Log level: trace|debug|info|warn|error
  --dry-run               - Show configuration without executing
  --debug                 - Enable debug output
  --help                  - Show this help message

Risk Levels:
  low       - Trusted agents, network access, standard resources
  medium    - Untrusted agents, no network, limited filesystem
  high      - Maximum isolation, minimal resources, heavy monitoring

Examples:
  # Execute low-risk Python agent
  $0 --risk-level low --language python --code agent.py

  # Execute high-risk bash script with custom agent ID
  $0 --risk-level high --language bash --code script.sh --agent-id malicious-test

  # Medium-risk Node.js with custom timeout
  $0 --risk-level medium --language node --code app.js --timeout 180

  # Dry run to see configuration
  $0 --risk-level high --language python --code test.py --dry-run

Security Features:
  - Risk-based profile selection
  - YAML configuration for reproducibility
  - Resource isolation (CPU, memory, network, filesystem)
  - Comprehensive logging and monitoring
  - Rate limiting and abuse prevention
  - Integration with sandbox-wrapper.sh

EOF
}

# Parse YAML profile (simple parser for our use case)
parse_yaml_value() {
    local file="$1"
    local key="$2"
    local default="${3:-}"

    if [[ ! -f "$file" ]]; then
        echo "$default"
        return
    fi

    # Simple YAML value extraction (handles key: value format)
    local value
    value=$(grep -E "^\\s*${key}:" "$file" | head -1 | sed "s/^\\s*${key}:\\s*//" | sed 's/#.*//' | tr -d '"' | tr -d "'" | xargs)

    if [[ -n "$value" ]]; then
        echo "$value"
    else
        echo "$default"
    fi
}

# Load profile configuration
load_profile() {
    local risk_level="$1"
    local profile_file="$PROFILES_DIR/${risk_level}-risk.yaml"

    if [[ ! -f "$profile_file" ]]; then
        log_error "Profile not found: $profile_file"
        log_error "Available profiles: low-risk, medium-risk, high-risk"
        exit 1
    fi

    log_info "Loading profile: $profile_file"

    # Parse profile values
    PROFILE_NAME=$(parse_yaml_value "$profile_file" "name" "$risk_level")
    PROFILE_MEMORY=$(parse_yaml_value "$profile_file" "memory" "256m")
    PROFILE_CPU=$(parse_yaml_value "$profile_file" "cpu" "0.5")
    PROFILE_TIMEOUT=$(parse_yaml_value "$profile_file" "timeout" "60")
    PROFILE_NETWORK=$(parse_yaml_value "$profile_file" "mode" "none")
    PROFILE_USER=$(parse_yaml_value "$profile_file" "user" "65534:65534")

    log_debug "Profile: $PROFILE_NAME"
    log_debug "Memory: $PROFILE_MEMORY"
    log_debug "CPU: $PROFILE_CPU"
    log_debug "Timeout: ${PROFILE_TIMEOUT}s"
    log_debug "Network: $PROFILE_NETWORK"
    log_debug "User: $PROFILE_USER"
}

# Validate inputs
validate_inputs() {
    # Check if code file exists
    if [[ ! -f "$CODE_FILE" ]]; then
        log_error "Code file not found: $CODE_FILE"
        exit 1
    fi

    # Check if sandbox-wrapper.sh exists
    if [[ ! -x "$SCRIPT_DIR/sandbox-wrapper.sh" ]]; then
        log_error "sandbox-wrapper.sh not found or not executable: $SCRIPT_DIR/sandbox-wrapper.sh"
        exit 1
    fi

    # Validate risk level
    case "$RISK_LEVEL" in
        low|medium|high)
            ;;
        *)
            log_error "Invalid risk level: $RISK_LEVEL (must be: low, medium, or high)"
            exit 1
            ;;
    esac

    # Validate language
    case "$LANGUAGE" in
        python|bash|node|rust|go)
            ;;
        *)
            log_error "Invalid language: $LANGUAGE (must be: python, bash, node, rust, or go)"
            exit 1
            ;;
    esac
}

# Build sandbox command
build_sandbox_command() {
    local cmd=("$SCRIPT_DIR/sandbox-wrapper.sh")

    # Add language and code file
    cmd+=("$LANGUAGE" "$CODE_FILE")

    # Add profile-based settings (can be overridden by user)
    cmd+=("--timeout" "${OVERRIDE_TIMEOUT:-$PROFILE_TIMEOUT}")
    cmd+=("--memory" "${OVERRIDE_MEMORY:-$PROFILE_MEMORY}")
    cmd+=("--cpu" "${OVERRIDE_CPU:-$PROFILE_CPU}")
    cmd+=("--network" "$PROFILE_NETWORK")
    cmd+=("--user" "$PROFILE_USER")
    cmd+=("--readonly")

    # Add custom environment variables
    for env in "${ENV_VARS[@]}"; do
        cmd+=("--env" "$env")
    done

    # Add volumes (with warning for high-risk)
    if [[ ${#VOLUMES[@]} -gt 0 ]]; then
        if [[ "$RISK_LEVEL" == "high" ]]; then
            log_warn "Mounting volumes in high-risk mode is not recommended"
            read -r -p "Continue? (y/N) " response
            if [[ ! "$response" =~ ^[Yy]$ ]]; then
                log_info "Aborted by user"
                exit 0
            fi
        fi
        for vol in "${VOLUMES[@]}"; do
            cmd+=("--volume" "$vol")
        done
    fi

    echo "${cmd[@]}"
}

# Execute agent in sandbox
execute_agent() {
    local start_time
    start_time=$(date +%s)
    local timestamp
    timestamp=$(date +%Y%m%d_%H%M%S)

    # Create log directory
    mkdir -p "$LOG_DIR"

    # Build log filename
    local log_file="$LOG_DIR/${RISK_LEVEL}-risk_${AGENT_ID}_${timestamp}.log"

    log_info "Executing agent in $RISK_LEVEL-risk sandbox"
    log_info "Agent ID: $AGENT_ID"
    log_info "Language: $LANGUAGE"
    log_info "Code file: $CODE_FILE"
    log_info "Log file: $log_file"

    # Build command
    local sandbox_cmd
    sandbox_cmd=$(build_sandbox_command)

    if [[ "$DRY_RUN" == "true" ]]; then
        log_info "DRY RUN - Command that would be executed:"
        echo "$sandbox_cmd"
        return 0
    fi

    # Execute and capture output
    log_info "Starting execution..."

    local exit_code=0
    if [[ -n "$OUTPUT_FILE" ]]; then
        # Save output to file
        bash -c "$sandbox_cmd" 2>&1 | tee "$log_file" "$OUTPUT_FILE" || exit_code=$?
    else
        # Output to stdout and log
        bash -c "$sandbox_cmd" 2>&1 | tee "$log_file" || exit_code=$?
    fi

    local end_time
    end_time=$(date +%s)
    local duration=$((end_time - start_time))

    # Log execution summary
    {
        echo "==================================="
        echo "Execution Summary"
        echo "==================================="
        echo "Agent ID: $AGENT_ID"
        echo "Risk Level: $RISK_LEVEL"
        echo "Language: $LANGUAGE"
        echo "Code File: $CODE_FILE"
        echo "Exit Code: $exit_code"
        echo "Duration: ${duration}s"
        echo "Timestamp: $(date)"
        echo "==================================="
    } >> "$log_file"

    if [[ $exit_code -eq 0 ]]; then
        log_info "Execution completed successfully in ${duration}s"
    else
        log_error "Execution failed with exit code $exit_code (duration: ${duration}s)"
    fi

    return "$exit_code"
}

# Parse arguments
parse_args() {
    RISK_LEVEL=""
    LANGUAGE=""
    CODE_FILE=""
    AGENT_ID="agent-$(date +%s)"
    CUSTOM_PROFILE=""
    ENV_VARS=()
    VOLUMES=()
    OUTPUT_FILE=""
    OVERRIDE_TIMEOUT=""
    OVERRIDE_MEMORY=""
    OVERRIDE_CPU=""
    LOG_LEVEL="info"
    DRY_RUN="false"
    DEBUG="false"

    while [[ $# -gt 0 ]]; do
        case $1 in
            --risk-level)
                RISK_LEVEL="$2"
                shift 2
                ;;
            --language)
                LANGUAGE="$2"
                shift 2
                ;;
            --code)
                CODE_FILE="$2"
                shift 2
                ;;
            --agent-id)
                AGENT_ID="$2"
                shift 2
                ;;
            --profile)
                CUSTOM_PROFILE="$2"
                shift 2
                ;;
            --timeout)
                OVERRIDE_TIMEOUT="$2"
                shift 2
                ;;
            --memory)
                OVERRIDE_MEMORY="$2"
                shift 2
                ;;
            --cpu)
                OVERRIDE_CPU="$2"
                shift 2
                ;;
            --env)
                ENV_VARS+=("$2")
                shift 2
                ;;
            --volume)
                VOLUMES+=("$2")
                shift 2
                ;;
            --output-file)
                OUTPUT_FILE="$2"
                shift 2
                ;;
            --log-level)
                LOG_LEVEL="$2"
                shift 2
                ;;
            --dry-run)
                DRY_RUN="true"
                shift
                ;;
            --debug)
                DEBUG="true"
                shift
                ;;
            --help)
                show_help
                exit 0
                ;;
            *)
                log_error "Unknown option: $1"
                show_help
                exit 1
                ;;
        esac
    done

    # Validate required arguments
    if [[ -z "$RISK_LEVEL" ]] || [[ -z "$LANGUAGE" ]] || [[ -z "$CODE_FILE" ]]; then
        log_error "Missing required arguments"
        show_help
        exit 1
    fi
}

# Main execution
main() {
    parse_args "$@"
    validate_inputs
    load_profile "$RISK_LEVEL"
    execute_agent
}

# Run main function
main "$@"
