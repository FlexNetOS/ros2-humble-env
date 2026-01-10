#!/bin/bash
# =============================================================================
# NATS JetStream Stream Initialization Script
# P2-002: Configure NATS JetStream for Message Persistence
# =============================================================================
# This script creates and configures JetStream streams for the agentic system.
# It should be run after NATS server is up and running.
#
# Usage:
#   ./init-jetstream.sh [--nats-url NATS_URL] [--user USER] [--password PASS]
#
# Environment Variables:
#   NATS_URL      - NATS server URL (default: nats://localhost:4222)
#   NATS_USER     - NATS username (default: admin)
#   NATS_PASSWORD - NATS password (required)
# =============================================================================

set -e
set -u
set -o pipefail

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
NATS_URL="${NATS_URL:-nats://localhost:4222}"
NATS_USER="${NATS_USER:-admin}"
NATS_PASSWORD="${NATS_PASSWORD:-}"

# Parse command line arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        --nats-url)
            NATS_URL="$2"
            shift 2
            ;;
        --user)
            NATS_USER="$2"
            shift 2
            ;;
        --password)
            NATS_PASSWORD="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [--nats-url URL] [--user USER] [--password PASS]"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Validate required parameters
if [ -z "$NATS_PASSWORD" ]; then
    echo "Error: NATS_PASSWORD is required"
    echo "Set via environment variable or --password flag"
    exit 1
fi

# -----------------------------------------------------------------------------
# Helper Functions
# -----------------------------------------------------------------------------
log_info() {
    echo "[INFO] $*"
}

log_success() {
    echo "[SUCCESS] $*"
}

log_error() {
    echo "[ERROR] $*" >&2
}

# Check if nats CLI is available
check_nats_cli() {
    if ! command -v nats &> /dev/null; then
        log_error "nats CLI not found. Please install it first."
        log_error "Install: https://github.com/nats-io/natscli#installation"
        exit 1
    fi
    log_info "nats CLI found: $(nats --version)"
}

# Wait for NATS server to be ready
wait_for_nats() {
    log_info "Waiting for NATS server at $NATS_URL..."
    local max_attempts=30
    local attempt=0

    while [ $attempt -lt $max_attempts ]; do
        if nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" server ping &> /dev/null; then
            log_success "NATS server is ready"
            return 0
        fi
        attempt=$((attempt + 1))
        sleep 2
    done

    log_error "NATS server is not responding after $max_attempts attempts"
    exit 1
}

# Create or update a stream
create_stream() {
    local stream_name="$1"
    local subjects="$2"
    local retention="$3"
    local max_age="$4"
    local max_msgs="$5"
    local max_bytes="$6"
    local replicas="${7:-1}"
    local discard="${8:-old}"

    log_info "Creating/updating stream: $stream_name"

    # Check if stream exists
    if nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
           stream info "$stream_name" &> /dev/null; then
        log_info "Stream $stream_name already exists, updating..."
        # Note: Update might fail if incompatible changes are made
        # In production, consider delete + recreate or use `nats stream edit`
    fi

    # Create/update stream
    nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
         stream add "$stream_name" \
         --subjects="$subjects" \
         --retention="$retention" \
         --max-age="$max_age" \
         --max-msgs="$max_msgs" \
         --max-bytes="$max_bytes" \
         --replicas="$replicas" \
         --storage=file \
         --discard="$discard" \
         --defaults \
         --no-allow-rollup \
         || log_error "Failed to create stream: $stream_name"

    log_success "Stream $stream_name configured"
}

# Create a consumer for a stream
create_consumer() {
    local stream_name="$1"
    local consumer_name="$2"
    local filter_subject="${3:-}"
    local ack_wait="${4:-30s}"
    local max_deliver="${5:-3}"
    local mode="${6:-pull}"  # pull or push

    log_info "Creating consumer: $consumer_name on stream $stream_name"

    # Check if consumer exists
    if nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
           consumer info "$stream_name" "$consumer_name" &> /dev/null; then
        log_info "Consumer $consumer_name already exists, skipping..."
        return 0
    fi

    # Build consumer creation command
    local cmd="nats --server=$NATS_URL --user=$NATS_USER --password=$NATS_PASSWORD \
               consumer add $stream_name $consumer_name \
               --ack=explicit \
               --ack-wait=$ack_wait \
               --max-deliver=$max_deliver \
               --deliver=all \
               --replay=instant"

    if [ "$mode" = "pull" ]; then
        cmd="$cmd --pull"
    fi

    if [ -n "$filter_subject" ]; then
        cmd="$cmd --filter=$filter_subject"
    fi

    # Execute command
    eval "$cmd" || log_error "Failed to create consumer: $consumer_name"

    log_success "Consumer $consumer_name configured"
}

# -----------------------------------------------------------------------------
# Main Execution
# -----------------------------------------------------------------------------
main() {
    log_info "Starting NATS JetStream initialization..."

    # Pre-flight checks
    check_nats_cli
    wait_for_nats

    # Create Streams
    log_info "===== Creating JetStream Streams ====="

    # WORKFLOWS Stream - WorkQueue retention for task distribution
    create_stream "WORKFLOWS" "workflows.>" "workqueue" "7d" "1000000" "10G" "1"

    # EVENTS Stream - Limits retention for event sourcing
    create_stream "EVENTS" "events.>" "limits" "30d" "10000000" "50G" "1"

    # LOGS Stream - Limits retention with size constraints
    create_stream "LOGS" "logs.>" "limits" "14d" "50000000" "100G" "1" "old"

    # COMMANDS Stream - WorkQueue with short retention for timely processing
    create_stream "COMMANDS" "commands.>" "workqueue" "1h" "100000" "1G" "1"

    # TELEMETRY Stream - Limits retention for metrics and traces
    create_stream "TELEMETRY" "telemetry.>" "limits" "7d" "100000000" "200G" "1" "old"

    # NOTIFICATIONS Stream - Interest retention for pub/sub
    create_stream "NOTIFICATIONS" "notifications.>" "interest" "24h" "1000000" "5G" "1"

    log_success "All streams created"

    # Create Consumers
    log_info "===== Creating JetStream Consumers ====="

    # Workflow consumers
    create_consumer "WORKFLOWS" "workflow-processor" "workflows.*" "30s" "3" "pull"

    # Event consumers
    create_consumer "EVENTS" "event-logger" "" "20s" "5" "push"
    create_consumer "EVENTS" "event-analytics" "events.metrics.>" "60s" "2" "pull"

    # Log consumers
    create_consumer "LOGS" "log-aggregator" "" "10s" "2" "pull"

    # Command consumers
    create_consumer "COMMANDS" "command-executor" "commands.*" "60s" "1" "pull"

    # Telemetry consumers
    create_consumer "TELEMETRY" "metrics-collector" "telemetry.metrics.>" "30s" "3" "pull"

    # Notification consumers
    create_consumer "NOTIFICATIONS" "notification-dispatcher" "" "15s" "5" "push"

    log_success "All consumers created"

    # Show summary
    log_info "===== JetStream Configuration Summary ====="
    nats --server="$NATS_URL" --user="$NATS_USER" --password="$NATS_PASSWORD" \
         stream list || log_error "Failed to list streams"

    log_success "JetStream initialization complete!"
    log_info "Stream configuration documented in: config/nats/jetstream.conf"
}

# Run main function
main "$@"
