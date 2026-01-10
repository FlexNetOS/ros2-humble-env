#!/usr/bin/env bash
# Setup automated certificate rotation via cron
# This script installs a cron job to run certificate rotation daily

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ROTATE_SCRIPT="$SCRIPT_DIR/rotate-certs.sh"

echo "🔄 Setting up Certificate Rotation Cron Job"
echo "==========================================="
echo "Project: $PROJECT_ROOT"
echo "Script: $ROTATE_SCRIPT"
echo ""

# Check if rotate-certs.sh exists
if [ ! -f "$ROTATE_SCRIPT" ]; then
    echo "❌ Error: rotate-certs.sh not found at $ROTATE_SCRIPT"
    exit 1
fi

# Make sure script is executable
chmod +x "$ROTATE_SCRIPT"

# Define cron schedule (default: daily at 2 AM)
CRON_SCHEDULE="${CRON_SCHEDULE:-0 2 * * *}"

# Create cron job command
CRON_COMMAND="cd $PROJECT_ROOT && $ROTATE_SCRIPT >> $PROJECT_ROOT/logs/cert-rotation.log 2>&1"

# Check if cron job already exists
if crontab -l 2>/dev/null | grep -q "$ROTATE_SCRIPT"; then
    echo "⚠️  Cron job already exists"
    echo ""
    echo "Current crontab:"
    crontab -l | grep "$ROTATE_SCRIPT"
    echo ""
    read -p "Do you want to update it? [y/N]: " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo "Aborted."
        exit 0
    fi

    # Remove existing cron job
    crontab -l | grep -v "$ROTATE_SCRIPT" | crontab -
    echo "✓ Removed existing cron job"
fi

# Add new cron job
(crontab -l 2>/dev/null; echo "$CRON_SCHEDULE $CRON_COMMAND") | crontab -

echo "✅ Cron job installed successfully!"
echo ""
echo "📋 Cron Schedule:"
echo "   Schedule: $CRON_SCHEDULE (daily at 2 AM)"
echo "   Command:  $CRON_COMMAND"
echo ""
echo "📝 To view your crontab:"
echo "   crontab -l"
echo ""
echo "📝 To remove the cron job:"
echo "   crontab -l | grep -v '$ROTATE_SCRIPT' | crontab -"
echo ""
echo "📝 To test certificate rotation manually:"
echo "   $ROTATE_SCRIPT"
echo ""
echo "📝 To view rotation logs:"
echo "   tail -f $PROJECT_ROOT/logs/cert-rotation.log"
echo ""

# Create systemd timer as an alternative (optional)
if command -v systemctl &> /dev/null; then
    echo "💡 Alternative: Use systemd timer instead of cron"
    echo ""
    cat > /tmp/cert-rotation.service <<EOF
[Unit]
Description=ARIA Certificate Rotation
After=network.target docker.service

[Service]
Type=oneshot
User=$USER
WorkingDirectory=$PROJECT_ROOT
ExecStart=$ROTATE_SCRIPT
StandardOutput=append:$PROJECT_ROOT/logs/cert-rotation.log
StandardError=append:$PROJECT_ROOT/logs/cert-rotation.log
EOF

    cat > /tmp/cert-rotation.timer <<EOF
[Unit]
Description=ARIA Certificate Rotation Timer
Requires=cert-rotation.service

[Timer]
OnCalendar=daily
Persistent=true

[Install]
WantedBy=timers.target
EOF

    echo "📝 Systemd service unit created at: /tmp/cert-rotation.service"
    echo "📝 Systemd timer unit created at: /tmp/cert-rotation.timer"
    echo ""
    echo "To install systemd timer:"
    echo "   sudo cp /tmp/cert-rotation.{service,timer} /etc/systemd/system/"
    echo "   sudo systemctl daemon-reload"
    echo "   sudo systemctl enable cert-rotation.timer"
    echo "   sudo systemctl start cert-rotation.timer"
    echo "   sudo systemctl status cert-rotation.timer"
    echo ""
fi

exit 0
