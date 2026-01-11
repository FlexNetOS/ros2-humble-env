# Infrastructure command wrappers (IPFS, NATS, Prometheus)
{ pkgs, ... }:

[
  (pkgs.writeShellScriptBin "ipfs-ctl" ''
    IPFS_PATH="''${IPFS_PATH:-$HOME/.ipfs}"
    case "''${1:-status}" in
      init)
        if [ -d "$IPFS_PATH" ]; then
          echo "IPFS already initialized at $IPFS_PATH"
        else
          echo "Initializing IPFS node..."
          ipfs init
        fi
        ;;
      start)
        if [ ! -d "$IPFS_PATH" ]; then
          echo "IPFS not initialized. Run: ipfs-ctl init" >&2
          exit 1
        fi
        echo "Starting IPFS daemon..."
        echo "  API: http://127.0.0.1:5001"
        echo "  Gateway: http://127.0.0.1:8080"
        exec ipfs daemon "''${@:2}"
        ;;
      stop)
        pkill -f "ipfs daemon" && echo "IPFS daemon stopped" || echo "Not running"
        ;;
      status)
        if pgrep -f "ipfs daemon" > /dev/null; then
          echo "IPFS daemon is running"
          ipfs id 2>/dev/null | head -5
        else
          echo "IPFS daemon is not running"
        fi
        ;;
      id)
        ipfs id
        ;;
      *)
        echo "Usage: ipfs-ctl [init|start|stop|status|id]"
        ;;
    esac
  '')
  (pkgs.writeShellScriptBin "nats-ctl" ''
    NATS_PORT="''${NATS_PORT:-4222}"
    NATS_HTTP_PORT="''${NATS_HTTP_PORT:-8222}"
    case "''${1:-status}" in
      start)
        echo "Starting NATS server..."
        echo "  Client: nats://127.0.0.1:$NATS_PORT"
        exec nats-server -p "$NATS_PORT" -m "$NATS_HTTP_PORT" "''${@:2}"
        ;;
      stop)
        pkill -f "nats-server" && echo "NATS server stopped" || echo "Not running"
        ;;
      status)
        if pgrep -f "nats-server" > /dev/null; then
          echo "NATS server is running"
          curl -s "http://127.0.0.1:$NATS_HTTP_PORT/varz" 2>/dev/null | jq -r '.server_id // "Connected"' || true
        else
          echo "NATS server is not running"
        fi
        ;;
      pub)
        [ -z "$2" ] || [ -z "$3" ] && { echo "Usage: nats-ctl pub <subject> <message>" >&2; exit 1; }
        nats pub "$2" "$3"
        ;;
      sub)
        [ -z "$2" ] && { echo "Usage: nats-ctl sub <subject>" >&2; exit 1; }
        nats sub "$2"
        ;;
      *)
        echo "Usage: nats-ctl [start|stop|status|pub|sub]"
        ;;
    esac
  '')
  (pkgs.writeShellScriptBin "prom-ctl" ''
    PROM_PORT="''${PROM_PORT:-9090}"
    PROM_CONFIG="''${PROM_CONFIG:-prometheus.yml}"
    PROM_DATA="''${PROM_DATA:-./prometheus-data}"
    case "''${1:-status}" in
      start)
        if [ ! -f "$PROM_CONFIG" ]; then
          cat > "$PROM_CONFIG" << 'PROMCFG'
global:
  scrape_interval: 15s
scrape_configs:
  - job_name: 'prometheus'
    static_configs:
      - targets: ['localhost:9090']
PROMCFG
        fi
        mkdir -p "$PROM_DATA"
        echo "Starting Prometheus on port $PROM_PORT..."
        exec prometheus --config.file="$PROM_CONFIG" --storage.tsdb.path="$PROM_DATA" --web.listen-address=":$PROM_PORT" "''${@:2}"
        ;;
      stop)
        pkill -f "prometheus.*--config.file" && echo "Prometheus stopped" || echo "Not running"
        ;;
      status)
        if pgrep -f "prometheus.*--config.file" > /dev/null; then
          echo "Prometheus is running"
          curl -s "http://127.0.0.1:$PROM_PORT/-/ready" && echo " - Ready"
        else
          echo "Prometheus is not running"
        fi
        ;;
      config)
        [ -f "$PROM_CONFIG" ] && cat "$PROM_CONFIG" || echo "Config not found: $PROM_CONFIG"
        ;;
      *)
        echo "Usage: prom-ctl [start|stop|status|config]"
        ;;
    esac
  '')
]
