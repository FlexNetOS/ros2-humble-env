# AI and tool execution command wrappers
# Includes: LocalAI, AGiXT, AIOS, sandbox-runtime, MCP tools
{ pkgs, ... }:

[
  (pkgs.writeShellScriptBin "ai" ''
    exec aichat "$@"
  '')
  (pkgs.writeShellScriptBin "pair" ''
    # aider-chat is the nixpkgs package name for aider
    if command -v aider-chat >/dev/null 2>&1; then
      exec aider-chat "$@"
    elif command -v aider >/dev/null 2>&1; then
      exec aider "$@"
    else
      echo "Neither 'aider-chat' nor 'aider' is available in PATH" >&2
      echo "Install via: nix develop .#full (includes aider-chat)" >&2
      exit 127
    fi
  '')
  (pkgs.writeShellScriptBin "promptfoo" ''
    exec npx promptfoo@latest "$@"
  '')
  (pkgs.writeShellScriptBin "localai" ''
    LOCALAI_MODELS="''${LOCALAI_MODELS_PATH:-$HOME/.local/share/localai/models}"
    mkdir -p "$LOCALAI_MODELS"
    case "''${1:-start}" in
      start)
        echo "Starting LocalAI on port 8080..."
        exec local-ai --models-path "$LOCALAI_MODELS" --address ":8080" "''${@:2}"
        ;;
      stop)
        pkill -f "local-ai" && echo "LocalAI stopped" || echo "LocalAI not running"
        ;;
      status)
        if pgrep -f "local-ai" > /dev/null; then
          echo "LocalAI is running"
          curl -s http://localhost:8080/readyz && echo " - API ready"
        else
          echo "LocalAI is not running"
        fi
        ;;
      models)
        echo "Available models in $LOCALAI_MODELS:"
        ls -la "$LOCALAI_MODELS" 2>/dev/null || echo "  (none)"
        ;;
      *)
        echo "Usage: localai [start|stop|status|models]"
        ;;
    esac
  '')
  (pkgs.writeShellScriptBin "prompt-cache" ''
    PROMPTCACHE_DIR="''${PROMPTCACHE_DIR:-$HOME/.local/share/prompt-cache}"
    PROMPTCACHE_PORT="''${PROMPTCACHE_PORT:-8080}"
    case "''${1:-status}" in
      install)
        mkdir -p "$PROMPTCACHE_DIR"
        if [ ! -d "$PROMPTCACHE_DIR/prompt-cache" ]; then
          git clone https://github.com/messkan/prompt-cache.git "$PROMPTCACHE_DIR/prompt-cache"
          echo "PromptCache installed to: $PROMPTCACHE_DIR/prompt-cache"
        else
          echo "PromptCache already installed"
        fi
        ;;
      up)
        if [ ! -d "$PROMPTCACHE_DIR/prompt-cache" ]; then
          echo "PromptCache not installed. Run: prompt-cache install"
          exit 1
        fi
        cd "$PROMPTCACHE_DIR/prompt-cache"
        if [ -f "docker-compose.yml" ]; then
          docker compose up -d "''${@:2}"
        else
          make run &
        fi
        ;;
      down)
        if [ -f "$PROMPTCACHE_DIR/prompt-cache/docker-compose.yml" ]; then
          cd "$PROMPTCACHE_DIR/prompt-cache" && docker compose down "''${@:2}"
        else
          pkill -f "prompt-cache" && echo "PromptCache stopped" || echo "Not running"
        fi
        ;;
      logs)
        if [ -f "$PROMPTCACHE_DIR/prompt-cache/docker-compose.yml" ]; then
          cd "$PROMPTCACHE_DIR/prompt-cache" && docker compose logs -f "''${@:2}"
        fi
        ;;
      status)
        if pgrep -f "prompt-cache" > /dev/null || docker ps 2>/dev/null | grep -q "prompt-cache"; then
          echo "PromptCache is running on port $PROMPTCACHE_PORT"
        else
          echo "PromptCache is not running"
        fi
        ;;
      *)
        echo "Usage: prompt-cache [install|up|down|logs|status]"
        ;;
    esac
  '')
  (pkgs.writeShellScriptBin "agixt" ''
    AGIXT_DIR="''${AGIXT_DIR:-$PWD}"
    COMPOSE_FILE="$AGIXT_DIR/docker-compose.agixt.yml"
    if [ ! -f "$COMPOSE_FILE" ]; then
      echo "Error: docker-compose.agixt.yml not found in $AGIXT_DIR"
      exit 1
    fi
    case "''${1:-status}" in
      up)
        docker compose -f "$COMPOSE_FILE" up -d "''${@:2}"
        echo "AGiXT API: http://localhost:7437"
        ;;
      down)
        docker compose -f "$COMPOSE_FILE" down "''${@:2}"
        ;;
      logs)
        docker compose -f "$COMPOSE_FILE" logs -f "''${@:2}"
        ;;
      status)
        docker compose -f "$COMPOSE_FILE" ps
        ;;
      shell)
        docker compose -f "$COMPOSE_FILE" exec agixt /bin/bash
        ;;
      *)
        echo "Usage: agixt [up|down|logs|status|shell]"
        ;;
    esac
  '')
  (pkgs.writeShellScriptBin "aios" ''
    AIOS_DIR="''${AIOS_DIR:-$HOME/.local/share/aios}"
    AIOS_PORT="''${AIOS_PORT:-8000}"
    case "''${1:-status}" in
      install)
        mkdir -p "$AIOS_DIR"
        if [ ! -d "$AIOS_DIR/AIOS" ]; then
          git clone https://github.com/agiresearch/AIOS.git "$AIOS_DIR/AIOS"
        else
          echo "AIOS already installed at $AIOS_DIR/AIOS"
        fi
        ;;
      start)
        if [ ! -d "$AIOS_DIR/AIOS" ]; then
          echo "AIOS not installed. Run: aios install"
          exit 1
        fi
        cd "$AIOS_DIR/AIOS"
        pixi run -e aios python -m uvicorn runtime.launch:app --host 0.0.0.0 --port "$AIOS_PORT" "''${@:2}"
        ;;
      stop)
        pkill -f "uvicorn runtime.launch:app" && echo "AIOS stopped" || echo "Not running"
        ;;
      status)
        if pgrep -f "uvicorn runtime.launch:app" > /dev/null; then
          echo "AIOS Kernel is running on port $AIOS_PORT"
        else
          echo "AIOS Kernel is not running"
        fi
        ;;
      *)
        echo "Usage: aios [install|start|stop|status]"
        ;;
    esac
  '')
  (pkgs.writeShellScriptBin "vault-dev" ''
    echo "Starting Vault in dev mode..."
    echo "  Address: https://127.0.0.1:8200"
    echo "  Token: root"
    exec vault server -dev -dev-root-token-id root -dev-tls "$@"
  '')
  (pkgs.writeShellScriptBin "neonctl" ''
    exec npx neonctl@latest "$@"
  '')
  (pkgs.writeShellScriptBin "agentgateway-install" ''
    if ! command -v cargo >/dev/null 2>&1; then
      echo "Error: Rust/Cargo not found" >&2
      exit 1
    fi
    cargo install --git https://github.com/agentgateway/agentgateway agentgateway "$@"
  '')
  (pkgs.writeShellScriptBin "sandbox-runtime" ''
    if command -v sandbox-runtime >/dev/null 2>&1; then
      exec sandbox-runtime "$@"
    fi
    if command -v npx >/dev/null 2>&1; then
      exec npx --yes @anthropic-ai/sandbox-runtime "$@"
    else
      echo "Error: sandbox-runtime requires Node.js and npx" >&2
      exit 127
    fi
  '')
  (pkgs.writeShellScriptBin "mcp-toolbox" ''
    if command -v mcp-toolbox >/dev/null 2>&1; then
      exec mcp-toolbox "$@"
    else
      echo "MCP Toolbox not found. Install via home-manager or go install" >&2
      exit 127
    fi
  '')
  (pkgs.writeShellScriptBin "sublinear-solver" ''
    if command -v npx >/dev/null 2>&1; then
      exec npx sublinear-time-solver "$@"
    else
      echo "ERROR: npx not found" >&2
      exit 127
    fi
  '')
  (pkgs.writeShellScriptBin "kata" ''
    case "''${1:-status}" in
      status)
        echo "Kata Containers Status"
        if command -v kata-runtime >/dev/null 2>&1; then
          echo "Runtime: AVAILABLE"
          kata-runtime --version
        else
          echo "Runtime: NOT AVAILABLE"
        fi
        ;;
      check)
        kata-runtime check
        ;;
      config-docker)
        KATA_PATH=$(command -v kata-runtime)
        echo "Add to /etc/docker/daemon.json:"
        echo '{"runtimes":{"kata":{"path":"'"$KATA_PATH"'"}}}'
        ;;
      test)
        docker run --rm --runtime=kata alpine cat /etc/os-release
        ;;
      version)
        kata-runtime --version
        ;;
      *)
        echo "Usage: kata [status|check|config-docker|test|version]"
        ;;
    esac
  '')
]
