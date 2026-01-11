# Workflow management command wrappers
# Includes: gh-issues, db-query, temporal-ctl, n8n-ctl, workflow-status, codebase-db
{ pkgs, ... }:

[
  (pkgs.writeShellScriptBin "open-url-helper" ''
    URL="$1"
    [ -z "$URL" ] && { echo "Error: No URL provided" >&2; exit 1; }
    echo "Opening: $URL"
    if command -v xdg-open >/dev/null 2>&1; then
      xdg-open "$URL"
    elif command -v open >/dev/null 2>&1; then
      open "$URL"
    else
      echo "Open in browser: $URL"
    fi
  '')
  (pkgs.writeShellScriptBin "gh-issues" ''
    [ ! command -v gh >/dev/null 2>&1 ] && { echo "Error: gh CLI not found" >&2; exit 1; }
    case "''${1:-list}" in
      list)
        STATE="''${2:-open}"
        gh issue list --state "$STATE" --limit "''${3:-20}"
        ;;
      search)
        [ -z "$2" ] && { echo "Usage: gh-issues search <query>" >&2; exit 1; }
        gh issue list --search "$2" --state "''${3:-all}"
        ;;
      labels)
        [ -z "$2" ] && gh label list || gh issue list --label "$2"
        ;;
      assigned)
        gh issue list --assignee "''${2:-@me}"
        ;;
      create)
        shift; gh issue create "$@"
        ;;
      view)
        [ -z "$2" ] && { echo "Usage: gh-issues view <number>" >&2; exit 1; }
        gh issue view "$2"
        ;;
      close)
        [ -z "$2" ] && { echo "Usage: gh-issues close <number>" >&2; exit 1; }
        gh issue close "$2" --reason "''${3:-completed}"
        ;;
      *)
        echo "Usage: gh-issues [list|search|labels|assigned|create|view|close]"
        ;;
    esac
  '')
  (pkgs.writeShellScriptBin "db-query" ''
    DB_TYPE="''${DB_TYPE:-sqlite}"
    DB_PATH="''${DB_PATH:-./data.db}"
    case "''${1:-help}" in
      query|q)
        shift
        case "$DB_TYPE" in
          sqlite) sqlite3 "$DB_PATH" "$@" ;;
          postgres) psql "$@" ;;
          *) echo "Unknown DB_TYPE: $DB_TYPE" >&2; exit 1 ;;
        esac
        ;;
      tables)
        case "$DB_TYPE" in
          sqlite) sqlite3 "$DB_PATH" ".tables" ;;
          postgres) psql -c "\dt" ;;
        esac
        ;;
      schema)
        [ -z "$2" ] && { echo "Usage: db-query schema <table>" >&2; exit 1; }
        case "$DB_TYPE" in
          sqlite) sqlite3 "$DB_PATH" ".schema $2" ;;
          postgres) psql -c "\d $2" ;;
        esac
        ;;
      *)
        echo "Usage: db-query [query|tables|schema]"
        echo "Set DB_TYPE (sqlite|postgres) and DB_PATH"
        ;;
    esac
  '')
  (pkgs.writeShellScriptBin "temporal-ctl" ''
    TEMPORAL_ADDRESS="''${TEMPORAL_ADDRESS:-localhost:7233}"
    TEMPORAL_NAMESPACE="''${TEMPORAL_NAMESPACE:-default}"
    case "''${1:-status}" in
      start)
        echo "Starting Temporal server..."
        temporal server start-dev --namespace "$TEMPORAL_NAMESPACE" "''${@:2}"
        ;;
      status)
        if pgrep -f "temporal server" > /dev/null; then
          echo "Temporal server is running"
          temporal operator namespace list 2>/dev/null || true
        else
          echo "Temporal server is not running"
        fi
        ;;
      workflow)
        shift
        temporal workflow "$@" --namespace "$TEMPORAL_NAMESPACE"
        ;;
      *)
        echo "Usage: temporal-ctl [start|status|workflow]"
        ;;
    esac
  '')
  (pkgs.writeShellScriptBin "n8n-ctl" ''
    N8N_PORT="''${N8N_PORT:-5678}"
    N8N_DATA="''${N8N_DATA:-$HOME/.n8n}"
    case "''${1:-status}" in
      start)
        mkdir -p "$N8N_DATA"
        echo "Starting n8n on port $N8N_PORT..."
        N8N_USER_FOLDER="$N8N_DATA" n8n start --port "$N8N_PORT" "''${@:2}"
        ;;
      stop)
        pkill -f "n8n start" && echo "n8n stopped" || echo "Not running"
        ;;
      status)
        if pgrep -f "n8n start" > /dev/null; then
          echo "n8n is running on port $N8N_PORT"
        else
          echo "n8n is not running"
        fi
        ;;
      *)
        echo "Usage: n8n-ctl [start|stop|status]"
        ;;
    esac
  '')
  (pkgs.writeShellScriptBin "workflow-status" ''
    echo "Workflow Engine Status"
    echo "====================="
    echo ""
    echo "Temporal:"
    if pgrep -f "temporal server" > /dev/null; then
      echo "  Status: Running"
    else
      echo "  Status: Not running"
    fi
    echo ""
    echo "n8n:"
    if pgrep -f "n8n start" > /dev/null; then
      echo "  Status: Running"
    else
      echo "  Status: Not running"
    fi
  '')
  (pkgs.writeShellScriptBin "codebase-db" ''
    DB_PATH="''${CODEBASE_DB:-$PWD/.codebase.db}"
    case "''${1:-status}" in
      init)
        echo "Initializing codebase database at $DB_PATH..."
        sqlite3 "$DB_PATH" << 'SQL'
CREATE TABLE IF NOT EXISTS files (
  id INTEGER PRIMARY KEY,
  path TEXT UNIQUE NOT NULL,
  type TEXT,
  size INTEGER,
  modified TEXT,
  indexed_at TEXT DEFAULT CURRENT_TIMESTAMP
);
CREATE TABLE IF NOT EXISTS symbols (
  id INTEGER PRIMARY KEY,
  file_id INTEGER REFERENCES files(id),
  name TEXT NOT NULL,
  kind TEXT,
  line INTEGER,
  column INTEGER
);
CREATE INDEX IF NOT EXISTS idx_files_path ON files(path);
CREATE INDEX IF NOT EXISTS idx_symbols_name ON symbols(name);
SQL
        echo "Database initialized."
        ;;
      scan)
        echo "Scanning codebase..."
        find . -type f \( -name "*.py" -o -name "*.nix" -o -name "*.rs" -o -name "*.ts" \) \
          ! -path "./.git/*" ! -path "./build/*" ! -path "./install/*" | while read -r f; do
          SIZE=$(stat -c%s "$f" 2>/dev/null || stat -f%z "$f" 2>/dev/null || echo 0)
          MOD=$(stat -c%Y "$f" 2>/dev/null || stat -f%m "$f" 2>/dev/null || echo 0)
          EXT="''${f##*.}"
          sqlite3 "$DB_PATH" "INSERT OR REPLACE INTO files (path, type, size, modified) VALUES ('$f', '$EXT', $SIZE, datetime($MOD, 'unixepoch'));"
        done
        echo "Scan complete."
        ;;
      query)
        shift
        sqlite3 -header -column "$DB_PATH" "$@"
        ;;
      status)
        if [ -f "$DB_PATH" ]; then
          echo "Database: $DB_PATH"
          echo "Files indexed: $(sqlite3 "$DB_PATH" "SELECT COUNT(*) FROM files;")"
          echo "Symbols indexed: $(sqlite3 "$DB_PATH" "SELECT COUNT(*) FROM symbols;")"
        else
          echo "Database not initialized. Run: codebase-db init"
        fi
        ;;
      *)
        echo "Usage: codebase-db [init|scan|query|status]"
        ;;
    esac
  '')
]
