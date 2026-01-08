# Bootstrap script for Windows/PowerShell
# Requires: Nix with flakes, Git

$ErrorActionPreference = "Stop"

function Print-Error($msg) {
    Write-Host "ERROR: $msg" -ForegroundColor Red
}

function Print-Success($msg) {
    Write-Host "✓ $msg" -ForegroundColor Green
}

function Print-Info($msg) {
    Write-Host "INFO: $msg" -ForegroundColor Yellow
}

Write-Host "=========================================="
Write-Host "  ROS2 Humble Environment Bootstrap"
Write-Host "=========================================="
Write-Host ""

# Check Nix
if (!(Get-Command nix -ErrorAction SilentlyContinue)) {
    Print-Error "Nix is not installed"
    Write-Host ""
    Write-Host "Install Nix with flakes enabled:"
    Write-Host "  Visit: https://nixos.org/download.html"
    Write-Host ""
    exit 1
}

# Test flakes
try {
    $null = nix eval --expr "1 + 1" 2>&1
} catch {
    Print-Error "Nix flakes are not enabled"
    Write-Host "Enable in nix.conf:"
    Write-Host "  experimental-features = nix-command flakes"
    exit 1
}

Print-Success "Nix with flakes is installed"

# Check Git
if (!(Get-Command git -ErrorAction SilentlyContinue)) {
    Print-Error "Git is not installed"
    Write-Host ""
    Write-Host "Install Git from: https://git-scm.com/download/win"
    Write-Host ""
    exit 1
}

Print-Success "Git is installed"

# Check direnv
if (Get-Command direnv -ErrorAction SilentlyContinue) {
    Print-Success "direnv is installed"
    Print-Info "After bootstrap, run: direnv allow"
} else {
    Print-Info "direnv not found - you'll need to use 'nom develop' manually"
}

Write-Host ""
Print-Info "Entering dev shell and running pixi install..."
Write-Host ""

# Use nom develop if available
$developCmd = if (Get-Command nom -ErrorAction SilentlyContinue) { "nom" } else { "nix" }

& $developCmd develop --command pwsh -Command {
    $ErrorActionPreference = "Stop"
    
    if (!(Get-Command pixi -ErrorAction SilentlyContinue)) {
        Write-Host "ERROR: pixi not available in dev shell" -ForegroundColor Red
        exit 1
    }
    
    Write-Host "Running pixi install..."
    pixi install
    
    Write-Host ""
    Write-Host "✓ Bootstrap complete!" -ForegroundColor Green
}

if ($LASTEXITCODE -eq 0) {
    Write-Host ""
    Write-Host "━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    Write-Host "  Next Steps"
    Write-Host "━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    Write-Host ""
    if (Get-Command direnv -ErrorAction SilentlyContinue) {
        Write-Host "1. Run: direnv allow"
        Write-Host "2. Environment will activate automatically"
    } else {
        Write-Host "1. Run: nom develop (or nix develop)"
    }
    Write-Host ""
    Print-Success "Setup complete!"
} else {
    Print-Error "Bootstrap failed"
    exit 1
}
