#Requires -RunAsAdministrator
<#
.SYNOPSIS
    Bootstrap script for ros2-humble-env on Windows with WSL2 and NixOS.

.DESCRIPTION
    This script sets up a complete ROS2 Humble development environment on Windows by:
    1. Checking and enabling WSL2 if needed
    2. Creating a NixOS WSL distribution
    3. Setting up ext4.vhdx virtual disk (1TB) and swap
    4. Installing Nix with flakes, pixi, and all development tools
    5. Configuring shells (bash, zsh, nushell)

.PARAMETER DistroName
    Name for the WSL distribution (default: NixOS-ROS2)

.PARAMETER InstallPath
    Installation path for the distro (default: $env:USERPROFILE\WSL\NixOS-ROS2)

.PARAMETER DiskSizeGB
    Size of the ext4.vhdx disk in GB (default: 1024 = 1TB)

.PARAMETER SwapSizeGB
    Size of swap in GB (default: 8)

.PARAMETER SkipWSLCheck
    Skip WSL installation check (for already configured systems)

.EXAMPLE
    .\bootstrap.ps1

.EXAMPLE
    .\bootstrap.ps1 -DistroName "MyROS2" -DiskSizeGB 512

.NOTES
    Requires Windows 10 version 2004+ or Windows 11
    Must be run as Administrator
#>

[CmdletBinding()]
param(
    [string]$DistroName = "NixOS-ROS2",
    [string]$InstallPath = "$env:USERPROFILE\WSL\NixOS-ROS2",
    [int]$DiskSizeGB = 1024,
    [int]$SwapSizeGB = 8,
    [switch]$SkipWSLCheck,
    [switch]$Force
)

# Configuration
$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"

# URLs and versions
$NixOSWSLRelease = "https://github.com/nix-community/NixOS-WSL/releases/latest/download/nixos-wsl.tar.gz"
$RepoURL = "https://github.com/FlexNetOS/ros2-humble-env.git"

# Colors for output
function Write-ColorOutput {
    param(
        [string]$Message,
        [string]$Type = "Info"
    )

    switch ($Type) {
        "Info"    { Write-Host "[INFO] " -ForegroundColor Blue -NoNewline; Write-Host $Message }
        "Success" { Write-Host "[SUCCESS] " -ForegroundColor Green -NoNewline; Write-Host $Message }
        "Warning" { Write-Host "[WARNING] " -ForegroundColor Yellow -NoNewline; Write-Host $Message }
        "Error"   { Write-Host "[ERROR] " -ForegroundColor Red -NoNewline; Write-Host $Message }
        "Step"    { Write-Host "`n=== " -ForegroundColor Cyan -NoNewline; Write-Host $Message -ForegroundColor Cyan -NoNewline; Write-Host " ===" -ForegroundColor Cyan }
    }
}

function Test-Administrator {
    $currentUser = [Security.Principal.WindowsIdentity]::GetCurrent()
    $principal = New-Object Security.Principal.WindowsPrincipal($currentUser)
    return $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
}

function Test-WindowsVersion {
    Write-ColorOutput "Checking Windows version..." "Info"

    $osVersion = [System.Environment]::OSVersion.Version
    $buildNumber = (Get-ItemProperty "HKLM:\SOFTWARE\Microsoft\Windows NT\CurrentVersion").CurrentBuildNumber

    # WSL2 requires Windows 10 version 2004 (build 19041) or higher
    if ($buildNumber -lt 19041) {
        Write-ColorOutput "Windows build $buildNumber detected. WSL2 requires build 19041 or higher." "Error"
        Write-ColorOutput "Please update Windows to version 2004 or later." "Error"
        return $false
    }

    Write-ColorOutput "Windows build $buildNumber - Compatible with WSL2" "Success"
    return $true
}

function Test-WSLInstalled {
    Write-ColorOutput "Checking WSL installation..." "Info"

    try {
        $wslStatus = wsl --status 2>&1
        if ($LASTEXITCODE -eq 0) {
            Write-ColorOutput "WSL is installed" "Success"
            return $true
        }
    }
    catch {
        # WSL not installed
    }

    return $false
}

function Test-WSL2Default {
    Write-ColorOutput "Checking WSL default version..." "Info"

    try {
        $wslStatus = wsl --status 2>&1 | Out-String
        if ($wslStatus -match "Default Version: 2" -or $wslStatus -match "既定のバージョン: 2") {
            Write-ColorOutput "WSL2 is the default version" "Success"
            return $true
        }
    }
    catch {
        # Unable to determine
    }

    return $false
}

function Install-WSL {
    Write-ColorOutput "Installing WSL..." "Step"

    # Enable WSL feature
    Write-ColorOutput "Enabling Windows Subsystem for Linux..." "Info"
    $wslFeature = Get-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux
    if ($wslFeature.State -ne "Enabled") {
        Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux -NoRestart -WarningAction SilentlyContinue | Out-Null
        Write-ColorOutput "WSL feature enabled" "Success"
    }
    else {
        Write-ColorOutput "WSL feature already enabled" "Info"
    }

    # Enable Virtual Machine Platform
    Write-ColorOutput "Enabling Virtual Machine Platform..." "Info"
    $vmFeature = Get-WindowsOptionalFeature -Online -FeatureName VirtualMachinePlatform
    if ($vmFeature.State -ne "Enabled") {
        Enable-WindowsOptionalFeature -Online -FeatureName VirtualMachinePlatform -NoRestart -WarningAction SilentlyContinue | Out-Null
        Write-ColorOutput "Virtual Machine Platform enabled" "Success"
    }
    else {
        Write-ColorOutput "Virtual Machine Platform already enabled" "Info"
    }

    # Install WSL
    Write-ColorOutput "Running WSL installation..." "Info"
    wsl --install --no-distribution

    # Set WSL2 as default
    Write-ColorOutput "Setting WSL2 as default version..." "Info"
    wsl --set-default-version 2

    Write-ColorOutput "WSL2 installation complete" "Success"
}

function Test-DistroExists {
    param([string]$Name)

    $distros = wsl --list --quiet 2>&1
    return $distros -contains $Name
}

function New-NixOSDistro {
    Write-ColorOutput "Setting up NixOS WSL distribution..." "Step"

    # Check if distro already exists
    if (Test-DistroExists -Name $DistroName) {
        if ($Force) {
            Write-ColorOutput "Removing existing distro '$DistroName'..." "Warning"
            wsl --unregister $DistroName
        }
        else {
            Write-ColorOutput "Distro '$DistroName' already exists. Use -Force to replace it." "Warning"
            return $true
        }
    }

    # Create installation directory
    Write-ColorOutput "Creating installation directory: $InstallPath" "Info"
    if (-not (Test-Path $InstallPath)) {
        New-Item -ItemType Directory -Path $InstallPath -Force | Out-Null
    }

    # Download NixOS-WSL
    $tarPath = Join-Path $InstallPath "nixos-wsl.tar.gz"
    Write-ColorOutput "Downloading NixOS-WSL..." "Info"

    try {
        Invoke-WebRequest -Uri $NixOSWSLRelease -OutFile $tarPath -UseBasicParsing
        Write-ColorOutput "Download complete" "Success"
    }
    catch {
        Write-ColorOutput "Failed to download NixOS-WSL: $_" "Error"
        return $false
    }

    # Import the distribution
    Write-ColorOutput "Importing NixOS distribution (this may take a few minutes)..." "Info"
    wsl --import $DistroName $InstallPath $tarPath --version 2

    if ($LASTEXITCODE -ne 0) {
        Write-ColorOutput "Failed to import NixOS distribution" "Error"
        return $false
    }

    Write-ColorOutput "NixOS distribution '$DistroName' created successfully" "Success"

    # Clean up tarball
    Remove-Item $tarPath -Force -ErrorAction SilentlyContinue

    return $true
}

function Set-VirtualDiskSize {
    Write-ColorOutput "Configuring virtual disk..." "Step"

    $vhdxPath = Join-Path $InstallPath "ext4.vhdx"

    if (-not (Test-Path $vhdxPath)) {
        Write-ColorOutput "Virtual disk not found at expected location" "Warning"
        return $false
    }

    # Resize the VHDX to specified size
    $diskSizeBytes = [int64]$DiskSizeGB * 1024 * 1024 * 1024

    Write-ColorOutput "Resizing virtual disk to ${DiskSizeGB}GB..." "Info"

    # Shutdown WSL first
    wsl --shutdown
    Start-Sleep -Seconds 2

    try {
        # Use diskpart to resize
        $diskpartScript = @"
select vdisk file="$vhdxPath"
expand vdisk maximum=$DiskSizeGB
"@
        $diskpartScript | diskpart | Out-Null

        Write-ColorOutput "Virtual disk resized to ${DiskSizeGB}GB" "Success"
    }
    catch {
        Write-ColorOutput "Note: Disk resize requires manual expansion inside WSL" "Warning"
    }

    return $true
}

function Set-WSLConfig {
    Write-ColorOutput "Configuring WSL settings..." "Step"

    $wslConfigPath = "$env:USERPROFILE\.wslconfig"

    $wslConfig = @"
[wsl2]
memory=${SwapSizeGB}GB
swap=${SwapSizeGB}GB
localhostForwarding=true

[experimental]
autoMemoryReclaim=gradual
sparseVhd=true
"@

    Write-ColorOutput "Writing WSL configuration to $wslConfigPath" "Info"
    Set-Content -Path $wslConfigPath -Value $wslConfig -Force

    Write-ColorOutput "WSL configuration complete" "Success"
    return $true
}

function Initialize-NixOSEnvironment {
    Write-ColorOutput "Initializing NixOS environment..." "Step"

    # Wait for NixOS to be ready
    Write-ColorOutput "Waiting for NixOS to initialize (first boot)..." "Info"
    wsl -d $DistroName -- echo "NixOS is ready"

    if ($LASTEXITCODE -ne 0) {
        Write-ColorOutput "Failed to start NixOS" "Error"
        return $false
    }

    # Run initial setup commands
    Write-ColorOutput "Running initial NixOS configuration..." "Info"

    # Update channels and rebuild
    $initScript = @'
set -e
echo "Updating Nix channels..."
sudo nix-channel --update

echo "Enabling flakes..."
mkdir -p ~/.config/nix
echo "experimental-features = nix-command flakes" > ~/.config/nix/nix.conf

echo "Initial setup complete"
'@

    wsl -d $DistroName -- bash -c $initScript

    Write-ColorOutput "NixOS environment initialized" "Success"
    return $true
}

function Install-ROS2Environment {
    Write-ColorOutput "Setting up ROS2 development environment..." "Step"

    # Clone the repository
    Write-ColorOutput "Cloning ros2-humble-env repository..." "Info"

    $cloneScript = @"
set -e
cd ~

if [ -d "ros2-humble-env" ]; then
    echo "Repository already exists, pulling latest..."
    cd ros2-humble-env
    git pull
else
    echo "Cloning repository..."
    git clone $RepoURL
    cd ros2-humble-env
fi

echo "Repository ready"
"@

    wsl -d $DistroName -- bash -c $cloneScript

    # Run the bootstrap script
    Write-ColorOutput "Running bootstrap script (this may take a while)..." "Info"

    $bootstrapScript = @'
set -e
cd ~/ros2-humble-env

# Make bootstrap executable
chmod +x bootstrap.sh

# Run bootstrap in CI mode (non-interactive)
./bootstrap.sh --ci

echo "Bootstrap complete!"
'@

    wsl -d $DistroName -- bash -c $bootstrapScript

    if ($LASTEXITCODE -ne 0) {
        Write-ColorOutput "Bootstrap script encountered errors" "Error"
        return $false
    }

    Write-ColorOutput "ROS2 development environment installed successfully" "Success"
    return $true
}

function Set-DefaultDistro {
    Write-ColorOutput "Setting $DistroName as default WSL distribution..." "Info"
    wsl --set-default $DistroName
    Write-ColorOutput "Default distribution set" "Success"
}

function Show-Summary {
    Write-ColorOutput "Installation Summary" "Step"

    Write-Host ""
    Write-Host "========================================" -ForegroundColor Green
    Write-Host "  ROS2 Humble Environment Ready!       " -ForegroundColor Green
    Write-Host "========================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "Distribution: $DistroName"
    Write-Host "Location: $InstallPath"
    Write-Host "Disk Size: ${DiskSizeGB}GB"
    Write-Host "Swap Size: ${SwapSizeGB}GB"
    Write-Host ""
    Write-Host "To enter the environment:" -ForegroundColor Cyan
    Write-Host "  wsl -d $DistroName"
    Write-Host "  cd ~/ros2-humble-env"
    Write-Host "  direnv allow  # or: nom develop"
    Write-Host ""
    Write-Host "Quick commands inside WSL:" -ForegroundColor Cyan
    Write-Host "  cb   - colcon build --symlink-install"
    Write-Host "  ct   - colcon test"
    Write-Host "  ros2 - ROS2 CLI"
    Write-Host ""
    Write-Host "For different shells:" -ForegroundColor Cyan
    Write-Host "  nom develop -c env 'SHELL=/bin/nu' /bin/nu    # Nushell"
    Write-Host "  nom develop -c env 'SHELL=/bin/zsh' /bin/zsh  # Zsh"
    Write-Host ""
}

function Show-RestartRequired {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Yellow
    Write-Host "  RESTART REQUIRED                     " -ForegroundColor Yellow
    Write-Host "========================================" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "Windows features have been enabled that require a restart."
    Write-Host "Please restart your computer and run this script again."
    Write-Host ""

    $restart = Read-Host "Would you like to restart now? (y/N)"
    if ($restart -eq 'y' -or $restart -eq 'Y') {
        Restart-Computer -Force
    }
}

# Main execution
function Main {
    Write-Host ""
    Write-Host "========================================" -ForegroundColor Cyan
    Write-Host "  ROS2 Humble Environment Bootstrap    " -ForegroundColor Cyan
    Write-Host "  Windows WSL2 + NixOS Setup           " -ForegroundColor Cyan
    Write-Host "========================================" -ForegroundColor Cyan
    Write-Host ""

    # Check administrator
    if (-not (Test-Administrator)) {
        Write-ColorOutput "This script must be run as Administrator" "Error"
        Write-ColorOutput "Right-click PowerShell and select 'Run as Administrator'" "Info"
        exit 1
    }

    # Check Windows version
    if (-not (Test-WindowsVersion)) {
        exit 1
    }

    # Check/Install WSL
    if (-not $SkipWSLCheck) {
        $wslInstalled = Test-WSLInstalled
        $wsl2Default = $false

        if ($wslInstalled) {
            $wsl2Default = Test-WSL2Default
        }

        if (-not $wslInstalled -or -not $wsl2Default) {
            Install-WSL

            # Check if restart is needed
            $wslCheck = wsl --status 2>&1
            if ($LASTEXITCODE -ne 0) {
                Show-RestartRequired
                exit 0
            }
        }
    }

    # Configure WSL settings
    Set-WSLConfig

    # Create NixOS distribution
    if (-not (New-NixOSDistro)) {
        Write-ColorOutput "Failed to create NixOS distribution" "Error"
        exit 1
    }

    # Configure virtual disk size
    Set-VirtualDiskSize

    # Shutdown WSL to apply changes
    Write-ColorOutput "Restarting WSL to apply configuration..." "Info"
    wsl --shutdown
    Start-Sleep -Seconds 3

    # Initialize NixOS
    if (-not (Initialize-NixOSEnvironment)) {
        Write-ColorOutput "Failed to initialize NixOS environment" "Error"
        exit 1
    }

    # Install ROS2 environment
    if (-not (Install-ROS2Environment)) {
        Write-ColorOutput "Failed to install ROS2 environment" "Error"
        exit 1
    }

    # Set as default distribution
    Set-DefaultDistro

    # Show summary
    Show-Summary
}

# Run main
try {
    Main
}
catch {
    Write-ColorOutput "An error occurred: $_" "Error"
    Write-ColorOutput $_.ScriptStackTrace "Error"
    exit 1
}
