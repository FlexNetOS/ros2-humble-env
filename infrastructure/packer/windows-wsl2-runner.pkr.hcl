# Packer template for Windows Server 2022 with WSL2 pre-installed
# This creates an AMI suitable for GitHub Actions self-hosted runners
#
# Usage:
#   packer init windows-wsl2-runner.pkr.hcl
#   packer build -var "github_runner_version=2.311.0" windows-wsl2-runner.pkr.hcl

packer {
  required_plugins {
    amazon = {
      version = ">= 1.2.0"
      source  = "github.com/hashicorp/amazon"
    }
  }
}

# Variables
variable "aws_region" {
  type    = string
  default = "us-west-2"
}

variable "github_runner_version" {
  type        = string
  default     = "2.311.0"
  description = "GitHub Actions runner version"
}

variable "instance_type" {
  type    = string
  default = "m5.large"
}

variable "ami_name_prefix" {
  type    = string
  default = "github-runner-windows-wsl2"
}

# Data sources
data "amazon-ami" "windows_server_2022" {
  filters = {
    name                = "Windows_Server-2022-English-Full-Base-*"
    virtualization-type = "hvm"
    root-device-type    = "ebs"
  }
  most_recent = true
  owners      = ["801119661308"] # Amazon
  region      = var.aws_region
}

# Source configuration
source "amazon-ebs" "windows_wsl2" {
  ami_name              = "${var.ami_name_prefix}-${formatdate("YYYYMMDD-hhmm", timestamp())}"
  ami_description       = "Windows Server 2022 with WSL2 for GitHub Actions self-hosted runners"
  instance_type         = var.instance_type
  region                = var.aws_region
  source_ami            = data.amazon-ami.windows_server_2022.id
  communicator          = "winrm"
  winrm_username        = "Administrator"
  winrm_use_ssl         = true
  winrm_insecure        = true
  winrm_timeout         = "10m"

  # User data to enable WinRM
  user_data = <<-EOF
    <powershell>
    # Enable WinRM
    winrm quickconfig -force
    winrm set winrm/config/service/auth '@{Basic="true"}'
    winrm set winrm/config/service '@{AllowUnencrypted="true"}'

    # Configure firewall
    netsh advfirewall firewall add rule name="WinRM HTTPS" dir=in action=allow protocol=TCP localport=5986
    netsh advfirewall firewall add rule name="WinRM HTTP" dir=in action=allow protocol=TCP localport=5985

    # Create self-signed certificate for WinRM HTTPS
    $cert = New-SelfSignedCertificate -DnsName $env:COMPUTERNAME -CertStoreLocation Cert:\LocalMachine\My
    winrm create winrm/config/Listener?Address=*+Transport=HTTPS "@{Hostname=`"$($env:COMPUTERNAME)`"; CertificateThumbprint=`"$($cert.Thumbprint)`"}"

    # Set Administrator password
    $password = ConvertTo-SecureString "SuperSecurePassword123!" -AsPlainText -Force
    Set-LocalUser -Name Administrator -Password $password
    </powershell>
  EOF

  # Tags
  tags = {
    Name        = "${var.ami_name_prefix}"
    Environment = "ci"
    Purpose     = "github-actions-runner"
    WSL2        = "enabled"
  }

  # Increase volume size for WSL2 distributions
  launch_block_device_mappings {
    device_name           = "/dev/sda1"
    volume_size           = 100
    volume_type           = "gp3"
    delete_on_termination = true
  }
}

# Build configuration
build {
  name    = "windows-wsl2-runner"
  sources = ["source.amazon-ebs.windows_wsl2"]

  # Step 1: Enable WSL and VM Platform features
  provisioner "powershell" {
    inline = [
      "Write-Host 'Enabling Windows features for WSL2...'",
      "Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux -NoRestart -WarningAction SilentlyContinue",
      "Enable-WindowsOptionalFeature -Online -FeatureName VirtualMachinePlatform -NoRestart -WarningAction SilentlyContinue",
      "Write-Host 'Windows features enabled, restarting...'"
    ]
  }

  # Restart for features to take effect
  provisioner "windows-restart" {
    restart_timeout = "15m"
  }

  # Step 2: Set WSL2 as default and install kernel update
  provisioner "powershell" {
    inline = [
      "Write-Host 'Configuring WSL2...'",

      "# Download and install WSL2 kernel update",
      "$wslKernelUrl = 'https://wslstorestorage.blob.core.windows.net/wslblob/wsl_update_x64.msi'",
      "$wslKernelPath = 'C:\\wsl_update_x64.msi'",
      "Invoke-WebRequest -Uri $wslKernelUrl -OutFile $wslKernelPath -UseBasicParsing",
      "Start-Process msiexec.exe -ArgumentList '/i', $wslKernelPath, '/quiet', '/norestart' -Wait",
      "Remove-Item $wslKernelPath -Force",

      "# Set WSL2 as default",
      "wsl --set-default-version 2",

      "Write-Host 'WSL2 configured successfully'"
    ]
  }

  # Step 3: Install GitHub Actions runner
  provisioner "powershell" {
    inline = [
      "Write-Host 'Installing GitHub Actions runner...'",

      "# Create runner directory",
      "New-Item -ItemType Directory -Path 'C:\\actions-runner' -Force",
      "Set-Location 'C:\\actions-runner'",

      "# Download runner",
      "$runnerVersion = '${var.github_runner_version}'",
      "$runnerUrl = \"https://github.com/actions/runner/releases/download/v$runnerVersion/actions-runner-win-x64-$runnerVersion.zip\"",
      "Invoke-WebRequest -Uri $runnerUrl -OutFile 'runner.zip' -UseBasicParsing",

      "# Extract runner",
      "Expand-Archive -Path 'runner.zip' -DestinationPath '.' -Force",
      "Remove-Item 'runner.zip' -Force",

      "Write-Host 'GitHub Actions runner installed'"
    ]
  }

  # Step 4: Pre-download NixOS-WSL
  provisioner "powershell" {
    inline = [
      "Write-Host 'Pre-downloading NixOS-WSL...'",

      "# Create directory for NixOS files",
      "New-Item -ItemType Directory -Path 'C:\\NixOS' -Force",

      "# Download NixOS-WSL release",
      "$nixosUrl = 'https://github.com/nix-community/NixOS-WSL/releases/latest/download/nixos-wsl.tar.gz'",
      "Invoke-WebRequest -Uri $nixosUrl -OutFile 'C:\\NixOS\\nixos-wsl.tar.gz' -UseBasicParsing",

      "Write-Host 'NixOS-WSL downloaded to C:\\NixOS'"
    ]
  }

  # Step 5: Configure Windows for CI
  provisioner "powershell" {
    inline = [
      "Write-Host 'Configuring Windows for CI...'",

      "# Disable Windows Update (for stability)",
      "Set-Service wuauserv -StartupType Disabled -ErrorAction SilentlyContinue",

      "# Disable Defender real-time protection (for performance)",
      "Set-MpPreference -DisableRealtimeMonitoring $true -ErrorAction SilentlyContinue",

      "# Enable long paths",
      "Set-ItemProperty -Path 'HKLM:\\SYSTEM\\CurrentControlSet\\Control\\FileSystem' -Name 'LongPathsEnabled' -Value 1",

      "# Configure power settings",
      "powercfg /change monitor-timeout-ac 0",
      "powercfg /change standby-timeout-ac 0",

      "Write-Host 'Windows configured for CI'"
    ]
  }

  # Step 6: Create runner configuration script
  provisioner "powershell" {
    inline = [
      "Write-Host 'Creating runner configuration script...'",

      "$configScript = @'",
      "param(",
      "    [Parameter(Mandatory)]",
      "    [string]$Url,",
      "    [Parameter(Mandatory)]",
      "    [string]$Token,",
      "    [string]$Labels = 'self-hosted,Windows,WSL2,nix'",
      ")",
      "",
      "Set-Location C:\\actions-runner",
      ".\\config.cmd --url $Url --token $Token --labels $Labels --unattended --replace",
      ".\\svc.cmd install",
      ".\\svc.cmd start",
      "'@",

      "Set-Content -Path 'C:\\actions-runner\\configure-runner.ps1' -Value $configScript",

      "Write-Host 'Runner configuration script created at C:\\actions-runner\\configure-runner.ps1'"
    ]
  }

  # Step 7: Create README
  provisioner "powershell" {
    inline = [
      "$readme = @'",
      "# GitHub Actions Self-Hosted Runner (Windows + WSL2)",
      "",
      "This AMI is pre-configured with:",
      "- Windows Server 2022",
      "- WSL2 enabled and configured",
      "- GitHub Actions runner installed (not configured)",
      "- NixOS-WSL pre-downloaded",
      "",
      "## Configure the Runner",
      "",
      "1. Get a registration token from GitHub:",
      "   - Go to Settings > Actions > Runners > New self-hosted runner",
      "   - Copy the token",
      "",
      "2. Run the configuration script:",
      "   ```powershell",
      "   C:\\actions-runner\\configure-runner.ps1 -Url 'https://github.com/OWNER/REPO' -Token 'YOUR_TOKEN'",
      "   ```",
      "",
      "3. The runner will start automatically as a Windows service.",
      "",
      "## Pre-installed Components",
      "",
      "- WSL2 kernel update",
      "- GitHub Actions runner v${var.github_runner_version}",
      "- NixOS-WSL (at C:\\NixOS\\nixos-wsl.tar.gz)",
      "",
      "## Labels",
      "",
      "Default labels: self-hosted, Windows, WSL2, nix",
      "'@",

      "Set-Content -Path 'C:\\README.md' -Value $readme",

      "Write-Host 'AMI build complete!'"
    ]
  }

  # Post-processors for manifest
  post-processor "manifest" {
    output     = "manifest.json"
    strip_path = true
  }
}
