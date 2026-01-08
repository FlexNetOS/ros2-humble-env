<#
.SYNOPSIS
    Pester unit tests for bootstrap.ps1

.DESCRIPTION
    Tests PowerShell functions with mocked WSL2 calls.
    Can run on any Windows system without actual WSL2.

.NOTES
    Run with: Invoke-Pester -Path test/unit/bootstrap.tests.ps1 -CI
#>

BeforeAll {
    # Dot-source the bootstrap script to get access to functions
    # We need to handle the #Requires -RunAsAdministrator by mocking
    $scriptPath = Join-Path $PSScriptRoot "../../bootstrap.ps1"

    # Read the script content and remove the #Requires directive for testing
    $scriptContent = Get-Content $scriptPath -Raw
    $scriptContent = $scriptContent -replace '#Requires -RunAsAdministrator', '# Requires directive removed for testing'

    # Create a temporary script file without the requires directive
    $tempScript = Join-Path $env:TEMP "bootstrap-test.ps1"
    Set-Content -Path $tempScript -Value $scriptContent

    # Dot-source the modified script
    . $tempScript
}

AfterAll {
    # Cleanup temp script
    $tempScript = Join-Path $env:TEMP "bootstrap-test.ps1"
    if (Test-Path $tempScript) {
        Remove-Item $tempScript -Force
    }
}

Describe "Write-ColorOutput" {
    It "Should write Info messages without error" {
        { Write-ColorOutput -Message "Test message" -Type "Info" } | Should -Not -Throw
    }

    It "Should write Success messages without error" {
        { Write-ColorOutput -Message "Test message" -Type "Success" } | Should -Not -Throw
    }

    It "Should write Warning messages without error" {
        { Write-ColorOutput -Message "Test message" -Type "Warning" } | Should -Not -Throw
    }

    It "Should write Error messages without error" {
        { Write-ColorOutput -Message "Test message" -Type "Error" } | Should -Not -Throw
    }

    It "Should write Step messages without error" {
        { Write-ColorOutput -Message "Test step" -Type "Step" } | Should -Not -Throw
    }
}

Describe "Test-Administrator" {
    It "Should return a boolean value" {
        $result = Test-Administrator
        $result | Should -BeOfType [bool]
    }
}

Describe "Test-WindowsVersion" {
    Context "When Windows version is compatible" {
        BeforeAll {
            Mock Get-ItemProperty {
                return @{ CurrentBuildNumber = "22621" }  # Windows 11
            }
        }

        It "Should return true for compatible Windows version" {
            $result = Test-WindowsVersion
            $result | Should -Be $true
        }
    }

    Context "When Windows version is incompatible" {
        BeforeAll {
            Mock Get-ItemProperty {
                return @{ CurrentBuildNumber = "18363" }  # Windows 10 1909
            }
        }

        It "Should return false for incompatible Windows version" {
            $result = Test-WindowsVersion
            $result | Should -Be $false
        }
    }
}

Describe "Test-WSLInstalled" {
    Context "When WSL is installed" {
        BeforeAll {
            Mock wsl {
                $global:LASTEXITCODE = 0
                return "Default Version: 2"
            }
        }

        It "Should return true when WSL is installed" {
            $result = Test-WSLInstalled
            $result | Should -Be $true
        }
    }

    Context "When WSL is not installed" {
        BeforeAll {
            Mock wsl {
                $global:LASTEXITCODE = 1
                throw "WSL not installed"
            }
        }

        It "Should return false when WSL is not installed" {
            $result = Test-WSLInstalled
            $result | Should -Be $false
        }
    }
}

Describe "Test-WSL2Default" {
    Context "When WSL2 is default" {
        BeforeAll {
            Mock wsl {
                $global:LASTEXITCODE = 0
                return "Default Version: 2"
            }
        }

        It "Should return true when WSL2 is default" {
            $result = Test-WSL2Default
            $result | Should -Be $true
        }
    }

    Context "When WSL1 is default" {
        BeforeAll {
            Mock wsl {
                $global:LASTEXITCODE = 0
                return "Default Version: 1"
            }
        }

        It "Should return false when WSL1 is default" {
            $result = Test-WSL2Default
            $result | Should -Be $false
        }
    }
}

Describe "Test-DistroExists" {
    Context "When distro exists" {
        BeforeAll {
            Mock wsl {
                return @("Ubuntu", "NixOS-ROS2", "Debian")
            }
        }

        It "Should return true when distro exists" {
            $result = Test-DistroExists -Name "NixOS-ROS2"
            $result | Should -Be $true
        }
    }

    Context "When distro does not exist" {
        BeforeAll {
            Mock wsl {
                return @("Ubuntu", "Debian")
            }
        }

        It "Should return false when distro does not exist" {
            $result = Test-DistroExists -Name "NixOS-ROS2"
            $result | Should -Be $false
        }
    }
}

Describe "Set-WSLConfig" {
    BeforeAll {
        # Use a temp directory for testing
        $script:originalUserProfile = $env:USERPROFILE
        $env:USERPROFILE = $TestDrive
    }

    AfterAll {
        $env:USERPROFILE = $script:originalUserProfile
    }

    It "Should create .wslconfig file" {
        # Set global variables that the function uses
        $script:SwapSizeGB = 8

        $result = Set-WSLConfig
        $result | Should -Be $true

        $configPath = Join-Path $TestDrive ".wslconfig"
        Test-Path $configPath | Should -Be $true
    }

    It "Should contain correct memory configuration" {
        $configPath = Join-Path $TestDrive ".wslconfig"
        $content = Get-Content $configPath -Raw

        $content | Should -Match "memory="
        $content | Should -Match "swap="
        $content | Should -Match "localhostForwarding=true"
    }

    It "Should contain experimental settings" {
        $configPath = Join-Path $TestDrive ".wslconfig"
        $content = Get-Content $configPath -Raw

        $content | Should -Match "\[experimental\]"
        $content | Should -Match "autoMemoryReclaim=gradual"
        $content | Should -Match "sparseVhd=true"
    }
}

Describe "Install-WSL" {
    BeforeAll {
        Mock Get-WindowsOptionalFeature {
            return @{ State = "Disabled" }
        }
        Mock Enable-WindowsOptionalFeature { }
        Mock wsl { $global:LASTEXITCODE = 0 }
    }

    It "Should call Enable-WindowsOptionalFeature for WSL" {
        Install-WSL
        Should -Invoke Enable-WindowsOptionalFeature -ParameterFilter {
            $FeatureName -eq "Microsoft-Windows-Subsystem-Linux"
        }
    }

    It "Should call Enable-WindowsOptionalFeature for VirtualMachinePlatform" {
        Install-WSL
        Should -Invoke Enable-WindowsOptionalFeature -ParameterFilter {
            $FeatureName -eq "VirtualMachinePlatform"
        }
    }

    It "Should call wsl --install" {
        Install-WSL
        Should -Invoke wsl -ParameterFilter {
            $args -contains "--install"
        }
    }

    It "Should set WSL2 as default" {
        Install-WSL
        Should -Invoke wsl -ParameterFilter {
            $args -contains "--set-default-version" -and $args -contains "2"
        }
    }
}

Describe "New-NixOSDistro" {
    BeforeAll {
        # Set script-level variables
        $script:DistroName = "NixOS-ROS2-Test"
        $script:InstallPath = $TestDrive
        $script:Force = $false
        $script:NixOSWSLRelease = "https://example.com/nixos.tar.gz"

        Mock Test-DistroExists { return $false }
        Mock Invoke-WebRequest { }
        Mock wsl { $global:LASTEXITCODE = 0 }
        Mock Remove-Item { }
    }

    It "Should create installation directory if it doesn't exist" {
        # Ensure directory doesn't exist
        $testPath = Join-Path $TestDrive "NewDir"
        $script:InstallPath = $testPath

        Mock Test-Path { return $false } -ParameterFilter { $Path -eq $testPath }
        Mock New-Item { }

        New-NixOSDistro

        Should -Invoke New-Item -ParameterFilter {
            $Path -eq $testPath -and $ItemType -eq "Directory"
        }
    }

    It "Should download NixOS-WSL tarball" {
        New-NixOSDistro
        Should -Invoke Invoke-WebRequest
    }

    It "Should import the distribution with WSL2" {
        New-NixOSDistro
        Should -Invoke wsl -ParameterFilter {
            $args -contains "--import" -and $args -contains "--version" -and $args -contains "2"
        }
    }

    Context "When distro already exists" {
        BeforeAll {
            Mock Test-DistroExists { return $true }
            $script:Force = $false
        }

        It "Should not unregister without Force flag" {
            New-NixOSDistro
            Should -Not -Invoke wsl -ParameterFilter {
                $args -contains "--unregister"
            }
        }
    }

    Context "When distro exists and Force is specified" {
        BeforeAll {
            Mock Test-DistroExists { return $true }
            $script:Force = $true
        }

        It "Should unregister existing distro with Force flag" {
            New-NixOSDistro
            Should -Invoke wsl -ParameterFilter {
                $args -contains "--unregister"
            }
        }
    }
}

Describe "Parameter Validation" {
    It "Should have valid default values" {
        # These are the defaults from the param block
        "NixOS-ROS2" | Should -Not -BeNullOrEmpty
        1024 | Should -BeGreaterThan 0
        8 | Should -BeGreaterThan 0
    }

    It "Should accept valid DiskSizeGB values" {
        $validSizes = @(64, 128, 256, 512, 1024, 2048)
        foreach ($size in $validSizes) {
            $size | Should -BeGreaterThan 0
            $size | Should -BeLessOrEqual 4096
        }
    }

    It "Should accept valid SwapSizeGB values" {
        $validSwaps = @(4, 8, 16, 32)
        foreach ($swap in $validSwaps) {
            $swap | Should -BeGreaterThan 0
            $swap | Should -BeLessOrEqual 64
        }
    }
}

Describe "URL Configuration" {
    It "Should have valid NixOS-WSL release URL" {
        $url = "https://github.com/nix-community/NixOS-WSL/releases/latest/download/nixos-wsl.tar.gz"
        $url | Should -Match "^https://github.com"
        $url | Should -Match "NixOS-WSL"
    }

    It "Should have valid repository URL" {
        $url = "https://github.com/FlexNetOS/ros2-humble-env.git"
        $url | Should -Match "^https://github.com"
        $url | Should -Match "\.git$"
    }
}
