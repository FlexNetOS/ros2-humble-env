# Python Dual-Environment Design

## Overview

This ROS2 Humble development environment uses **two separate Python installations** to balance compatibility and flexibility:

1. **Python 3.13** (from Nix flake) - System tools and build scripts
2. **Python 3.11** (from Pixi) - ROS2 Humble development

This document explains why, when to use each, and how to troubleshoot common issues.

---

## Why Two Python Environments?

### Python 3.13 from Nix (flake.nix)

**Purpose**: System-level tools, build scripts, and non-ROS utilities

**Benefits**:
- Access to latest Python 3.13 features and performance improvements
- Better for general-purpose scripts and DevOps tooling
- Isolated from ROS2 ecosystem constraints
- Includes tools like `pip`, `virtualenv`, and core utilities
- Used for automated scripts that don't interact with ROS2

**Used by**:
- System setup scripts
- Build automation tools
- Development utilities
- Non-ROS Python applications

### Python 3.11 from Pixi (pixi.toml)

**Purpose**: ROS2 Humble development and package management

**Constraints**:
- ROS2 Humble officially supports Python 3.11 and earlier
- Pre-built ROS2 binary packages are compiled for Python 3.11
- Mixed Python versions cause dependency conflicts and build failures

**Benefits**:
- Full ROS2 package compatibility
- Access to ROS2-specific packages via robostack and conda-forge
- Reproducible environments with Pixi/Conda lock files
- Easy management of C++ build dependencies via Conda

**Used by**:
- ROS2 nodes and packages
- ROS2 build processes (`colcon`)
- ROS2-dependent Python code
- Package management with robostack

---

## When to Use Which

### Use Python 3.13 (Nix) - System Scripts

```bash
# These automatically use Python 3.13 from the Nix environment
python3 /path/to/script.py          # Explicit Python 3.13
python3 -m pip install package      # Install to system Python
python3 -m venv ./venv              # Create virtual environment
```

**Examples**:
- Setup and bootstrap scripts (`bootstrap.sh`)
- Build orchestration scripts
- CI/CD pipeline scripts
- General-purpose utilities
- System administration tools

### Use Python 3.11 (Pixi) - ROS2 Development

```bash
# The Pixi environment is activated by direnv automatically
python -m colcon build              # Build ROS2 packages
python -c "import rclpy"            # Import ROS2 packages
pixi run python script.py           # Explicitly run with Pixi Python
```

**Examples**:
- ROS2 node development
- ROS2 package building
- ROS2-dependent scripts
- Any Python code that imports `rclpy`, `geometry_msgs`, etc.

### Environment Activation

```bash
# direnv automatically activates the Pixi environment
# This adds Python 3.11 to your PATH
direnv allow

# Or manually activate
source <(pixi shell-hook)

# You can check which Python is active
which python    # Should point to Pixi environment
python --version # Should show 3.11.x
```

---

## The Pixi Constraint: `>=3.11,<3.13`

Located in `pixi.toml`:

```toml
[dependencies]
python = ">=3.11,<3.13"
```

### What This Means

| Operator | Meaning | Rationale |
|----------|---------|-----------|
| `>=3.11` | Minimum Python 3.11 | ROS2 Humble requirement |
| `<3.13` | Exclude Python 3.13+ | Avoid conflicts with Nix Python 3.13 |

### Why Not Allow 3.13?

1. **Incompatibility Risk**: ROS2 binary packages not tested with Python 3.13
2. **Duplicate Interpreters**: Would create confusion with system Python 3.13
3. **Conda vs Nix Conflict**: Pixi (Conda) Python 3.13 would conflict with Nix Python 3.13
4. **Version Clarity**: The constraint ensures users understand the separation

### Why Not Lower Than 3.11?

- ROS2 Humble dropped support for Python 3.10 and earlier
- Robostack (the ROS2 Conda channel) only builds for 3.11+
- No benefit to supporting older EOL Python versions

---

## Common Workflows

### Developing a ROS2 Node

```bash
# Ensure Pixi environment is active
direnv allow

# Create your ROS2 package (uses Pixi Python 3.11)
ros2 pkg create my_package --build-type ament_python

# Write your node code
cat > my_package/my_package/node.py << 'EOF'
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')

if __name__ == '__main__':
    rclpy.init()
    node = MyNode()
    rclpy.spin(node)
EOF

# Build the package (uses Pixi Python 3.11)
colcon build --packages-select my_package

# Run the node
ros2 run my_package my_node
```

### Running a System Utility Script

```bash
# This automatically uses Nix Python 3.13
python3 ./scripts/deploy.py --target production

# Or if direnv is active, explicitly use Nix python
nix-shell -p python313 --run "python3 ./scripts/deploy.py"
```

### Checking Your Python Versions

```bash
# While in the Nix develop environment (Pixi activated)
python --version     # Should show 3.11.x (from Pixi)
python3 --version    # Should show 3.13.x (from Nix)

# Outside the environment
python --version     # System Python (may be anything)
python3 --version    # System Python (may be anything)
```

---

## Troubleshooting

### Issue 1: ROS2 Import Errors ("ModuleNotFoundError: No module named 'rclpy'")

**Cause**: Using the wrong Python interpreter

**Solution**:
```bash
# Verify Pixi environment is active
which python
# Should output something like: /home/user/.pixi/envs/robostack/bin/python

# If not, activate it
direnv allow
source <(pixi shell-hook)

# Verify Python version
python --version
# Should show: Python 3.11.x
```

### Issue 2: Conflicting Python Versions When Building

**Cause**: `colcon build` using wrong Python version

**Solution**:
```bash
# Ensure Pixi environment is active BEFORE building
direnv allow
source <(pixi shell-hook)

# Verify which colcon is being used
which colcon
# Should show Pixi version

# Clean and rebuild
colcon build --symlink-install
```

### Issue 3: "Python 3.13 Package Not Available in Conda"

**Cause**: Trying to use Python 3.13 with ROS2 packages

**Solution**: Use Python 3.11 (Pixi) for ROS2 work:
```bash
# Check pixi.toml constraint
cat pixi.toml | grep "python ="

# This is intentional - use 3.11 for ROS2
# Use system Python 3.13 for non-ROS scripts
python3 /path/to/non-ros-script.py
```

### Issue 4: Package Installed in Wrong Python

**Cause**: Installing with system `pip` instead of Pixi

**Solution**:
```bash
# ❌ WRONG (installs to system Python)
pip install some-ros2-package

# ✅ CORRECT (installs to Pixi Python)
pixi add some-ros2-package

# Or activate Pixi first
direnv allow
pixi run pip install some-package
```

### Issue 5: Virtual Environment Conflicts

**Cause**: Creating venv with Python 3.13 for ROS2 work

**Solution**: Don't create virtual environments with Python 3.13 for ROS2:
```bash
# ❌ WRONG
python3 -m venv ./ros2-venv
source ./ros2-venv/bin/activate
# This gives Python 3.13, but ROS2 expects 3.11

# ✅ CORRECT - Use Pixi environment instead
# (no venv needed - Pixi provides isolation)
direnv allow
pixi run python -c "import rclpy"
```

### Issue 6: "python3: command not found"

**Cause**: Outside the Nix develop environment

**Solution**:
```bash
# Enter the development environment
nom develop
# or
nix develop

# Now python3 is available
python3 --version

# For direnv auto-activation
direnv allow
```

---

## Quick Reference

| Use Case | Command | Python Version |
|----------|---------|-----------------|
| ROS2 development | `direnv allow && colcon build` | 3.11 (Pixi) |
| ROS2 nodes | `ros2 run package_name node_name` | 3.11 (Pixi) |
| System scripts | `python3 script.py` | 3.13 (Nix) |
| Build tools | `./scripts/deploy.py` | 3.13 (Nix) |
| Check active Python | `which python` | Shows path |
| Switch to system tools | `python3` | Nix 3.13 |
| Switch to ROS2 | `python` | Pixi 3.11 |

---

## Related Configuration Files

| File | Relevant Section | Purpose |
|------|------------------|---------|
| `flake.nix` | Lines ~167-169 | Defines Python 3.13 for Nix |
| `pixi.toml` | Line 8 | Defines Python 3.11 constraint for ROS2 |
| `.envrc` | (auto-activated by direnv) | Activates Pixi environment |
| `bootstrap.sh` | Python setup section | Environment initialization |

---

## Additional Resources

- [ROS2 Humble Official Requirements](https://docs.ros.org/en/humble/Installation.html)
- [Pixi Documentation](https://pixi.sh)
- [Robostack (ROS2 Conda Channel)](https://www.robostack.org/)
- [Nix Flakes Documentation](https://nixos.org/manual/nix/unstable/command-ref/new-cli/nix3-flake.html)
- [Python 3.13 Release Notes](https://www.python.org/downloads/release/python-3130/)
- [Python 3.11 Release Notes](https://www.python.org/downloads/release/python-3110/)

---

## Summary

The dual Python environment design:

1. **Provides flexibility**: Use modern Python 3.13 for tools and utilities
2. **Ensures compatibility**: Use Python 3.11 for ROS2 Humble development
3. **Prevents conflicts**: Clear separation avoids environment confusion
4. **Enables reproducibility**: Pixi/Conda manages precise versions

When in doubt:
- **For ROS2 work**: Use `python` (Pixi, 3.11)
- **For system tools**: Use `python3` (Nix, 3.13)
- **To verify**: Run `which python` and `which python3`
