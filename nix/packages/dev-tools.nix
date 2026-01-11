# Full development tools (heavier packages for devShells.full)
# These are optional extras beyond base packages
{ pkgs, ... }:

with pkgs; [
  nix-output-monitor
  nix-tree

  # Shell utilities
  bat
  eza
  fd
  ripgrep
  fzf
  yq
  gnutar
  wget
  unzip
  gzip

  # Directory navigation
  zoxide

  # System monitoring
  btop
  htop

  # Infrastructure & Monitoring (from GitHub resources research)
  # See docs/GITHUB-RESOURCES.md for full analysis
  prometheus          # Metrics collection for ROS2 DDS
  natscli             # NATS CLI for messaging
  nats-server         # WAN/multi-site robot messaging
  trippy              # Network diagnostics for DDS traffic
  trivy               # Container/SBOM security scanning
  opa                 # Policy enforcement for ROS2 topics
  syft                # SBOM generation
  grype               # Vulnerability scanning from SBOM
  cosign              # Container image signing and verification
  kubectl             # Kubernetes CLI for cluster management
  helm                # Kubernetes package manager
  kustomize           # Kubernetes configuration management
  containerd          # Container runtime

  # Additional tools from BUILDKIT_STARTER_SPEC
  neovim              # Editor
  sqlite              # Local database

  # Secrets Management (see docs/GITHUB-RESOURCES.md)
  # Note: Vault uses BSL license - requires NIXPKGS_ALLOW_UNFREE=1
  # For dev mode: vault server -dev -dev-root-token-id root
  vault               # HashiCorp Vault for secrets management

  # P2P & Content-Addressed Storage (BUILDKIT_STARTER_SPEC.md L10-11)
  kubo                # IPFS implementation (content-addressed storage)

  # Supply Chain Security (BUILDKIT_STARTER_SPEC.md L18)
  # Note: syft, grype, cosign already listed above

  # PKI Automation (BUILDKIT_STARTER_SPEC.md L5)
  step-cli            # smallstep CLI for mTLS/PKI

  # Isolation & Sandboxing (BUILDKIT_STARTER_SPEC.md L3)
  # Layer 3: 33% -> 100% coverage (P0-001, P0-002)
  firecracker         # MicroVM for untrusted workloads
  runc                # OCI container runtime (low-level)
  kata-runtime        # P0-002: Kata Containers runtime (lightweight VMs)

  # Shells
  zsh
  nushell

  # Editor
  helix

  # Prompt
  starship

  # AI assistants
  aichat
  aider-chat

  # AI inference (edge/local models) - LocalAI
  # Alternative: docker run -p 8080:8080 localai/localai
  # OpenAI-compatible API server for local LLM inference
  # Supports: GGUF, GGML, Safetensors models
  # P2P federation for multi-robot distributed inference
  # See: docs/adr/adr-006-agixt-integration.md
  local-ai

  # Audio (for aider voice features)
  portaudio

  # Build tools & compilation cache
  ccache              # Fast C/C++ compilation cache
  sccache             # Distributed compilation cache with cloud support
  mold                # Fast modern linker (12x faster than lld)
  maturin             # Build tool for PyO3 Rust-Python bindings

  # Rust toolchain (for AGiXT Rust SDK and ROS2 bridges)
  # See: rust/agixt-bridge/Cargo.toml
  cargo               # Rust package manager
  rustc               # Rust compiler
  rust-analyzer       # Rust LSP for editors
  rustfmt             # Rust code formatter
  clippy              # Rust linter

  # Database tools
  sqlx-cli            # SQL database CLI for migrations and schema management

  # Tree-sitter (for LazyVim/Neovim)
  tree-sitter

  # Node.js ecosystem (for LazyVim plugins & LLM testing)
  nodejs_22           # LTS "Jod" - active until Apr 2027
  nodePackages.pnpm
  # Note: promptfoo (LLM eval/testing) not in nixpkgs - use 'npx promptfoo@latest'

  # Git tools
  lazygit             # Git TUI (integrates with LazyVim)

  # Remote development
  devpod              # Client-only devcontainer environments (any backend)
]
