# =============================================================================
# HashiCorp Vault Configuration
# =============================================================================
# Development Mode Configuration
# WARNING: DO NOT use this configuration in production!
#
# This configuration is for local development only. In production:
# 1. Use proper storage backend (Consul, Raft, or cloud storage)
# 2. Enable TLS with valid certificates
# 3. Configure proper authentication methods
# 4. Set up audit logging
# 5. Use seal/unseal with proper key management
#
# For production setup, see: https://developer.hashicorp.com/vault/docs/configuration
# =============================================================================

# Storage Backend - File system (dev only)
storage "file" {
  path = "/vault/file"
}

# Listener - HTTP (dev mode)
listener "tcp" {
  address     = "0.0.0.0:8200"
  tls_disable = 1
}

# API Address
api_addr = "http://0.0.0.0:8200"

# UI Configuration
ui = true

# Development Mode Settings
# In dev mode, Vault runs entirely in-memory and starts unsealed
# with a single unseal key. The root token is logged to stdout.

# Disable mlock in containerized environments
disable_mlock = true

# Log level
log_level = "info"

# =============================================================================
# Production Configuration Template (commented out)
# =============================================================================
# Uncomment and modify for production use:

# # Storage Backend - Raft (recommended for production)
# storage "raft" {
#   path    = "/vault/data"
#   node_id = "vault-node-1"
#
#   retry_join {
#     leader_api_addr = "https://vault-2:8200"
#   }
#   retry_join {
#     leader_api_addr = "https://vault-3:8200"
#   }
# }

# # Listener - HTTPS with mTLS
# listener "tcp" {
#   address       = "0.0.0.0:8200"
#   tls_cert_file = "/vault/certs/vault.crt"
#   tls_key_file  = "/vault/certs/vault.key"
#   tls_client_ca_file = "/etc/ssl/certs/aria-root-ca.crt"
#   tls_require_and_verify_client_cert = true
# }

# # Cluster Address for HA
# cluster_addr = "https://vault-1:8201"
# api_addr = "https://vault-1:8200"

# # Seal Configuration - Auto-unseal with cloud KMS
# seal "awskms" {
#   region     = "us-east-1"
#   kms_key_id = "alias/vault-unseal-key"
# }

# # Telemetry
# telemetry {
#   prometheus_retention_time = "30s"
#   disable_hostname = true
# }

# # Audit Logging
# # Enable via CLI: vault audit enable file file_path=/vault/logs/audit.log
