# Vault Policy for Agent Runtime
# Grants access to secrets needed by AIOS, AGiXT, and other agents

# API keys for LLM providers
path "secret/data/llm/*" {
  capabilities = ["read"]
}

# Agent-specific secrets
path "secret/data/agents/*" {
  capabilities = ["read", "create", "update"]
}

# Database credentials (read-only)
path "secret/data/database/*" {
  capabilities = ["read"]
}

# Temporary credentials for tool execution
path "secret/data/tools/*" {
  capabilities = ["read"]
}

# Deny access to admin secrets
path "secret/data/admin/*" {
  capabilities = ["deny"]
}
