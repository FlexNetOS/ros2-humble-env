---
name: multi-model
description: Multi-model orchestration for ARIA. Activate when dispatching tasks to different LLM providers (OpenAI, local models, cloud providers) or when optimizing cost/latency tradeoffs.
---

# Multi-Model Orchestration

This skill enables ARIA to dispatch tasks to multiple LLM providers beyond Claude.

## Configuration Files

| File | Purpose |
|------|---------|
| `.claude/config/models.json` | Model registry and routing rules |
| `.claude/config/env.template` | API key template (copy to secure location) |
| `.claude/mcp-servers.json` | MCP server configurations |

## Supported Providers

### Native Claude (via Task tool)
```python
# Built-in - no additional config needed
Task(subagent_type="general-purpose", model="opus", prompt="...")
Task(subagent_type="general-purpose", model="sonnet", prompt="...")
Task(subagent_type="general-purpose", model="haiku", prompt="...")
```

### External Models (via MCP or Bash)

#### 1. OpenAI (GPT-4, GPT-4o)
```bash
# Via aichat (configured in pixi.toml)
aichat -m openai:gpt-4-turbo "Your prompt here"

# Via direct API
curl https://api.openai.com/v1/chat/completions \
  -H "Authorization: Bearer $OPENAI_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{"model": "gpt-4-turbo", "messages": [{"role": "user", "content": "..."}]}'
```

#### 2. Local Models (Ollama)
```bash
# List available models
ollama list

# Run inference
ollama run llama3.2 "Your prompt here"

# Via API
curl http://localhost:11434/api/generate \
  -d '{"model": "llama3.2", "prompt": "..."}'
```

#### 3. LocalAI
```bash
# Start LocalAI server
localai run --models-path ./models

# Query (OpenAI-compatible API)
curl http://localhost:8080/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{"model": "llama3", "messages": [...]}'
```

#### 4. vLLM (High-throughput)
```bash
# Start vLLM server
python -m vllm.entrypoints.openai.api_server \
  --model mistralai/Mixtral-8x7B-Instruct-v0.1

# Query (OpenAI-compatible)
curl http://localhost:8000/v1/chat/completions \
  -H "Content-Type: application/json" \
  -d '{"model": "mistralai/Mixtral-8x7B-Instruct-v0.1", "messages": [...]}'
```

## ARIA Multi-Model Dispatch Pattern

### Cost-Optimized Routing
```yaml
routing_strategy:
  # Use cheapest model that can handle the task
  simple_tasks:
    primary: claude.haiku      # Fast, cheap
    fallback: local.ollama     # Free, offline

  analysis_tasks:
    primary: claude.sonnet     # Balanced
    fallback: openai.gpt4      # Alternative

  complex_tasks:
    primary: claude.opus       # Best reasoning
    fallback: openai.gpt4      # Alternative
```

### Parallel Multi-Model Execution
```python
# Example: Get multiple perspectives on architecture decision
# Launch tasks to different models in parallel

# Claude perspective
Task(subagent_type="general-purpose", model="sonnet",
     prompt="Analyze this architecture from security standpoint...")

# OpenAI perspective (via Bash + aichat)
Bash(command='aichat -m openai:gpt-4 "Analyze this architecture..."')

# Local model (via Bash + ollama)
Bash(command='ollama run llama3.2 "Analyze this architecture..."')
```

### Consensus Pattern
```python
# Get agreement from multiple models before proceeding
models = ["claude.sonnet", "openai.gpt4", "local.llama3"]
responses = []

for model in models:
    response = query_model(model, prompt)
    responses.append(response)

# Require 2/3 agreement for critical decisions
consensus = check_consensus(responses, threshold=0.66)
```

## Environment Setup

### 1. Copy API Key Template
```bash
cp .claude/config/env.template ~/.config/claude-code/env
# Edit with your actual API keys
```

### 2. Source Environment
```bash
# Add to ~/.bashrc or ~/.zshrc
if [ -f ~/.config/claude-code/env ]; then
  set -a
  source ~/.config/claude-code/env
  set +a
fi
```

### 3. Verify Configuration
```bash
# Check Claude
echo "Claude: ${ANTHROPIC_API_KEY:0:10}..."

# Check OpenAI
echo "OpenAI: ${OPENAI_API_KEY:0:10}..."

# Check local models
ollama list
curl -s http://localhost:8080/v1/models | jq .  # LocalAI
```

## Model Selection Matrix

| Task Type | Recommended | Fallback | Reason |
|-----------|-------------|----------|--------|
| Orchestration | claude.opus | - | Best reasoning |
| Code review | claude.sonnet | openai.gpt4 | Code understanding |
| Documentation | claude.haiku | local.llama3 | Cost-effective |
| Vision/Images | openai.gpt4o | claude.sonnet | Multimodal |
| Private data | local.ollama | local.localai | Data stays local |
| High volume | local.vllm | local.localai | Throughput |
| Offline | local.ollama | - | No internet |

## Security Considerations

1. **Never commit API keys** - Use env files outside repo
2. **Rotate keys regularly** - Especially for production
3. **Use local models for sensitive data** - PII, proprietary code
4. **Audit model access** - Log which models see what data
5. **Rate limit external APIs** - Prevent cost overruns

## Troubleshooting

### Model not responding
```bash
# Check if service is running
curl -s http://localhost:11434/api/tags  # Ollama
curl -s http://localhost:8080/v1/models  # LocalAI

# Check API key
curl -s https://api.openai.com/v1/models \
  -H "Authorization: Bearer $OPENAI_API_KEY" | jq .error
```

### Slow inference
```bash
# Use quantized models locally
ollama pull llama3.2:3b-instruct-q4_0  # Smaller, faster

# Or use vLLM for batching
python -m vllm.entrypoints.openai.api_server \
  --model meta-llama/Llama-3-8b-instruct \
  --tensor-parallel-size 2  # Multi-GPU
```

## Related Skills

- [AI Assistants](../ai-assistants/SKILL.md) - aichat, aider configuration
- [Inference](../inference/SKILL.md) - LocalAI, vLLM setup
- [Observability](../observability/SKILL.md) - Model metrics monitoring
