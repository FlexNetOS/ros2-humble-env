# Prompts Directory

This directory contains reusable prompt templates designed for Claude Opus 4.5 and other advanced models.

## Available Prompts

| Prompt | Description | Slash Command |
|--------|-------------|---------------|
| `aria-orchestrator.md` | ARIA: Agentic Research & Integration Architect | `/aria-audit` |

## ARIA Orchestrator

ARIA is a comprehensive codebase audit and orchestration system that:

1. **Discovers** all files, subjects, and reference links
2. **Deploys** parallel research teams for each domain
3. **Synthesizes** findings into structured reports
4. **Generates** prioritized task backlogs

### Quick Commands

```bash
# Full comprehensive audit
/aria-audit

# Quick scan (discovery only)
/aria-scan

# Domain-specific audits
/aria-audit-nix      # Nix/Flake configuration
/aria-audit-ci       # CI/CD workflows
/aria-audit-agents   # Agents and skills

# Generate tasks from audit
/aria-tasks
```

### Team Structure

Each subject domain gets a dedicated team:

```
Orchestrator (ARIA)
    │
    ├── Team Lead (Explore agent)
    │   ├── Specialist 1 (domain-specific)
    │   ├── Specialist 2 (domain-specific)
    │   ├── Specialist 3 (domain-specific)
    │   ├── Sub-agent 1 (validation)
    │   ├── Sub-agent 2 (link checking)
    │   └── Sub-agent 3 (dependency verification)
    │
    └── [Repeat for each domain]
```

## Prompt Design Principles

These prompts follow Anthropic's best practices for Claude 4.x:

1. **Explicit Instructions** — Clear, specific directives
2. **Structured Format** — XML tags for sections
3. **Chain-of-Thought** — Reasoning guidance included
4. **Output Templates** — Defined deliverable formats
5. **Constraints** — Clear boundaries and limitations

## Creating New Prompts

1. Create a new `.md` file in this directory
2. Use the ARIA orchestrator as a template
3. Include these sections:
   - `<system>` — Role and identity
   - `<context>` — Environment and resources
   - `<objective>` — Mission and success criteria
   - `<method>` — Execution strategy
   - `<output_format>` — Required deliverables
   - `<constraints>` — Operating boundaries
   - `<thinking_guidance>` — Reasoning approach
   - `<execution>` — Trigger and first steps

4. Create corresponding slash command in `.claude/commands/`

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0.0 | 2026-01 | Initial ARIA orchestrator |
