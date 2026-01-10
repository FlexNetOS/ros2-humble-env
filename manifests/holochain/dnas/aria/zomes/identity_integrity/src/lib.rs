use hdi::prelude::*;

/// Agent profile entry type
/// Stores identity information for ARIA agents in the DHT
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AgentProfile {
    /// Display name for the agent
    pub name: String,
    /// List of capabilities this agent supports (e.g., ["inference", "storage", "compute"])
    pub capabilities: Vec<String>,
    /// Timestamp when this profile was created
    pub created_at: Timestamp,
    /// Optional metadata as JSON string
    pub metadata: Option<String>,
}

/// Agent credential entry type
/// Stores cryptographic credentials and authentication data
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct AgentCredential {
    /// Public key identifier
    pub public_key: String,
    /// Credential type (e.g., "ed25519", "secp256k1")
    pub credential_type: String,
    /// Timestamp of credential issuance
    pub issued_at: Timestamp,
    /// Optional expiration timestamp
    pub expires_at: Option<Timestamp>,
    /// Issuer's agent public key
    pub issuer: AgentPubKey,
}

/// Role assignment entry type
/// Maps agents to roles for RBAC
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct RoleAssignment {
    /// The role identifier (e.g., "admin", "operator", "viewer")
    pub role: String,
    /// Agent assigned to this role
    pub agent: AgentPubKey,
    /// Timestamp of assignment
    pub assigned_at: Timestamp,
    /// Optional expiration
    pub expires_at: Option<Timestamp>,
    /// Assigner's agent public key
    pub assigned_by: AgentPubKey,
}

/// All entry types for the identity zome
#[hdk_entry_defs]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_def(required_validations = 5)]
    AgentProfile(AgentProfile),
    #[entry_def(required_validations = 5)]
    AgentCredential(AgentCredential),
    #[entry_def(required_validations = 5)]
    RoleAssignment(RoleAssignment),
}

/// Link types for the identity zome
#[hdk_link_types]
pub enum LinkTypes {
    /// Links from agent to their profile
    AgentToProfile,
    /// Links from agent to their credentials
    AgentToCredential,
    /// Links from agent to their role assignments
    AgentToRole,
    /// Links from role to assigned agents
    RoleToAgent,
    /// All profiles anchor
    AllProfiles,
}

/// Validation rules for AgentProfile
pub fn validate_agent_profile(profile: &AgentProfile) -> ExternResult<ValidateCallbackResult> {
    // Name must not be empty
    if profile.name.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Agent name cannot be empty".to_string(),
        ));
    }

    // Name length constraints
    if profile.name.len() > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Agent name cannot exceed 100 characters".to_string(),
        ));
    }

    // Capabilities validation
    if profile.capabilities.is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Agent must have at least one capability".to_string(),
        ));
    }

    // Each capability should be non-empty
    for cap in &profile.capabilities {
        if cap.trim().is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "Capability name cannot be empty".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validation rules for AgentCredential
pub fn validate_agent_credential(
    credential: &AgentCredential,
) -> ExternResult<ValidateCallbackResult> {
    // Public key must not be empty
    if credential.public_key.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Public key cannot be empty".to_string(),
        ));
    }

    // Credential type must be recognized
    let valid_types = vec!["ed25519", "secp256k1", "rsa2048", "rsa4096"];
    if !valid_types.contains(&credential.credential_type.as_str()) {
        return Ok(ValidateCallbackResult::Invalid(format!(
            "Invalid credential type: {}. Must be one of: {}",
            credential.credential_type,
            valid_types.join(", ")
        )));
    }

    // If expiration is set, it must be in the future
    if let Some(expires_at) = credential.expires_at {
        if expires_at <= credential.issued_at {
            return Ok(ValidateCallbackResult::Invalid(
                "Expiration time must be after issuance time".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validation rules for RoleAssignment
pub fn validate_role_assignment(
    assignment: &RoleAssignment,
) -> ExternResult<ValidateCallbackResult> {
    // Role must not be empty
    if assignment.role.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Role cannot be empty".to_string(),
        ));
    }

    // Validate role name format (alphanumeric + underscore + hyphen)
    if !assignment
        .role
        .chars()
        .all(|c| c.is_alphanumeric() || c == '_' || c == '-')
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Role name can only contain alphanumeric characters, underscores, and hyphens"
                .to_string(),
        ));
    }

    // If expiration is set, it must be in the future
    if let Some(expires_at) = assignment.expires_at {
        if expires_at <= assignment.assigned_at {
            return Ok(ValidateCallbackResult::Invalid(
                "Expiration time must be after assignment time".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, .. }) => match app_entry {
            EntryTypes::AgentProfile(profile) => validate_agent_profile(&profile),
            EntryTypes::AgentCredential(credential) => validate_agent_credential(&credential),
            EntryTypes::RoleAssignment(assignment) => validate_role_assignment(&assignment),
        },
        FlatOp::StoreEntry(OpEntry::UpdateEntry {
            app_entry, original_action_hash, ..
        }) => {
            // For updates, ensure the original entry exists and validate the new entry
            match app_entry {
                EntryTypes::AgentProfile(profile) => {
                    // Verify the original entry exists
                    let _ = must_get_action(original_action_hash)?;
                    validate_agent_profile(&profile)
                }
                EntryTypes::AgentCredential(credential) => {
                    let _ = must_get_action(original_action_hash)?;
                    validate_agent_credential(&credential)
                }
                EntryTypes::RoleAssignment(assignment) => {
                    let _ = must_get_action(original_action_hash)?;
                    validate_role_assignment(&assignment)
                }
            }
        }
        FlatOp::StoreEntry(OpEntry::DeleteEntry { original_action_hash, .. }) => {
            // Verify the entry to be deleted exists
            let _ = must_get_action(original_action_hash)?;
            Ok(ValidateCallbackResult::Valid)
        }
        FlatOp::RegisterCreateLink {
            link_type,
            base_address: _,
            target_address: _,
            tag: _,
            action: _,
        } => {
            // Validate link creation based on link type
            match link_type {
                LinkTypes::AgentToProfile
                | LinkTypes::AgentToCredential
                | LinkTypes::AgentToRole
                | LinkTypes::RoleToAgent
                | LinkTypes::AllProfiles => Ok(ValidateCallbackResult::Valid),
            }
        }
        FlatOp::RegisterDeleteLink {
            link_type: _,
            base_address: _,
            target_address: _,
            tag: _,
            original_action: _,
            action: _,
        } => {
            // Allow link deletion
            Ok(ValidateCallbackResult::Valid)
        }
        _ => Ok(ValidateCallbackResult::Valid),
    }
}
