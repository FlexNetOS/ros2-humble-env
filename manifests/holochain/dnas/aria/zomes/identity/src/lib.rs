use hdk::prelude::*;
use identity_integrity::*;

/// Anchor for all profiles
const ALL_PROFILES_ANCHOR: &str = "all_profiles";

/// Create a new agent profile
/// This creates the profile entry and links it to the agent
#[hdk_extern]
pub fn create_profile(profile: AgentProfile) -> ExternResult<ActionHash> {
    // Create the profile entry
    let action_hash = create_entry(EntryTypes::AgentProfile(profile.clone()))?;

    // Get the entry hash for linking
    let entry_hash = hash_entry(&profile)?;

    // Link from the agent to their profile
    let agent_pubkey = agent_info()?.agent_latest_pubkey;
    create_link(
        agent_pubkey.clone(),
        entry_hash.clone(),
        LinkTypes::AgentToProfile,
        (),
    )?;

    // Link from the all_profiles anchor
    let anchor_hash = hash_entry(&ALL_PROFILES_ANCHOR)?;
    create_link(
        anchor_hash,
        entry_hash,
        LinkTypes::AllProfiles,
        (),
    )?;

    Ok(action_hash)
}

/// Get the profile for the current agent
#[hdk_extern]
pub fn get_my_profile(_: ()) -> ExternResult<Option<Record>> {
    let agent_pubkey = agent_info()?.agent_latest_pubkey;
    get_profile_for_agent(agent_pubkey)
}

/// Get the profile for a specific agent
#[hdk_extern]
pub fn get_profile_for_agent(agent: AgentPubKey) -> ExternResult<Option<Record>> {
    // Get links from agent to profile
    let links = get_links(
        GetLinksInputBuilder::try_new(agent, LinkTypes::AgentToProfile)?.build(),
    )?;

    if links.is_empty() {
        return Ok(None);
    }

    // Get the first (most recent) profile
    let link = &links[0];
    let record = get(link.target.clone(), GetOptions::default())?;

    Ok(record)
}

/// Update the current agent's profile
#[hdk_extern]
pub fn update_profile(profile: AgentProfile) -> ExternResult<ActionHash> {
    // Get the current profile
    let current_profile_record = get_my_profile(())?;

    match current_profile_record {
        Some(record) => {
            // Update the existing profile
            let action_hash = update_entry(record.action_hash().clone(), &profile)?;
            Ok(action_hash)
        }
        None => {
            // No existing profile, create a new one
            create_profile(profile)
        }
    }
}

/// Delete the current agent's profile
#[hdk_extern]
pub fn delete_profile(_: ()) -> ExternResult<ActionHash> {
    let current_profile_record = get_my_profile(())?;

    match current_profile_record {
        Some(record) => {
            let action_hash = delete_entry(record.action_hash().clone())?;
            Ok(action_hash)
        }
        None => Err(wasm_error!(WasmErrorInner::Guest(
            "No profile found to delete".to_string()
        ))),
    }
}

/// Get all profiles in the network
#[hdk_extern]
pub fn get_all_profiles(_: ()) -> ExternResult<Vec<Record>> {
    let anchor_hash = hash_entry(&ALL_PROFILES_ANCHOR)?;

    // Get all links from the anchor
    let links = get_links(
        GetLinksInputBuilder::try_new(anchor_hash, LinkTypes::AllProfiles)?.build(),
    )?;

    // Fetch all the records
    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Create a new credential for an agent
#[hdk_extern]
pub fn create_credential(credential: AgentCredential) -> ExternResult<ActionHash> {
    // Create the credential entry
    let action_hash = create_entry(EntryTypes::AgentCredential(credential.clone()))?;

    // Get the entry hash for linking
    let entry_hash = hash_entry(&credential)?;

    // Link from the agent to their credential
    let agent_pubkey = agent_info()?.agent_latest_pubkey;
    create_link(
        agent_pubkey,
        entry_hash,
        LinkTypes::AgentToCredential,
        (),
    )?;

    Ok(action_hash)
}

/// Get all credentials for the current agent
#[hdk_extern]
pub fn get_my_credentials(_: ()) -> ExternResult<Vec<Record>> {
    let agent_pubkey = agent_info()?.agent_latest_pubkey;
    get_credentials_for_agent(agent_pubkey)
}

/// Get all credentials for a specific agent
#[hdk_extern]
pub fn get_credentials_for_agent(agent: AgentPubKey) -> ExternResult<Vec<Record>> {
    // Get links from agent to credentials
    let links = get_links(
        GetLinksInputBuilder::try_new(agent, LinkTypes::AgentToCredential)?.build(),
    )?;

    // Fetch all credential records
    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Revoke a credential (delete it)
#[hdk_extern]
pub fn revoke_credential(action_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_entry(action_hash)
}

/// Assign a role to an agent
#[hdk_extern]
pub fn assign_role(assignment: RoleAssignment) -> ExternResult<ActionHash> {
    // Create the role assignment entry
    let action_hash = create_entry(EntryTypes::RoleAssignment(assignment.clone()))?;

    // Get the entry hash for linking
    let entry_hash = hash_entry(&assignment)?;

    // Link from the agent to their role assignment
    create_link(
        assignment.agent.clone(),
        entry_hash.clone(),
        LinkTypes::AgentToRole,
        (),
    )?;

    // Link from the role to the agent
    let role_anchor = hash_entry(&assignment.role)?;
    create_link(
        role_anchor,
        entry_hash,
        LinkTypes::RoleToAgent,
        (),
    )?;

    Ok(action_hash)
}

/// Get all role assignments for the current agent
#[hdk_extern]
pub fn get_my_roles(_: ()) -> ExternResult<Vec<Record>> {
    let agent_pubkey = agent_info()?.agent_latest_pubkey;
    get_roles_for_agent(agent_pubkey)
}

/// Get all role assignments for a specific agent
#[hdk_extern]
pub fn get_roles_for_agent(agent: AgentPubKey) -> ExternResult<Vec<Record>> {
    // Get links from agent to role assignments
    let links = get_links(
        GetLinksInputBuilder::try_new(agent, LinkTypes::AgentToRole)?.build(),
    )?;

    // Fetch all role assignment records
    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Get all agents with a specific role
#[hdk_extern]
pub fn get_agents_with_role(role: String) -> ExternResult<Vec<Record>> {
    let role_anchor = hash_entry(&role)?;

    // Get links from role to agents
    let links = get_links(
        GetLinksInputBuilder::try_new(role_anchor, LinkTypes::RoleToAgent)?.build(),
    )?;

    // Fetch all role assignment records
    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Revoke a role assignment
#[hdk_extern]
pub fn revoke_role(action_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_entry(action_hash)
}

/// Search profiles by capability
#[hdk_extern]
pub fn search_profiles_by_capability(capability: String) -> ExternResult<Vec<Record>> {
    let all_profiles = get_all_profiles(())?;

    // Filter profiles that have the requested capability
    let matching_profiles: Vec<Record> = all_profiles
        .into_iter()
        .filter(|record| {
            if let Some(entry) = record.entry().as_option() {
                if let Ok(profile) = AgentProfile::try_from(entry.clone()) {
                    return profile.capabilities.contains(&capability);
                }
            }
            false
        })
        .collect();

    Ok(matching_profiles)
}

/// Get agent info (wrapper for convenience)
#[hdk_extern]
pub fn whoami(_: ()) -> ExternResult<AgentInfo> {
    agent_info()
}
