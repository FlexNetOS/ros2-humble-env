use hdk::prelude::*;
use storage_integrity::*;

/// Anchor for all collections
const ALL_COLLECTIONS_ANCHOR: &str = "all_collections";

/// Anchor for all records
const ALL_RECORDS_ANCHOR: &str = "all_records";

/// Input for creating a data record
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct CreateRecordInput {
    pub key: String,
    pub data: String,
    pub data_type: String,
    pub metadata: Option<String>,
    pub tags: Vec<String>,
    pub collection_name: Option<String>,
}

/// Input for updating a data record
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct UpdateRecordInput {
    pub action_hash: ActionHash,
    pub data: String,
    pub metadata: Option<String>,
    pub tags: Vec<String>,
}

/// Input for searching records
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SearchRecordsInput {
    pub data_type: Option<String>,
    pub tags: Option<Vec<String>>,
    pub collection_name: Option<String>,
    pub limit: Option<u32>,
}

/// Helper to create an anchor hash from a string
fn anchor_hash(anchor: &str) -> ExternResult<EntryHash> {
    hash_entry(anchor)
}

/// Helper to create a type anchor
fn type_anchor(data_type: &str) -> String {
    format!("type:{}", data_type)
}

/// Helper to create a tag anchor
fn tag_anchor(tag: &str) -> String {
    format!("tag:{}", tag)
}

/// Helper to create a key anchor
fn key_anchor(key: &str) -> String {
    format!("key:{}", key)
}

/// Create a new data record
#[hdk_extern]
pub fn create_record(input: CreateRecordInput) -> ExternResult<ActionHash> {
    let now = sys_time()?;

    let record = DataRecord {
        key: input.key.clone(),
        data: input.data,
        data_type: input.data_type.clone(),
        created_at: now,
        updated_at: now,
        metadata: input.metadata,
        tags: input.tags.clone(),
    };

    // Create the record entry
    let action_hash = create_entry(EntryTypes::DataRecord(record.clone()))?;
    let entry_hash = hash_entry(&record)?;

    // Link from all records anchor
    let all_records_hash = anchor_hash(ALL_RECORDS_ANCHOR)?;
    create_link(
        all_records_hash,
        entry_hash.clone(),
        LinkTypes::AllRecords,
        (),
    )?;

    // Link from key anchor
    let key_hash = anchor_hash(&key_anchor(&input.key))?;
    create_link(
        key_hash,
        entry_hash.clone(),
        LinkTypes::KeyToRecord,
        (),
    )?;

    // Link from type anchor
    let type_hash = anchor_hash(&type_anchor(&input.data_type))?;
    create_link(
        type_hash,
        entry_hash.clone(),
        LinkTypes::TypeToRecord,
        (),
    )?;

    // Link from tag anchors
    for tag in &input.tags {
        let tag_hash = anchor_hash(&tag_anchor(tag))?;
        create_link(
            tag_hash,
            entry_hash.clone(),
            LinkTypes::TagToRecord,
            (),
        )?;
    }

    // If collection specified, link to collection
    if let Some(collection_name) = input.collection_name {
        let collection_hash = anchor_hash(&format!("collection:{}", collection_name))?;
        create_link(
            collection_hash,
            entry_hash,
            LinkTypes::CollectionToRecord,
            (),
        )?;
    }

    Ok(action_hash)
}

/// Get a record by its key
#[hdk_extern]
pub fn get_record_by_key(key: String) -> ExternResult<Option<Record>> {
    let key_hash = anchor_hash(&key_anchor(&key))?;

    let links = get_links(
        GetLinksInputBuilder::try_new(key_hash, LinkTypes::KeyToRecord)?.build(),
    )?;

    if links.is_empty() {
        return Ok(None);
    }

    // Get the first (most recent) record
    let link = &links[0];
    get(link.target.clone(), GetOptions::default())
}

/// Get a record by its action hash
#[hdk_extern]
pub fn get_record(action_hash: ActionHash) -> ExternResult<Option<Record>> {
    get(action_hash, GetOptions::default())
}

/// Update a data record
#[hdk_extern]
pub fn update_record(input: UpdateRecordInput) -> ExternResult<ActionHash> {
    // Get the current record
    let current_record = get_record(input.action_hash.clone())?;

    match current_record {
        Some(record) => {
            if let Some(entry) = record.entry().as_option() {
                if let Ok(mut data_record) = DataRecord::try_from(entry.clone()) {
                    // Update fields
                    data_record.data = input.data;
                    data_record.updated_at = sys_time()?;
                    data_record.metadata = input.metadata;
                    data_record.tags = input.tags;

                    // Update the entry
                    let action_hash = update_entry(input.action_hash, &data_record)?;
                    Ok(action_hash)
                } else {
                    Err(wasm_error!(WasmErrorInner::Guest(
                        "Failed to deserialize data record".to_string()
                    )))
                }
            } else {
                Err(wasm_error!(WasmErrorInner::Guest(
                    "Record entry not found".to_string()
                )))
            }
        }
        None => Err(wasm_error!(WasmErrorInner::Guest(
            "Record not found".to_string()
        ))),
    }
}

/// Delete a data record
#[hdk_extern]
pub fn delete_record(action_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_entry(action_hash)
}

/// Get all records
#[hdk_extern]
pub fn get_all_records(_: ()) -> ExternResult<Vec<Record>> {
    let all_records_hash = anchor_hash(ALL_RECORDS_ANCHOR)?;

    let links = get_links(
        GetLinksInputBuilder::try_new(all_records_hash, LinkTypes::AllRecords)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Get records by data type
#[hdk_extern]
pub fn get_records_by_type(data_type: String) -> ExternResult<Vec<Record>> {
    let type_hash = anchor_hash(&type_anchor(&data_type))?;

    let links = get_links(
        GetLinksInputBuilder::try_new(type_hash, LinkTypes::TypeToRecord)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Get records by tag
#[hdk_extern]
pub fn get_records_by_tag(tag: String) -> ExternResult<Vec<Record>> {
    let tag_hash = anchor_hash(&tag_anchor(&tag))?;

    let links = get_links(
        GetLinksInputBuilder::try_new(tag_hash, LinkTypes::TagToRecord)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Search records with filters
#[hdk_extern]
pub fn search_records(input: SearchRecordsInput) -> ExternResult<Vec<Record>> {
    let limit = input.limit.unwrap_or(100) as usize;
    let mut results = Vec::new();

    // Determine the best starting point based on filters
    let candidate_records: Vec<Record> = if let Some(ref data_type) = input.data_type {
        get_records_by_type(data_type.clone())?
    } else if let Some(ref tags) = input.tags {
        if !tags.is_empty() {
            get_records_by_tag(tags[0].clone())?
        } else {
            get_all_records(())?
        }
    } else if let Some(ref collection_name) = input.collection_name {
        get_collection_records(collection_name.clone())?
    } else {
        get_all_records(())?
    };

    // Apply all filters
    for record in candidate_records {
        if results.len() >= limit {
            break;
        }

        if let Some(entry) = record.entry().as_option() {
            if let Ok(data_record) = DataRecord::try_from(entry.clone()) {
                // Apply type filter
                if let Some(ref data_type) = input.data_type {
                    if data_record.data_type != *data_type {
                        continue;
                    }
                }

                // Apply tags filter (record must have all specified tags)
                if let Some(ref tags) = input.tags {
                    let has_all_tags = tags.iter().all(|t| data_record.tags.contains(t));
                    if !has_all_tags {
                        continue;
                    }
                }

                results.push(record);
            }
        }
    }

    Ok(results)
}

/// Create a new collection
#[hdk_extern]
pub fn create_collection(collection: Collection) -> ExternResult<ActionHash> {
    // Create the collection entry
    let action_hash = create_entry(EntryTypes::Collection(collection.clone()))?;
    let entry_hash = hash_entry(&collection)?;

    // Link from all collections anchor
    let all_collections_hash = anchor_hash(ALL_COLLECTIONS_ANCHOR)?;
    create_link(
        all_collections_hash,
        entry_hash.clone(),
        LinkTypes::AllCollections,
        (),
    )?;

    // Link from agent to collection
    let agent_pubkey = agent_info()?.agent_latest_pubkey;
    create_link(
        agent_pubkey,
        entry_hash,
        LinkTypes::AgentToCollection,
        (),
    )?;

    Ok(action_hash)
}

/// Get a collection by name
#[hdk_extern]
pub fn get_collection(name: String) -> ExternResult<Option<Record>> {
    let all_collections = get_all_collections(())?;

    for record in all_collections {
        if let Some(entry) = record.entry().as_option() {
            if let Ok(collection) = Collection::try_from(entry.clone()) {
                if collection.name == name {
                    return Ok(Some(record));
                }
            }
        }
    }

    Ok(None)
}

/// Get all collections
#[hdk_extern]
pub fn get_all_collections(_: ()) -> ExternResult<Vec<Record>> {
    let all_collections_hash = anchor_hash(ALL_COLLECTIONS_ANCHOR)?;

    let links = get_links(
        GetLinksInputBuilder::try_new(all_collections_hash, LinkTypes::AllCollections)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Get all collections owned by the current agent
#[hdk_extern]
pub fn get_my_collections(_: ()) -> ExternResult<Vec<Record>> {
    let agent_pubkey = agent_info()?.agent_latest_pubkey;

    let links = get_links(
        GetLinksInputBuilder::try_new(agent_pubkey, LinkTypes::AgentToCollection)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Get all records in a collection
#[hdk_extern]
pub fn get_collection_records(collection_name: String) -> ExternResult<Vec<Record>> {
    let collection_hash = anchor_hash(&format!("collection:{}", collection_name))?;

    let links = get_links(
        GetLinksInputBuilder::try_new(collection_hash, LinkTypes::CollectionToRecord)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Delete a collection
#[hdk_extern]
pub fn delete_collection(action_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_entry(action_hash)
}

/// Create an index entry for fast lookups
#[hdk_extern]
pub fn create_index(entry: IndexEntry) -> ExternResult<ActionHash> {
    // Create the index entry
    let action_hash = create_entry(EntryTypes::IndexEntry(entry.clone()))?;
    let entry_hash = hash_entry(&entry)?;

    // Link from index name anchor
    let index_hash = anchor_hash(&format!("index:{}", entry.index_name))?;
    create_link(
        index_hash,
        entry_hash,
        LinkTypes::IndexNameToEntry,
        (),
    )?;

    Ok(action_hash)
}

/// Query an index by name and key
#[hdk_extern]
pub fn query_index(index_name: String) -> ExternResult<Vec<Record>> {
    let index_hash = anchor_hash(&format!("index:{}", index_name))?;

    let links = get_links(
        GetLinksInputBuilder::try_new(index_hash, LinkTypes::IndexNameToEntry)?.build(),
    )?;

    let mut records = Vec::new();
    for link in links {
        if let Some(record) = get(link.target, GetOptions::default())? {
            records.push(record);
        }
    }

    Ok(records)
}

/// Delete an index entry
#[hdk_extern]
pub fn delete_index(action_hash: ActionHash) -> ExternResult<ActionHash> {
    delete_entry(action_hash)
}

/// Get agent info (convenience wrapper)
#[hdk_extern]
pub fn whoami(_: ()) -> ExternResult<AgentInfo> {
    agent_info()
}

/// Get current system time (convenience wrapper)
#[hdk_extern]
pub fn now(_: ()) -> ExternResult<Timestamp> {
    sys_time()
}
