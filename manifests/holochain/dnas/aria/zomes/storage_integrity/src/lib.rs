use hdi::prelude::*;

/// Data record entry type for storing arbitrary data in the DHT
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct DataRecord {
    /// Unique key/identifier for this record
    pub key: String,
    /// The data content as a JSON string
    pub data: String,
    /// Type/category of the data (e.g., "config", "state", "cache")
    pub data_type: String,
    /// Timestamp when this record was created
    pub created_at: Timestamp,
    /// Timestamp of the last update
    pub updated_at: Timestamp,
    /// Optional metadata as JSON string
    pub metadata: Option<String>,
    /// Tags for categorization and search
    pub tags: Vec<String>,
}

/// Collection entry type for grouping related data records
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct Collection {
    /// Unique name for this collection
    pub name: String,
    /// Description of the collection
    pub description: String,
    /// Timestamp when this collection was created
    pub created_at: Timestamp,
    /// Owner of this collection
    pub owner: AgentPubKey,
    /// Optional schema definition as JSON string
    pub schema: Option<String>,
    /// Collection metadata as JSON string
    pub metadata: Option<String>,
}

/// Index entry for fast lookups and queries
#[hdk_entry_helper]
#[derive(Clone, PartialEq)]
pub struct IndexEntry {
    /// Index name/identifier
    pub index_name: String,
    /// Index key for lookup
    pub index_key: String,
    /// Value or reference associated with this index key
    pub value: String,
    /// Timestamp when this index entry was created
    pub created_at: Timestamp,
    /// Optional metadata
    pub metadata: Option<String>,
}

/// All entry types for the storage zome
#[hdk_entry_defs]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    #[entry_def(required_validations = 5)]
    DataRecord(DataRecord),
    #[entry_def(required_validations = 5)]
    Collection(Collection),
    #[entry_def(required_validations = 3)]
    IndexEntry(IndexEntry),
}

/// Link types for the storage zome
#[hdk_link_types]
pub enum LinkTypes {
    /// Links from collection to data records
    CollectionToRecord,
    /// Links from key to data record
    KeyToRecord,
    /// Links from data type to records
    TypeToRecord,
    /// Links from tag to records
    TagToRecord,
    /// Links from index name to index entries
    IndexNameToEntry,
    /// Links from agent to their collections
    AgentToCollection,
    /// All collections anchor
    AllCollections,
    /// All records anchor
    AllRecords,
}

/// Validation rules for DataRecord
pub fn validate_data_record(record: &DataRecord) -> ExternResult<ValidateCallbackResult> {
    // Key must not be empty
    if record.key.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Record key cannot be empty".to_string(),
        ));
    }

    // Key length constraints
    if record.key.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Record key cannot exceed 256 characters".to_string(),
        ));
    }

    // Data must not be empty
    if record.data.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Record data cannot be empty".to_string(),
        ));
    }

    // Data must be valid JSON
    if serde_json::from_str::<serde_json::Value>(&record.data).is_err() {
        return Ok(ValidateCallbackResult::Invalid(
            "Record data must be valid JSON".to_string(),
        ));
    }

    // Data size limit (1MB)
    if record.data.len() > 1_048_576 {
        return Ok(ValidateCallbackResult::Invalid(
            "Record data cannot exceed 1MB".to_string(),
        ));
    }

    // Data type must not be empty
    if record.data_type.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Data type cannot be empty".to_string(),
        ));
    }

    // Updated timestamp must not be before created timestamp
    if record.updated_at < record.created_at {
        return Ok(ValidateCallbackResult::Invalid(
            "Updated timestamp cannot be before created timestamp".to_string(),
        ));
    }

    // Validate metadata is valid JSON if present
    if let Some(ref metadata) = record.metadata {
        if !metadata.is_empty() {
            if serde_json::from_str::<serde_json::Value>(metadata).is_err() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Record metadata must be valid JSON".to_string(),
                ));
            }
        }
    }

    // Validate tags are non-empty
    for tag in &record.tags {
        if tag.trim().is_empty() {
            return Ok(ValidateCallbackResult::Invalid(
                "Tag cannot be empty".to_string(),
            ));
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validation rules for Collection
pub fn validate_collection(collection: &Collection) -> ExternResult<ValidateCallbackResult> {
    // Name must not be empty
    if collection.name.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Collection name cannot be empty".to_string(),
        ));
    }

    // Name length constraints
    if collection.name.len() > 100 {
        return Ok(ValidateCallbackResult::Invalid(
            "Collection name cannot exceed 100 characters".to_string(),
        ));
    }

    // Validate name format (alphanumeric + underscore + hyphen + dot)
    if !collection
        .name
        .chars()
        .all(|c| c.is_alphanumeric() || c == '_' || c == '-' || c == '.')
    {
        return Ok(ValidateCallbackResult::Invalid(
            "Collection name can only contain alphanumeric characters, underscores, hyphens, and dots"
                .to_string(),
        ));
    }

    // Description must not be empty
    if collection.description.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Collection description cannot be empty".to_string(),
        ));
    }

    // Validate schema is valid JSON if present
    if let Some(ref schema) = collection.schema {
        if !schema.is_empty() {
            if serde_json::from_str::<serde_json::Value>(schema).is_err() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Collection schema must be valid JSON".to_string(),
                ));
            }
        }
    }

    // Validate metadata is valid JSON if present
    if let Some(ref metadata) = collection.metadata {
        if !metadata.is_empty() {
            if serde_json::from_str::<serde_json::Value>(metadata).is_err() {
                return Ok(ValidateCallbackResult::Invalid(
                    "Collection metadata must be valid JSON".to_string(),
                ));
            }
        }
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Validation rules for IndexEntry
pub fn validate_index_entry(entry: &IndexEntry) -> ExternResult<ValidateCallbackResult> {
    // Index name must not be empty
    if entry.index_name.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Index name cannot be empty".to_string(),
        ));
    }

    // Index key must not be empty
    if entry.index_key.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Index key cannot be empty".to_string(),
        ));
    }

    // Value must not be empty
    if entry.value.trim().is_empty() {
        return Ok(ValidateCallbackResult::Invalid(
            "Index value cannot be empty".to_string(),
        ));
    }

    // Key and value length constraints
    if entry.index_key.len() > 256 {
        return Ok(ValidateCallbackResult::Invalid(
            "Index key cannot exceed 256 characters".to_string(),
        ));
    }

    if entry.value.len() > 1024 {
        return Ok(ValidateCallbackResult::Invalid(
            "Index value cannot exceed 1024 characters".to_string(),
        ));
    }

    Ok(ValidateCallbackResult::Valid)
}

/// Main validation callback
#[hdk_extern]
pub fn validate(op: Op) -> ExternResult<ValidateCallbackResult> {
    match op.flattened::<EntryTypes, LinkTypes>()? {
        FlatOp::StoreEntry(OpEntry::CreateEntry { app_entry, .. }) => match app_entry {
            EntryTypes::DataRecord(record) => validate_data_record(&record),
            EntryTypes::Collection(collection) => validate_collection(&collection),
            EntryTypes::IndexEntry(entry) => validate_index_entry(&entry),
        },
        FlatOp::StoreEntry(OpEntry::UpdateEntry {
            app_entry,
            original_action_hash,
            ..
        }) => {
            // For updates, ensure the original entry exists and validate the new entry
            match app_entry {
                EntryTypes::DataRecord(record) => {
                    let _ = must_get_action(original_action_hash)?;
                    validate_data_record(&record)
                }
                EntryTypes::Collection(collection) => {
                    let _ = must_get_action(original_action_hash)?;
                    validate_collection(&collection)
                }
                EntryTypes::IndexEntry(entry) => {
                    let _ = must_get_action(original_action_hash)?;
                    validate_index_entry(&entry)
                }
            }
        }
        FlatOp::StoreEntry(OpEntry::DeleteEntry {
            original_action_hash,
            ..
        }) => {
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
                LinkTypes::CollectionToRecord
                | LinkTypes::KeyToRecord
                | LinkTypes::TypeToRecord
                | LinkTypes::TagToRecord
                | LinkTypes::IndexNameToEntry
                | LinkTypes::AgentToCollection
                | LinkTypes::AllCollections
                | LinkTypes::AllRecords => Ok(ValidateCallbackResult::Valid),
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
