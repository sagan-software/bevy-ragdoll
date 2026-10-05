//! Validation for GLB headers, chunk ordering, and the supported glTF profile.

use serde_json::Value;

use super::{GltfRigError, GltfRigValidationError, invalid};

/// Four-byte chunk type for the first GLB JSON chunk.
pub(super) const JSON_CHUNK: u32 = 0x4E4F_534A;
/// Four-byte chunk type for the optional GLB binary chunk.
pub(super) const BIN_CHUNK: u32 = 0x004E_4942;

/// Parses the GLB's JSON document after validating its binary container.
pub(super) fn parse_document(bytes: &[u8]) -> Result<Value, GltfRigError> {
    let json_bytes = json_chunk(bytes)?;
    Ok(serde_json::from_slice(json_bytes)?)
}

/// The JSON bytes in a structurally valid GLB 2.0 container.
pub(super) fn json_chunk(bytes: &[u8]) -> Result<&[u8], GltfRigError> {
    // Validate fixed header fields before reading any chunk bytes.
    validate_glb_header(bytes)?;
    // Locate the required JSON chunk and validate every remaining chunk.
    let range = json_chunk_range(bytes)?;
    validate_remaining_chunks(bytes, range.end).map_err(GltfRigError::Invalid)?;
    bytes
        .get(range)
        .ok_or_else(|| invalid("the JSON chunk range is invalid"))
}

/// Checks the fixed GLB header and requires the declared length to match the input.
fn validate_glb_header(bytes: &[u8]) -> Result<(), GltfRigError> {
    // Check the fixed identifier and version before trusting declared lengths.
    if bytes.get(0..4) != Some(b"glTF".as_slice()) {
        return Err(invalid("the header magic is not glTF"));
    }
    if read_u32(bytes, 4) != Some(2) {
        return Err(invalid("the GLB container version is not 2"));
    }
    // Convert and compare the total length before reading the first chunk header.
    let declared_length = read_u32(bytes, 8)
        .and_then(|length| usize::try_from(length).ok())
        .ok_or_else(|| invalid("the header has no valid total length"))?;
    if declared_length != bytes.len() {
        return Err(invalid("the declared total length does not match the file"));
    }
    Ok(())
}

/// Returns the required first JSON chunk's byte range after checking its header.
fn json_chunk_range(bytes: &[u8]) -> Result<std::ops::Range<usize>, GltfRigError> {
    // Read and align the first chunk size before computing its end position.
    let json_length = read_u32(bytes, 12)
        .and_then(|length| usize::try_from(length).ok())
        .ok_or_else(|| invalid("the first chunk has no length"))?;
    if read_u32(bytes, 16) != Some(JSON_CHUNK) {
        return Err(invalid("the first chunk is not JSON"));
    }
    if json_length % 4 != 0 {
        return Err(invalid("the JSON chunk length is not four-byte aligned"));
    }
    // Checked addition prevents malformed lengths from wrapping the slice bound.
    let end = 20_usize
        .checked_add(json_length)
        .filter(|end| *end <= bytes.len())
        .ok_or_else(|| invalid("the JSON chunk is truncated"))?;
    Ok(20..end)
}

/// Validates chunk bounds, ordering, and alignment while leaving payloads unread.
pub(super) fn validate_remaining_chunks(
    bytes: &[u8],
    mut cursor: usize,
) -> Result<(), GltfRigValidationError> {
    let mut chunk_number = 1;
    let mut has_seen_bin = false;
    let mut has_seen_unknown = false;
    // Parse each chunk header before trusting its declared payload size.
    while cursor < bytes.len() {
        let chunk = remaining_chunk(bytes, cursor)?;
        // Reject duplicate JSON and invalid BIN ordering before advancing the cursor.
        validate_remaining_chunk_type(
            chunk.kind,
            chunk_number,
            &mut has_seen_bin,
            &mut has_seen_unknown,
        )?;
        cursor = chunk.end;
        chunk_number += 1;
    }
    Ok(())
}

/// Describes one validated chunk header and payload range.
struct RemainingChunk {
    /// The four-byte GLB chunk type from its header.
    kind: u32,
    /// The first byte after its validated payload.
    end: usize,
}

/// Reads one remaining chunk header and checks aligned, in-bounds payload length.
fn remaining_chunk(bytes: &[u8], cursor: usize) -> Result<RemainingChunk, GltfRigValidationError> {
    // Parse the complete header before checking its declared payload range.
    let header = parse_chunk_header(bytes, cursor)?;
    // Check payload bounds independently from header parsing.
    let end = validate_chunk_payload_end(bytes, header.payload_start, header.length)?;
    Ok(RemainingChunk {
        kind: header.kind,
        end,
    })
}

/// Parsed chunk fields needed to validate its payload and ordering.
struct ChunkHeader {
    /// The first payload byte following this fixed-size header.
    payload_start: usize,
    /// The payload length in bytes after conversion to `usize`.
    length: usize,
    /// The chunk type used to enforce GLB ordering.
    kind: u32,
}

/// Reads and validates the fixed-size header fields at `cursor`.
fn parse_chunk_header(bytes: &[u8], cursor: usize) -> Result<ChunkHeader, GltfRigValidationError> {
    // Read exactly eight header bytes before interpreting the length and type.
    let header = chunk_header_bytes(bytes, cursor)?;
    let length = chunk_length(header)?;
    let kind = read_u32(header, 4)
        .ok_or_else(|| GltfRigValidationError::new("a GLB chunk type is missing"))?;
    // A chunk's byte length must preserve the required four-byte alignment.
    if length % 4 != 0 {
        return Err("a GLB chunk length is not four-byte aligned".into());
    }
    // Header access proves this addition stays in the source slice's address range.
    let payload_start = cursor + header.len();
    Ok(ChunkHeader {
        payload_start,
        length,
        kind,
    })
}

/// Reads exactly one in-bounds GLB chunk header.
fn chunk_header_bytes(bytes: &[u8], cursor: usize) -> Result<&[u8], GltfRigValidationError> {
    let end = cursor
        .checked_add(8)
        .filter(|end| *end <= bytes.len())
        .ok_or_else(|| GltfRigValidationError::new("a GLB chunk header is truncated"))?;
    bytes
        .get(cursor..end)
        .ok_or_else(|| GltfRigValidationError::new("a GLB chunk header is truncated"))
}

/// Converts a chunk's little-endian length into the platform's address size.
fn chunk_length(header: &[u8]) -> Result<usize, GltfRigValidationError> {
    let length = read_u32(header, 0)
        .ok_or_else(|| GltfRigValidationError::new("a GLB chunk length is missing"))?;
    usize::try_from(length).map_err(|error| {
        GltfRigValidationError::new(format!(
            "a GLB chunk length does not fit this platform: {error}"
        ))
    })
}

/// Returns a chunk payload end only when checked addition stays inside the input.
fn validate_chunk_payload_end(
    bytes: &[u8],
    header_end: usize,
    length: usize,
) -> Result<usize, GltfRigValidationError> {
    header_end
        .checked_add(length)
        .filter(|end| *end <= bytes.len())
        .ok_or_else(|| GltfRigValidationError::new("a GLB chunk payload is truncated"))
}

/// Enforces unique JSON and optional second-position BIN chunk semantics.
fn validate_remaining_chunk_type(
    kind: u32,
    chunk_number: usize,
    has_seen_bin: &mut bool,
    has_seen_unknown: &mut bool,
) -> Result<(), GltfRigValidationError> {
    // The required JSON chunk may appear only once in a GLB container.
    if kind == JSON_CHUNK {
        return Err("the GLB contains more than one JSON chunk".into());
    }
    // BIN is optional and legal only immediately after JSON.
    if kind == BIN_CHUNK {
        if chunk_number != 1 || *has_seen_bin || *has_seen_unknown {
            return Err("the BIN chunk is not the optional second chunk".into());
        }
        *has_seen_bin = true;
    } else {
        // Unknown chunks are accepted, but their position prevents a later BIN chunk.
        *has_seen_unknown = true;
    }
    Ok(())
}

/// Reads a little-endian word at `offset` when four bytes remain.
pub(super) fn read_u32(bytes: &[u8], offset: usize) -> Option<u32> {
    let end = offset.checked_add(4)?;
    let word = bytes.get(offset..end)?;
    let word: [u8; 4] = word.try_into().ok()?;
    Some(u32::from_le_bytes(word))
}

/// Checks glTF version metadata and extension requirements for this importer.
pub(super) fn validate_asset_profile(document: &Value) -> Result<(), GltfRigValidationError> {
    // Validate asset metadata before following document references.
    validate_asset_version(document)?;
    // Reject required extensions before this importer reads the document body.
    validate_required_extensions(document)
}

/// Checks the required glTF asset version and this importer's minimum-version policy.
fn validate_asset_version(document: &Value) -> Result<(), GltfRigValidationError> {
    // Require glTF asset metadata separately from the already checked GLB version.
    let asset = document.get("asset").and_then(Value::as_object);
    let version = asset
        .and_then(|asset| asset.get("version"))
        .and_then(Value::as_str);
    if version != Some("2.0") {
        return Err("the glTF asset version is not 2.0".into());
    }
    // This profile accepts no minimum version or exact 2.0; lower valid minima are unsupported.
    if let Some(minimum_value) = asset.and_then(|asset| asset.get("minVersion")) {
        let minimum = minimum_value.as_str().ok_or_else(|| {
            GltfRigValidationError::new("the glTF asset minimum version is not a string")
        })?;
        if minimum != "2.0" {
            return Err("the glTF asset minimum version is not 2.0".into());
        }
    }
    Ok(())
}

/// Rejects malformed or unsupported required-extension declarations.
fn validate_required_extensions(document: &Value) -> Result<(), GltfRigValidationError> {
    // An omitted list imposes no extension requirement.
    let Some(required_value) = document.get("extensionsRequired") else {
        return Ok(());
    };
    // Require an array before inspecting extension names.
    let required = required_value.as_array().ok_or_else(|| {
        GltfRigValidationError::new("the glTF extensionsRequired member is not an array")
    })?;
    // Classify schema-invalid arrays before checking extension support.
    let error = if required.is_empty() {
        "the glTF extensionsRequired member is empty"
    } else if required.iter().any(|extension| !extension.is_string()) {
        "the glTF extensionsRequired member contains a non-string value"
    } else {
        "the glTF asset requires unsupported extensions"
    };
    // No glTF extensions change this importer's document interpretation.
    Err(error.into())
}
