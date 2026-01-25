# Data Model: Retrieval Service

## Entities

### QueryRequest
Represents the input to the retrieval service.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| user_query | string | The main query text from the user | Required, min length 1, max length 1000 |
| selected_text | string? | Optional selected/highlighted text for context | Optional, max length 2000 |

### QueryResponse
Represents the output from the retrieval service.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| chunks | Chunk[] | Array of relevant content chunks | Required, min 0, max 5 elements |
| query_time | float | Time taken to process the query in seconds | Required, positive value |

### Chunk
Represents a single content chunk returned by the retrieval service.

| Field | Type | Description | Validation |
|-------|------|-------------|------------|
| text | string | Full text content of the chunk | Required, min length 1 |
| source_url | string | URL or section reference where the chunk originated | Required, valid URL format |
| relevance_score | float | Numerical score indicating similarity to the query | Required, between 0 and 1 |

## Relationships

- QueryRequest is processed by RetrievalService to produce QueryResponse
- QueryResponse contains multiple Chunk objects (1 to 5)

## State Transitions

The retrieval system has no persistent state - each query is processed independently.

## Validation Rules

1. QueryRequest.user_query must be between 1 and 1000 characters
2. QueryRequest.selected_text, if provided, must be between 1 and 2000 characters
3. QueryResponse.chunks must contain between 0 and 5 Chunk objects
4. Chunk.relevance_score must be between 0 and 1 (inclusive)
5. Chunk.text must not be empty
6. Chunk.source_url must be a valid URL format