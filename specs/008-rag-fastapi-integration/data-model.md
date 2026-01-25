# Data Model: RAG-FastAPI Integration

## Overview
This document defines the data structures and models for the RAG-FastAPI integration feature.

## Entities

### 1. Query Request
Represents a user query sent from the frontend to the backend.

**Fields:**
- `query` (string, required): The text of the user's query
- `session_id` (string, optional): Identifier for the conversation session
- `metadata` (object, optional): Additional information about the query context

**Validation rules:**
- Query must be between 1 and 2000 characters
- Session ID, if provided, must be a valid UUID format

**Example:**
```json
{
  "query": "How does robot localization work?",
  "session_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
  "metadata": {
    "timestamp": "2023-10-01T12:00:00Z",
    "user_id": "user123"
  }
}
```

### 2. RAG Agent Response
Contains the processed response from the RAG agent, including the answer and supporting information.

**Fields:**
- `response` (string, required): The agent's response to the query
- `sources` (array of objects, optional): List of sources used to generate the response
- `session_id` (string, optional): Identifier for the conversation session
- `query_id` (string, required): Unique identifier for this query-response pair
- `timestamp` (string, required): ISO 8601 timestamp of response generation

**Validation rules:**
- Response must not exceed 10,000 characters
- Sources, if provided, must be an array of source objects
- Query ID must be unique for each response

**Example:**
```json
{
  "response": "Robot localization is the process of determining a robot's position and orientation in a given environment...",
  "sources": [
    {
      "title": "Chapter 5: Robot Localization Techniques",
      "url": "/module-3/ai-robot-brain/chapter-5-localization",
      "page_number": 127,
      "relevance_score": 0.92
    }
  ],
  "session_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
  "query_id": "q1r2s3t4-u5v6-7890-1234-567890fedcba",
  "timestamp": "2023-10-01T12:00:05Z"
}
```

### 3. Source Reference
Information about documents or resources used by the RAG system to generate a response.

**Fields:**
- `title` (string, required): Title of the source document
- `url` (string, required): URL or path to the source
- `page_number` (integer, optional): Page number where information was found
- `relevance_score` (float, required): Score indicating relevance to the query (0.0 to 1.0)
- `content_snippet` (string, optional): Brief excerpt from the source

**Validation rules:**
- Title must be between 1 and 200 characters
- URL must be a valid relative or absolute URL
- Relevance score must be between 0.0 and 1.0

**Example:**
```json
{
  "title": "Introduction to SLAM Algorithms",
  "url": "/module-3/ai-robot-brain/chapter-4-slam",
  "page_number": 98,
  "relevance_score": 0.87,
  "content_snippet": "Simultaneous Localization and Mapping (SLAM) is a computational problem for constructing..."
}
```

### 4. Error Response
Standardized error response format for API errors.

**Fields:**
- `error_code` (string, required): Machine-readable error code
- `message` (string, required): Human-readable error message
- `details` (object, optional): Additional error details
- `timestamp` (string, required): ISO 8601 timestamp of error

**Validation rules:**
- Error code must be alphanumeric with hyphens/underscores
- Message must be between 1 and 500 characters

**Example:**
```json
{
  "error_code": "AGENT_UNAVAILABLE",
  "message": "The RAG agent is temporarily unavailable",
  "details": {
    "retry_after": 30
  },
  "timestamp": "2023-10-01T12:00:05Z"
}
```

## State Transitions

### Query Processing States
1. **Received**: Query has been received by the API endpoint
2. **Processing**: Query is being processed by the RAG agent
3. **Completed**: Response has been generated and is ready to return
4. **Failed**: An error occurred during processing

## Relationships
- A `Query Request` generates one `RAG Agent Response`
- A `RAG Agent Response` may contain multiple `Source Reference` objects
- Multiple queries can belong to the same `session_id` for conversation continuity