# RAG Agent API Documentation

This document provides information about the RAG Agent API endpoints and how to use them.

## Base URL

The API is served at: `http://localhost:8000`

## Endpoints

### 1. Query Endpoint

Submit a query to the RAG agent and receive a contextual response.

- **URL**: `/query`
- **Method**: `POST`
- **Content-Type**: `application/json`

#### Request Body

```json
{
  "query": "Your question or query here",
  "session_id": "optional session identifier",
  "metadata": {
    "user_id": "optional user identifier",
    "timestamp": "optional timestamp"
  }
}
```

#### Example Request

```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How does robot localization work?",
    "session_id": "abc123"
  }'
```

#### Response

```json
{
  "response": "The response from the RAG agent",
  "sources": [
    {
      "title": "Title of the source document",
      "url": "URL or path to the source",
      "page_number": 123,
      "relevance_score": 0.95,
      "content_snippet": "A brief excerpt from the source"
    }
  ],
  "session_id": "abc123",
  "query_id": "generated-unique-query-id",
  "timestamp": "2023-01-01T00:00:00"
}
```

### 2. Health Check Endpoint

Check the health status of the API.

- **URL**: `/health`
- **Method**: `GET`

#### Example Request

```bash
curl http://localhost:8000/health
```

#### Response

```json
{
  "status": "healthy",
  "timestamp": "2023-01-01T00:00:00"
}
```

## Error Handling

The API returns appropriate HTTP status codes and error responses:

- `400 Bad Request`: When the request format is invalid
- `422 Unprocessable Entity`: When validation fails
- `429 Too Many Requests`: When rate limit is exceeded
- `500 Internal Server Error`: When an unexpected error occurs

### Error Response Format

```json
{
  "error_code": "ERROR_CODE",
  "message": "Human-readable error message",
  "details": {},
  "timestamp": "2023-01-01T00:00:00"
}
```

## Rate Limiting

The API implements rate limiting to prevent abuse. By default, clients are limited to 5 requests per minute per IP address.

## Frontend Integration

To integrate with the frontend, use the API service located at `book/frontend/src/services/apiService.js`.

Example usage:

```javascript
import ApiService from './services/apiService';

// Send a query
const queryRequest = {
  query: "How does robot localization work?",
  session_id: "user-session-id"
};

try {
  const response = await ApiService.sendQuery(queryRequest);
  console.log("Response:", response);
} catch (error) {
  console.error("Error sending query:", error);
}
```