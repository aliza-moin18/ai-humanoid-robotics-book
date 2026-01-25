# API Contract: RAG Agent Interface

## Overview
This contract defines the interface for the Retrieval Augmented Generation (RAG) agent that processes user queries against book content.

## Core Methods

### process_query(user_query, previous_context=None)
**Description**: Process a user query and return an answer with source citations

**Parameters**:
- `user_query` (string, required): Natural language question about book content (max 1000 characters)
- `previous_context` (object, optional): Previous conversation context for follow-up queries
  - `query` (string): The previous user query
  - `response` (string): The previous agent response

**Returns**:
- `answer` (string): The agent's response based only on retrieved content
- `citations` (array): List of 3-5 source citations
  - `text_excerpt` (string): Excerpt from the source document
  - `source_url` (string): URL to the original document
  - `relevance_score` (float): Relevance score of the citation (0-1)

**Errors**:
- `InvalidQueryError`: When the query exceeds length limits or is malformed
- `RetrievalError`: When the retrieval API is unavailable or returns no results
- `GenerationError`: When the LLM fails to generate a response

**Example Request**:
```python
response = agent.process_query(
    "What are the ethical considerations in robotics?",
    previous_context={
        "query": "What are the key principles of robotics?",
        "response": "The key principles include autonomy, interactivity, mobility..."
    }
)
```

**Example Response**:
```json
{
  "answer": "Ethical considerations in robotics include ensuring safety, maintaining human oversight, considering job displacement, and addressing privacy concerns.",
  "citations": [
    {
      "text_excerpt": "Safety is paramount in robotic design...",
      "source_url": "https://book.example.com/chapter-5",
      "relevance_score": 0.92
    },
    {
      "text_excerpt": "Human oversight must be maintained in autonomous systems...",
      "source_url": "https://book.example.com/chapter-7",
      "relevance_score": 0.87
    },
    {
      "text_excerpt": "Privacy concerns arise when robots collect personal data...",
      "source_url": "https://book.example.com/chapter-9",
      "relevance_score": 0.81
    }
  ]
}
```

## Integration Points

### Retrieval API Integration
The agent integrates with the existing Spec-2 retrieval API to fetch relevant content chunks:

- **Endpoint**: Configured via `RETRIEVAL_API_URL` environment variable
- **Method**: POST /query
- **Request**: `{ "query": "user query text", "top_k": 5 }`
- **Response**: Array of content chunks with text and metadata

## Constraints
- No hallucination: Answers must be based only on retrieved content
- Source citations: Must provide 3-5 citations with excerpts and URLs
- Timeout: API calls must complete within 30 seconds
- Query length: Limited to 1000 characters