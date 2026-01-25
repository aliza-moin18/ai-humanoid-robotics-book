# Retrieval Service for AI Robotics Book

This service provides a retrieval API for querying book content using Cohere embeddings and Qdrant vector search.

## Features

- FastAPI endpoint for querying book content
- Support for general queries and queries with selected text context
- Top-5 similarity search results
- Returns chunks with text, source URL, and relevance score
- Cohere embeddings for semantic search
- Qdrant vector database for efficient similarity search

## Setup

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Set your Cohere API key:
```bash
export COHERE_API_KEY="your-cohere-api-key-here"
```

3. Ensure Qdrant is running and the book content is indexed in a collection named "book_chunks"

## Usage

Start the service:
```bash
python retrieve.py
```

Or with uvicorn:
```bash
uvicorn retrieve:app --host 0.0.0.0 --port 8000
```

Query the service:
```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "user_query": "What are ROS2 fundamentals?",
    "selected_text": "I am learning about robotics frameworks"
  }'
```

## API Endpoints

- `POST /query` - Query book content
- `GET /health` - Health check

## Configuration

The service can be configured via environment variables:
- `COHERE_API_KEY` - Your Cohere API key
- `QDRANT_URL` - Qdrant server URL (defaults to localhost)
- `QDRANT_PORT` - Qdrant server port (defaults to 6333)
- `COLLECTION_NAME` - Qdrant collection name (defaults to "book_chunks")