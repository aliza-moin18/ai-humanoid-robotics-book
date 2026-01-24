# Quickstart: Retrieval Service

## Prerequisites

- Python 3.11+
- Cohere API key
- Qdrant vector database access
- FastAPI-compatible environment

## Installation

1. Install required dependencies:
```bash
pip install fastapi cohere qdrant-client pydantic uvicorn
```

2. Set your Cohere API key as an environment variable:
```bash
export COHERE_API_KEY="your-cohere-api-key-here"
```

## Running the Service

1. Start the FastAPI server:
```bash
uvicorn retrieve:app --host 0.0.0.0 --port 8000
```

2. The service will be available at `http://localhost:8000`

## Making a Query

### Using curl:
```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "user_query": "What are ROS2 fundamentals?",
    "selected_text": "I am learning about robotics frameworks"
  }'
```

### Expected Response:
```json
{
  "chunks": [
    {
      "text": "ROS2 (Robot Operating System 2) is a flexible framework for writing robot applications...",
      "source_url": "https://book.example.com/ros2/introduction#fundamentals",
      "relevance_score": 0.92
    }
  ],
  "query_time": 0.245
}
```

## Configuration

The service connects to an existing Qdrant collection. Ensure your Qdrant connection details are properly configured in the environment.

## Testing

Run the tests to validate functionality:
```bash
pytest tests/
```