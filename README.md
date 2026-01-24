# RAG Agent with OpenAI Agents SDK

This module implements a Retrieval-Augmented Generation (RAG) agent that queries book content using the OpenAI API and provides answers with source citations.

## Setup

1. **Install Dependencies**
   ```bash
   pip install -r requirements.txt
   ```

2. **Environment Configuration**
   Create a `.env` file with:
   ```env
   OPENAI_API_KEY=your_openai_api_key_here
   RETRIEVAL_API_URL=http://localhost:8000/query  # URL to existing retrieval API
   QUERY_MAX_LENGTH=1000  # Maximum length of queries (optional, defaults to 1000)
   TIMEOUT_SECONDS=30     # Timeout for API calls (optional, defaults to 30)
   CITATION_COUNT=5       # Number of citations to return (optional, defaults to 5)
   ```

## Usage

### Basic Query
```python
from backend.agent import RetrievalAgent

# Initialize the agent
agent = RetrievalAgent()

# Process a query
response = agent.process_query("What are the key principles of robotics?")
print(response.answer)
print(response.citations)
```

### Query with Context (Follow-up)
```python
# For follow-up questions, include previous context
response = agent.process_query(
    "Can you elaborate on that?",
    previous_context={
        "query": "What are the key principles of robotics?",
        "response": "The key principles include autonomy, interactivity, mobility..."
    }
)
```

## Running Tests
```bash
pytest tests/
```

## Key Components

- **RetrievalAgent**: Main class that orchestrates the RAG process
- **process_query()**: Primary method that takes user input and returns answers with citations
- **Integration with existing API**: Connects to the Spec-2 retrieval API for document chunks
- **Citation generation**: Automatically generates 3-5 source citations with excerpts and URLs

## Architecture Overview

1. User submits a query to the RetrievalAgent
2. Agent calls the existing retrieval API to get relevant content chunks
3. Agent uses OpenAI to generate an answer based only on retrieved content
4. Agent formats the response with 3-5 source citations
5. For follow-up queries, previous context is included in the prompt