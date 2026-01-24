# Quickstart: RAG Agent with OpenAI Agents SDK

## Setup

1. **Install Dependencies**
   ```bash
   pip install openai python-dotenv requests
   ```

2. **Environment Configuration**
   Create a `.env` file with:
   ```env
   OPENAI_API_KEY=your_openai_api_key_here
   RETRIEVAL_API_URL=http://localhost:8000/query  # URL to existing retrieval API
   ```

## Usage

### Basic Query
```python
from agent import RetrievalAgent

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
pytest tests/test_agent.py
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