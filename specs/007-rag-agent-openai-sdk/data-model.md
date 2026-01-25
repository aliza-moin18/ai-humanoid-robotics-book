# Data Model: RAG Agent with OpenAI Agents SDK

## Entities

### Query
- **Definition**: A natural language question submitted by the user about book content
- **Fields**:
  - `text` (string): The actual query text (max 1000 characters)
  - `timestamp` (datetime): When the query was submitted
  - `user_id` (string, optional): Identifier for the user (for follow-up context)
- **Validation**: 
  - Length ≤ 1000 characters
  - Non-empty text
- **Relationships**: Associated with multiple RetrievedContent chunks

### RetrievedContent
- **Definition**: Document chunks retrieved from Qdrant database based on the user's query
- **Fields**:
  - `id` (string): Unique identifier for the chunk
  - `content` (string): The actual text content of the chunk
  - `source_url` (string): URL reference to the original document
  - `score` (float): Relevance score from the retrieval system
  - `metadata` (dict): Additional metadata about the source
- **Validation**:
  - Content must be non-empty
  - Score must be between 0 and 1
- **Relationships**: Belongs to a Query, referenced in AgentResponse

### AgentResponse
- **Definition**: The AI-generated answer with source citations based on retrieved content
- **Fields**:
  - `answer` (string): The generated answer based on retrieved content
  - `citations` (list of Citation objects): References to source documents
  - `timestamp` (datetime): When the response was generated
  - `query_id` (string): Reference to the original query
- **Validation**:
  - Answer must be based only on retrieved content (no hallucination)
  - Must contain 3-5 citations
- **Relationships**: Associated with a Query and multiple Citations

### Citation
- **Definition**: Reference to the original document chunk with text excerpt and URL
- **Fields**:
  - `text_excerpt` (string): The relevant text excerpt from the source
  - `source_url` (string): URL to the original document
  - `chunk_id` (string): Reference to the RetrievedContent chunk
  - `relevance_score` (float): How relevant this citation is to the answer
- **Validation**:
  - Text excerpt must exist in the corresponding RetrievedContent
  - Source URL must be valid
- **Relationships**: Links AgentResponse to RetrievedContent

### ConversationContext
- **Definition**: Maintains basic context for follow-up queries
- **Fields**:
  - `previous_query` (string): The previous query in the conversation
  - `previous_response` (string): The previous response
  - `session_id` (string): Identifier for the conversation session
  - `timestamp` (datetime): When this context was created
- **Validation**:
  - Session ID must be valid
  - Timestamp must be recent (within reasonable timeframe)
- **Relationships**: Links related Query and AgentResponse objects