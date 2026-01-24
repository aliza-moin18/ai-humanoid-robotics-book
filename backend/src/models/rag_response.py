"""
Pydantic model for RAGAgentResponse
Contains the processed response from the RAG agent.
"""

from pydantic import BaseModel
from typing import Optional, List
import uuid
from datetime import datetime


class RAGAgentResponse(BaseModel):
    """Contains the processed response from the RAG agent."""
    response: str
    sources: Optional[List[dict]] = []  # Using dict for SourceReference until that model is created
    session_id: Optional[str] = None
    query_id: str = str(uuid.uuid4())
    timestamp: str = datetime.now().isoformat()