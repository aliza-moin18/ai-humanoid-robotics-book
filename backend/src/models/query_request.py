"""
Pydantic model for QueryRequest
Represents a user query sent from the frontend to the backend.
"""

from pydantic import BaseModel
from typing import Optional, Dict, Any


class QueryRequest(BaseModel):
    """Represents a user query sent from the frontend to the backend."""
    query: str
    session_id: Optional[str] = None
    metadata: Optional[Dict[str, Any]] = {}