"""
Pydantic model for SourceReference
Information about documents or resources used by the RAG system.
"""

from pydantic import BaseModel, validator
from typing import Optional


class SourceReference(BaseModel):
    """Information about documents or resources used by the RAG system."""
    title: str
    url: str
    page_number: Optional[int] = None
    relevance_score: float  # Between 0.0 and 1.0
    content_snippet: Optional[str] = None

    @validator('relevance_score')
    def validate_relevance_score(cls, v):
        if v < 0.0 or v > 1.0:
            raise ValueError('Relevance score must be between 0.0 and 1.0')
        return v