"""
Model for representing a segment of book content that has been cleaned and prepared for embedding.
"""
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Any, Optional


@dataclass
class TextChunk:
    """
    Represents a segment of book content that has been cleaned and prepared for embedding,
    with associated metadata like source location and chunk ID.
    """
    id: str
    book_content_id: str
    content: str
    chunk_index: int
    source_url: str
    source_location: str = ""
    token_count: Optional[int] = None
    created_at: Optional[datetime] = None
    metadata: Optional[Dict[str, Any]] = None
    
    def __post_init__(self):
        """Validate the TextChunk instance after initialization."""
        if not self.content:
            raise ValueError("content is required")
        
        if self.chunk_index < 0:
            raise ValueError("chunk_index must be a non-negative integer")
        
        if self.token_count is not None and self.token_count <= 0:
            raise ValueError("token_count must be positive")
        
        if not self.book_content_id:
            raise ValueError("book_content_id is required")
        
        if self.created_at is None:
            self.created_at = datetime.now()
        
        if self.metadata is None:
            self.metadata = {}