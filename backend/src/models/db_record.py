"""
Model for representing an entry in Qdrant containing the embedding vector and associated metadata.
"""
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, Any, Optional


@dataclass
class VectorDatabaseRecord:
    """
    Represents an entry in Qdrant containing the embedding vector and associated metadata 
    for retrieval and search operations.
    """
    id: str
    embedding_id: str
    vector: list  # The embedding vector
    payload: Dict[str, Any]  # Additional metadata stored with the vector in Qdrant
    created_at: Optional[datetime] = None
    
    def __post_init__(self):
        """Validate the VectorDatabaseRecord instance after initialization."""
        if not self.embedding_id:
            raise ValueError("embedding_id is required")
        
        if not self.vector:
            raise ValueError("vector is required")
        
        if self.payload is None:
            raise ValueError("payload is required")
        
        if self.created_at is None:
            self.created_at = datetime.now()