"""
Model for representing book content extracted from URLs.
"""
from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional


@dataclass
class BookContent:
    """
    Represents the text content extracted from book URLs, including metadata such as 
    source URL, page/chapter information, and content structure.
    """
    id: str
    source_url: str
    content: str
    title: Optional[str] = None
    author: Optional[str] = None
    language: str = "en"
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None
    page_count: Optional[int] = None
    word_count: Optional[int] = None

    def __post_init__(self) -> None:
        """Validate and set defaults after initialization."""
        if not self.source_url:
            raise ValueError("source_url is required")

        if not self.content:
            raise ValueError("content is required")

        if self.page_count is not None and self.page_count <= 0:
            raise ValueError("page_count must be a positive integer")

        if self.word_count is not None and self.word_count < 0:
            raise ValueError("word_count must be a non-negative integer")

        # Set defaults if not provided
        if self.created_at is None:
            self.created_at = datetime.now()

        if self.updated_at is None:
            self.updated_at = datetime.now()

    def __str__(self) -> str:
        """String representation for easy debugging/printing."""
        return (f"BookContent(id={self.id}, "
                f"title={self.title or 'Untitled'}, "
                f"source_url={self.source_url}, "
                f"words={self.word_count or 'N/A'})")

    def to_dict(self) -> dict:
        """Convert to dict for easy serialization (e.g., Qdrant payload)."""
        return {
            "id": self.id,
            "source_url": self.source_url,
            "content": self.content,
            "title": self.title,
            "author": self.author,
            "language": self.language,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None,
            "page_count": self.page_count,
            "word_count": self.word_count
        }