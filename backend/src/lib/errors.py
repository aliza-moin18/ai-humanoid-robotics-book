"""
Custom exception classes for the book embeddings ingestion system.
"""


class BookIngestionError(Exception):
    """Base exception for the book ingestion system."""
    pass


class ContentExtractionError(BookIngestionError):
    """Raised when content extraction from a URL fails."""
    pass


class TextProcessingError(BookIngestionError):
    """Raised when text processing operations fail."""
    pass


class EmbeddingGenerationError(BookIngestionError):
    """Raised when embedding generation fails."""
    pass


class VectorStorageError(BookIngestionError):
    """Raised when vector storage operations fail."""
    pass


class ConfigurationError(BookIngestionError):
    """Raised when configuration validation fails."""
    pass


class APIError(BookIngestionError):
    """Raised when API calls fail."""
    def __init__(self, message: str, status_code: int = None, response_data: dict = None):
        super().__init__(message)
        self.status_code = status_code
        self.response_data = response_data