"""
Custom exceptions for the RAG agent
"""


class InvalidQueryError(Exception):
    """
    Raised when the query exceeds length limits or is malformed
    """
    def __init__(self, message: str = "Invalid query provided"):
        self.message = message
        super().__init__(self.message)


class RetrievalError(Exception):
    """
    Raised when the retrieval API is unavailable or returns no results
    """
    def __init__(self, message: str = "Error retrieving content from the API"):
        self.message = message
        super().__init__(self.message)


class GenerationError(Exception):
    """
    Raised when the LLM fails to generate a response
    """
    def __init__(self, message: str = "Error generating response from the LLM"):
        self.message = message
        super().__init__(self.message)