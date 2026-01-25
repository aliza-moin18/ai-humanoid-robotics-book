"""
Pydantic model for ErrorResponse
Standardized error response format for API errors.
"""

from pydantic import BaseModel
from typing import Optional, Dict, Any
from datetime import datetime


class ErrorResponse(BaseModel):
    """Standardized error response format for API errors."""
    error_code: str
    message: str
    details: Optional[Dict[str, Any]] = {}
    timestamp: str = datetime.now().isoformat()