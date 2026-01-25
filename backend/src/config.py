from pydantic_settings import BaseSettings
from dotenv import load_dotenv
from typing import Optional
import os

load_dotenv()

class Config(BaseSettings):
    """Configuration class to manage application settings."""

    LOG_LEVEL: str = "INFO"

    COHERE_API_KEY: str

    # Qdrant settings
    QDRANT_API_KEY: str
    QDRANT_HOST: str
    QDRANT_URL: str
    QDRANT_COLLECTION_NAME: str = "book_content"

    # Yeh nayi lines add karo
    CHUNK_SIZE: int = 1000
    CHUNK_OVERLAP: int = 200

    # Other
    DEPLOY_VERCEL_URL: Optional[str] = None

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = False

config = Config()