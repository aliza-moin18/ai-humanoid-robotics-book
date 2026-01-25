"""
Service for generating text embeddings using Cohere API.
"""
import cohere
from typing import List, Dict, Any
from ..models.text_chunk import TextChunk
from ..models.embedding_vector import EmbeddingVector
from src.config import config
from ..lib.logger import app_logger
from ..lib.errors import EmbeddingGenerationError
import time


class EmbeddingService:
    """
    Service for generating text embeddings using Cohere API.
    """
    
    def __init__(self, model_name: str = "embed-multilingual-v3.0", input_type: str = "search_document"):
        """
        Initialize the embedding service.
        
        Args:
            model_name: Name of the Cohere model to use
            input_type: Type of input for the embedding model
        """
        self.model_name = model_name
        self.input_type = input_type
        self.client = cohere.Client(config.COHERE_API_KEY)
    
    def generate_embeddings(self, text_chunks: List[TextChunk]) -> List[EmbeddingVector]:
        """
        Generate embeddings for a list of text chunks.
        
        Args:
            text_chunks: List of TextChunk objects to generate embeddings for
            
        Returns:
            List of EmbeddingVector objects
        """
        if not text_chunks:
            return []
        
        # Extract text content from chunks
        texts = [chunk.content for chunk in text_chunks]
        
        try:
            app_logger.info(f"Generating embeddings for {len(texts)} text chunks using model {self.model_name}")
            
            # Call Cohere API to generate embeddings
            response = self.client.embed(
                texts=texts,
                model=self.model_name,
                input_type=self.input_type
            )
            
            # Create EmbeddingVector objects from the response
            embedding_vectors = []
            for i, embedding in enumerate(response.embeddings):
                # Get model version from response - the structure may vary depending on the Cohere version
                model_version = getattr(response, 'model_version', 'unknown')
                if hasattr(response, 'meta') and hasattr(response.meta, 'api_version'):
                    model_version = response.meta.api_version.version
                elif hasattr(response, 'model_version'):
                    model_version = response.model_version
                else:
                    model_version = "unknown"

                embedding_vector = EmbeddingVector(
                    id=text_chunks[i].id,  # Using the same ID as the text chunk for reference
                    text_chunk_id=text_chunks[i].id,
                    vector=embedding,
                    model_name=self.model_name,
                    model_version=model_version,
                    embedding_size=len(embedding)
                )
                embedding_vectors.append(embedding_vector)
            
            app_logger.info(f"Successfully generated {len(embedding_vectors)} embeddings")
            return embedding_vectors
            
        except Exception as e:
            app_logger.error(f"Error generating embeddings: {str(e)}")
            raise EmbeddingGenerationError(f"Failed to generate embeddings: {str(e)}")
    
    def validate_embedding(self, embedding_vector: EmbeddingVector) -> bool:
        """
        Validate that an embedding has the correct dimensions and format.
        
        Args:
            embedding_vector: EmbeddingVector to validate
            
        Returns:
            True if valid, False otherwise
        """
        try:
            # Check that vector is a list of floats
            if not isinstance(embedding_vector.vector, list):
                return False
            
            # Check that all elements are floats/numbers
            if not all(isinstance(val, (int, float)) for val in embedding_vector.vector):
                return False
            
            # Check that the embedding size matches the vector length
            if embedding_vector.embedding_size != len(embedding_vector.vector):
                return False
            
            return True
        except Exception:
            return False
    
    def get_model_info(self) -> Dict[str, Any]:
        """
        Get information about the current embedding model.
        
        Returns:
            Dictionary with model information
        """
        # Note: Cohere API doesn't have a direct method to get model info
        # This is a placeholder implementation
        return {
            "model_name": self.model_name,
            "input_type": self.input_type,
            "expected_dimensions": "Depends on model"
        }