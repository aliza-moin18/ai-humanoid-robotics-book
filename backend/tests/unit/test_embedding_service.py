"""
Unit tests for EmbeddingService.
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
from src.services.embedding_service import EmbeddingService
from src.models.text_chunk import TextChunk
from src.models.embedding_vector import EmbeddingVector
from src.lib.config import Config
from src.lib.errors import EmbeddingGenerationError


class TestEmbeddingService(unittest.TestCase):
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock the Config to avoid needing real API keys
        Config.COHERE_API_KEY = "test-key"
    
    @patch('src.services.embedding_service.cohere.Client')
    def test_generate_embeddings_success(self, mock_cohere_client):
        """Test successful embedding generation."""
        # Arrange
        mock_client_instance = Mock()
        mock_cohere_client.return_value = mock_client_instance
        
        # Mock the embed response
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3], [0.4, 0.5, 0.6]]
        mock_response.meta = {"api_version": {"version": "1.0"}}
        mock_client_instance.embed.return_value = mock_response
        
        service = EmbeddingService()
        
        # Create test text chunks
        text_chunks = [
            TextChunk(
                id="chunk-1",
                book_content_id="book-1",
                content="This is the first text chunk.",
                chunk_index=0,
                source_url="https://example.com"
            ),
            TextChunk(
                id="chunk-2",
                book_content_id="book-1",
                content="This is the second text chunk.",
                chunk_index=1,
                source_url="https://example.com"
            )
        ]
        
        # Act
        result = service.generate_embeddings(text_chunks)
        
        # Assert
        self.assertEqual(len(result), 2)
        self.assertIsInstance(result[0], EmbeddingVector)
        self.assertEqual(result[0].text_chunk_id, "chunk-1")
        self.assertEqual(result[0].vector, [0.1, 0.2, 0.3])
        self.assertEqual(result[1].text_chunk_id, "chunk-2")
        self.assertEqual(result[1].vector, [0.4, 0.5, 0.6])
        
        # Verify the embed method was called with the correct parameters
        mock_client_instance.embed.assert_called_once_with(
            texts=["This is the first text chunk.", "This is the second text chunk."],
            model="embed-multilingual-v3.0",
            input_type="search_document"
        )
    
    @patch('src.services.embedding_service.cohere.Client')
    def test_generate_embeddings_empty_list(self, mock_cohere_client):
        """Test embedding generation with empty list."""
        # Arrange
        service = EmbeddingService()
        text_chunks = []
        
        # Act
        result = service.generate_embeddings(text_chunks)
        
        # Assert
        self.assertEqual(result, [])
        # The Cohere client should not be called for an empty list
        mock_cohere_client.return_value.embed.assert_not_called()
    
    @patch('src.services.embedding_service.cohere.Client')
    def test_generate_embeddings_error(self, mock_cohere_client):
        """Test embedding generation with API error."""
        # Arrange
        mock_client_instance = Mock()
        mock_cohere_client.return_value = mock_client_instance
        mock_client_instance.embed.side_effect = Exception("API Error")
        
        service = EmbeddingService()
        text_chunks = [
            TextChunk(
                id="chunk-1",
                book_content_id="book-1",
                content="This is a text chunk.",
                chunk_index=0,
                source_url="https://example.com"
            )
        ]
        
        # Act & Assert
        with self.assertRaises(EmbeddingGenerationError):
            service.generate_embeddings(text_chunks)
    
    def test_validate_embedding_valid(self):
        """Test validation of a valid embedding."""
        # Arrange
        embedding_vector = EmbeddingVector(
            id="emb-1",
            text_chunk_id="chunk-1",
            vector=[0.1, 0.2, 0.3],
            model_name="test-model",
            model_version="1.0",
            embedding_size=3
        )
        
        service = EmbeddingService()
        
        # Act
        result = service.validate_embedding(embedding_vector)
        
        # Assert
        self.assertTrue(result)
    
    def test_validate_embedding_invalid_size(self):
        """Test validation of an embedding with incorrect size."""
        # Arrange and Act & Assert
        with self.assertRaises(ValueError):
            EmbeddingVector(
                id="emb-1",
                text_chunk_id="chunk-1",
                vector=[0.1, 0.2, 0.3],
                model_name="test-model",
                model_version="1.0",
                embedding_size=5  # Incorrect size
            )
    
    def test_validate_embedding_non_numeric_values(self):
        """Test validation of an embedding with non-numeric values."""
        # Arrange
        embedding_vector = EmbeddingVector(
            id="emb-1",
            text_chunk_id="chunk-1",
            vector=[0.1, "invalid", 0.3],
            model_name="test-model",
            model_version="1.0",
            embedding_size=3
        )
        
        service = EmbeddingService()
        
        # Act
        result = service.validate_embedding(embedding_vector)
        
        # Assert
        self.assertFalse(result)


if __name__ == '__main__':
    unittest.main()