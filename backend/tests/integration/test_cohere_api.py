"""
Integration tests for Cohere API interactions.
"""
import unittest
from unittest.mock import Mock, patch
import os
from src.services.embedding_service import EmbeddingService
from src.models.text_chunk import TextChunk
from src.lib.config import Config
from src.lib.errors import EmbeddingGenerationError


class TestCohereAPIIntegration(unittest.TestCase):
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        # Store original config values
        self.original_api_key = Config.COHERE_API_KEY
        
        # Use a test API key for these tests
        Config.COHERE_API_KEY = os.getenv("COHERE_API_KEY", "test-key")
    
    def tearDown(self):
        """Clean up after each test method."""
        # Restore original config values
        Config.COHERE_API_KEY = self.original_api_key
    
    @unittest.skipIf(not os.getenv("COHERE_API_KEY"), "Cohere API key not set, skipping integration test")
    def test_cohere_api_integration(self):
        """Test actual integration with Cohere API if key is available."""
        # This test will only run if a real Cohere API key is available
        service = EmbeddingService()
        
        # Create test text chunks
        text_chunks = [
            TextChunk(
                id="chunk-1",
                book_content_id="book-1",
                content="This is a test sentence for embedding.",
                chunk_index=0,
                source_url="https://example.com"
            )
        ]
        
        # Generate embeddings
        embedding_vectors = service.generate_embeddings(text_chunks)
        
        # Verify results
        self.assertEqual(len(embedding_vectors), 1)
        self.assertIsNotNone(embedding_vectors[0].vector)
        self.assertGreater(len(embedding_vectors[0].vector), 0)
        self.assertEqual(embedding_vectors[0].text_chunk_id, "chunk-1")
    
    @patch('src.services.embedding_service.cohere.Client')
    def test_cohere_api_mocked(self, mock_cohere_client):
        """Test Cohere API interaction with mocked client."""
        # Arrange
        mock_client_instance = Mock()
        mock_cohere_client.return_value = mock_client_instance
        
        # Mock the embed response
        mock_response = Mock()
        mock_response.embeddings = [[0.1, 0.2, 0.3, 0.4]]
        mock_response.meta = {"api_version": {"version": "1.0"}}
        mock_client_instance.embed.return_value = mock_response
        
        service = EmbeddingService()
        
        # Create test text chunks
        text_chunks = [
            TextChunk(
                id="chunk-1",
                book_content_id="book-1",
                content="This is a test sentence for embedding.",
                chunk_index=0,
                source_url="https://example.com"
            )
        ]
        
        # Act
        embedding_vectors = service.generate_embeddings(text_chunks)
        
        # Assert
        self.assertEqual(len(embedding_vectors), 1)
        self.assertEqual(embedding_vectors[0].vector, [0.1, 0.2, 0.3, 0.4])
        self.assertEqual(embedding_vectors[0].text_chunk_id, "chunk-1")
        
        # Verify the embed method was called with correct parameters
        mock_client_instance.embed.assert_called_once()
        call_args = mock_client_instance.embed.call_args
        self.assertEqual(call_args[1]['model'], "embed-multilingual-v3.0")
        self.assertEqual(call_args[1]['input_type'], "search_document")
        self.assertEqual(call_args[1]['texts'], ["This is a test sentence for embedding."])
    
    @patch('src.services.embedding_service.cohere.Client')
    def test_cohere_api_error_handling(self, mock_cohere_client):
        """Test error handling when Cohere API fails."""
        # Arrange
        mock_client_instance = Mock()
        mock_cohere_client.return_value = mock_client_instance
        mock_client_instance.embed.side_effect = Exception("API Error")
        
        service = EmbeddingService()
        
        # Create test text chunks
        text_chunks = [
            TextChunk(
                id="chunk-1",
                book_content_id="book-1",
                content="This is a test sentence for embedding.",
                chunk_index=0,
                source_url="https://example.com"
            )
        ]
        
        # Act & Assert
        with self.assertRaises(EmbeddingGenerationError):
            service.generate_embeddings(text_chunks)


if __name__ == '__main__':
    unittest.main()