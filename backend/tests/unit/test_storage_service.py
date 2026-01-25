"""
Unit tests for StorageService.
"""
import unittest
from unittest.mock import Mock, patch, MagicMock
from src.services.storage_service import StorageService
from src.models.embedding_vector import EmbeddingVector
from src.lib.config import Config
from src.lib.errors import VectorStorageError


class TestStorageService(unittest.TestCase):
    
    def setUp(self):
        """Set up test fixtures before each test method."""
        # Mock the Config to avoid needing real API keys
        Config.QDRANT_HOST = "https://test-qdrant.example.com"
        Config.QDRANT_API_KEY = "test-key"
        Config.QDRANT_COLLECTION_NAME = "test-collection"
    
    @patch('src.services.storage_service.QdrantClient')
    def test_store_embeddings_success(self, mock_qdrant_client):
        """Test successful storage of embeddings."""
        # Arrange
        mock_client_instance = Mock()
        mock_qdrant_client.return_value = mock_client_instance
        mock_client_instance.get_collections.return_value = Mock(collections=[])
        
        service = StorageService()
        
        # Create test embedding vectors
        embedding_vectors = [
            EmbeddingVector(
                id="emb-1",
                text_chunk_id="chunk-1",
                vector=[0.1, 0.2, 0.3],
                model_name="test-model",
                model_version="1.0",
                embedding_size=3
            ),
            EmbeddingVector(
                id="emb-2",
                text_chunk_id="chunk-2",
                vector=[0.4, 0.5, 0.6],
                model_name="test-model",
                model_version="1.0",
                embedding_size=3
            )
        ]
        
        # Act
        result = service.store_embeddings(embedding_vectors)
        
        # Assert
        self.assertEqual(len(result), 2)
        # Verify upsert was called
        mock_client_instance.upsert.assert_called_once()
        args, kwargs = mock_client_instance.upsert.call_args
        self.assertEqual(kwargs['collection_name'], "test-collection")
        self.assertEqual(len(kwargs['points']), 2)
    
    @patch('src.services.storage_service.QdrantClient')
    def test_store_embeddings_empty_list(self, mock_qdrant_client):
        """Test storage of empty embeddings list."""
        # Arrange
        mock_client_instance = Mock()
        mock_qdrant_client.return_value = mock_client_instance
        mock_client_instance.get_collections.return_value = Mock(collections=[])
        
        service = StorageService()
        embedding_vectors = []
        
        # Act
        result = service.store_embeddings(embedding_vectors)
        
        # Assert
        self.assertEqual(result, [])
        # upsert should not be called for an empty list
        mock_client_instance.upsert.assert_not_called()
    
    @patch('src.services.storage_service.QdrantClient')
    def test_store_embeddings_error(self, mock_qdrant_client):
        """Test storage of embeddings with error."""
        # Arrange
        mock_client_instance = Mock()
        mock_qdrant_client.return_value = mock_client_instance
        mock_client_instance.get_collections.return_value = Mock(collections=[])
        mock_client_instance.upsert.side_effect = Exception("Storage Error")
        
        service = StorageService()
        embedding_vectors = [
            EmbeddingVector(
                id="emb-1",
                text_chunk_id="chunk-1",
                vector=[0.1, 0.2, 0.3],
                model_name="test-model",
                model_version="1.0",
                embedding_size=3
            )
        ]
        
        # Act & Assert
        with self.assertRaises(VectorStorageError):
            service.store_embeddings(embedding_vectors)
    
    @patch('src.services.storage_service.QdrantClient')
    def test_search_success(self, mock_qdrant_client):
        """Test successful search."""
        # Arrange
        mock_client_instance = Mock()
        mock_qdrant_client.return_value = mock_client_instance
        mock_client_instance.get_collections.return_value = Mock(collections=[])
        
        # Mock search results
        mock_hit1 = Mock()
        mock_hit1.id = "result-1"
        mock_hit1.score = 0.9
        mock_hit1.payload = {"text_chunk_id": "chunk-1"}
        mock_hit1.vector = [0.1, 0.2, 0.3]
        
        mock_hit2 = Mock()
        mock_hit2.id = "result-2"
        mock_hit2.score = 0.8
        mock_hit2.payload = {"text_chunk_id": "chunk-2"}
        mock_hit2.vector = [0.4, 0.5, 0.6]
        
        mock_client_instance.search.return_value = [mock_hit1, mock_hit2]
        
        service = StorageService()
        query_vector = [0.1, 0.2, 0.3]
        
        # Act
        results = service.search(query_vector, top_k=2)
        
        # Assert
        self.assertEqual(len(results), 2)
        self.assertEqual(results[0]["id"], "result-1")
        self.assertEqual(results[0]["score"], 0.9)
        self.assertEqual(results[0]["payload"], {"text_chunk_id": "chunk-1"})
        self.assertEqual(results[1]["id"], "result-2")
        self.assertEqual(results[1]["score"], 0.8)
        
        # Verify search was called with correct parameters
        mock_client_instance.search.assert_called_once_with(
            collection_name="test-collection",
            query_vector=query_vector,
            limit=2
        )
    
    @patch('src.services.storage_service.QdrantClient')
    def test_search_error(self, mock_qdrant_client):
        """Test search with error."""
        # Arrange
        mock_client_instance = Mock()
        mock_qdrant_client.return_value = mock_client_instance
        mock_client_instance.get_collections.return_value = Mock(collections=[])
        mock_client_instance.search.side_effect = Exception("Search Error")
        
        service = StorageService()
        query_vector = [0.1, 0.2, 0.3]
        
        # Act & Assert
        with self.assertRaises(VectorStorageError):
            service.search(query_vector, top_k=2)


if __name__ == '__main__':
    unittest.main()