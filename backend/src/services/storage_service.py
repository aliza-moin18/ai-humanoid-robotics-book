"""
Service for storing embeddings in a Qdrant vector database.
"""
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
import uuid

from ..models.embedding_vector import EmbeddingVector
from ..models.db_record import VectorDatabaseRecord
from src.config import config
from ..lib.logger import app_logger
from ..lib.errors import VectorStorageError


class StorageService:
    """
    Service for storing embeddings in a Qdrant vector database.
    """
    
    def __init__(self, collection_name: Optional[str] = None):
        """
        Initialize the storage service.
        
        Args:
            collection_name: Name of the Qdrant collection to use
        """
        self.collection_name = collection_name or config.QDRANT_COLLECTION_NAME
        
        # Initialize Qdrant client
        self.client = QdrantClient(
            url=config.QDRANT_URL,
            api_key=config.QDRANT_API_KEY,
            prefer_grpc=False  # Use gRPC for better performance if available
        )
        
        # Create collection if it doesn't exist
        self._ensure_collection_exists()
    
    def _ensure_collection_exists(self):
        """
        Ensure the collection exists in Qdrant with the appropriate configuration.
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]
            
            if self.collection_name not in collection_names:
                # Create collection with appropriate vector size
                vector_size = 1024  # This should match the embedding size of your model
                
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(
                        size=vector_size,
                        distance=models.Distance.COSINE
                    )
                )
                
                app_logger.info(f"Created new Qdrant collection: {self.collection_name}")
            else:
                app_logger.info(f"Using existing Qdrant collection: {self.collection_name}")
        
        except Exception as e:
            app_logger.error(f"Error ensuring collection exists: {str(e)}")
            raise VectorStorageError(f"Failed to ensure collection exists: {str(e)}")
    
    def store_embeddings(self, embedding_vectors: List[EmbeddingVector], payload: Optional[List[dict]] = None) -> List[str]:
        """
        Store embedding vectors in the Qdrant database with optional payload.
        
        Args:
            embedding_vectors: List of EmbeddingVector objects to store
            payload: Optional list of payloads (same length as embeddings)
            
        Returns:
            List of IDs of the stored vectors
        """
        if not embedding_vectors:
            return []
        
        try:
            # Prepare points for insertion
            points = []
            stored_ids = []
            
            for idx, embedding_vector in enumerate(embedding_vectors):
                # Create a unique ID for the record in Qdrant
                record_id = str(uuid.uuid4())
                
                # Use provided payload or default metadata
                point_payload = payload[idx] if payload and idx < len(payload) else {
                    "text_chunk_id": embedding_vector.text_chunk_id,
                    "embedding_id": embedding_vector.id,
                    "model_name": embedding_vector.model_name,
                    "model_version": embedding_vector.model_version,
                    "created_at": embedding_vector.created_at.isoformat() if embedding_vector.created_at else None
                }
                
                # Create point structure
                point = PointStruct(
                    id=record_id,
                    vector=embedding_vector.vector,
                    payload=point_payload
                )
                
                points.append(point)
                stored_ids.append(record_id)
            
            # Upload points to Qdrant
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )
            
            app_logger.info(f"Successfully stored {len(stored_ids)} embeddings in collection {self.collection_name}")
            return stored_ids
            
        except Exception as e:
            app_logger.error(f"Error storing embeddings: {str(e)}")
            raise VectorStorageError(f"Failed to store embeddings: {str(e)}")
    
    def search(self, query_vector: List[float], top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Perform a vector similarity search in the Qdrant database.
        
        Args:
            query_vector: The vector to search for similar vectors
            top_k: Number of top results to return
            
        Returns:
            List of search results with payload and score
        """
        try:
            # Perform search in Qdrant
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=top_k
            )
            
            # Format results
            results = []
            for hit in search_results:
                result = {
                    "id": hit.id,
                    "score": hit.score,
                    "payload": hit.payload,
                    "vector": hit.vector if hit.vector else None
                }
                results.append(result)
            
            app_logger.info(f"Search returned {len(results)} results")
            return results
            
        except Exception as e:
            app_logger.error(f"Error performing search: {str(e)}")
            raise VectorStorageError(f"Failed to perform search: {str(e)}")
    
    def get_record_by_id(self, record_id: str) -> Optional[VectorDatabaseRecord]:
        """
        Retrieve a specific record by its ID.
        
        Args:
            record_id: ID of the record to retrieve
            
        Returns:
            VectorDatabaseRecord if found, None otherwise
        """
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[record_id]
            )
            
            if records:
                record = records[0]
                return VectorDatabaseRecord(
                    id=record.id,
                    embedding_id=record.payload.get("embedding_id"),
                    vector=record.vector if record.vector else [],
                    payload=record.payload
                )
            
            return None
        except Exception as e:
            app_logger.error(f"Error retrieving record by ID: {str(e)}")
            raise VectorStorageError(f"Failed to retrieve record: {str(e)}")
    
    def delete_collection(self):
        """
        Delete the entire collection (use with caution!).
        """
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            app_logger.info(f"Deleted collection: {self.collection_name}")
        except Exception as e:
            app_logger.error(f"Error deleting collection: {str(e)}")
            raise VectorStorageError(f"Failed to delete collection: {str(e)}")