"""
Integration tests for the end-to-end query flow
Tests the complete flow from frontend request to backend processing and response
"""

import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch

from api import app
from backend.src.models.query_request import QueryRequest
from backend.src.models.rag_response import RAGAgentResponse


@pytest.fixture
def client():
    """Create a test client for the API."""
    with TestClient(app) as test_client:
        yield test_client


def test_end_to_end_query_flow(client):
    """Test the complete end-to-end query flow."""
    # Mock the RAG service to avoid actual API calls
    with patch('backend.src.services.rag_service.RAGService.process_query') as mock_process_query:
        # Create a mock response
        mock_response = RAGAgentResponse(
            response="This is a mock response to your query.",
            sources=[],
            session_id="test-session-id",
            query_id="test-query-id",
            timestamp="2023-01-01T00:00:00"
        )

        # Configure the mock to return the mock response
        mock_process_query.return_value = mock_response

        # Define the query request
        query_data = {
            "query": "How does robot localization work?",
            "session_id": "test-session-id"
        }

        # Send the query to the API
        response = client.post("/query", json=query_data)

        # Assert the response
        assert response.status_code == 200
        assert response.json()["response"] == "This is a mock response to your query."
        assert response.json()["session_id"] == "test-session-id"


def test_health_endpoint(client):
    """Test the health endpoint."""
    response = client.get("/health")

    # Assert the response
    assert response.status_code == 200
    assert response.json()["status"] == "healthy"
    assert "timestamp" in response.json()


def test_invalid_query_format(client):
    """Test that invalid query formats return appropriate errors."""
    # Send an invalid query (missing required 'query' field)
    invalid_query_data = {
        "session_id": "test-session-id"
    }

    response = client.post("/query", json=invalid_query_data)

    # Assert that we get a validation error
    assert response.status_code == 422  # Unprocessable Entity for validation errors


if __name__ == "__main__":
    pytest.main([__file__])