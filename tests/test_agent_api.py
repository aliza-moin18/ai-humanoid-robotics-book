"""
Unit tests for API integration
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
from requests.exceptions import Timeout, RequestException
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.agent import RetrievalAgent
from backend.exceptions import RetrievalError


class TestAPIIntegration:
    """Test suite for API integration functionality"""

    def test_successful_api_call(self):
        """Test that the agent successfully calls the retrieval API"""
        with patch.dict('os.environ', {
            'OPENAI_API_KEY': 'test-key',
            'RETRIEVAL_API_URL': 'http://test-api.local/query'
        }):
            with patch('backend.agent.OpenAI') as mock_openai_class:
                # Create mock objects
                mock_client = Mock()
                mock_assistant = Mock()
                mock_assistant.id = 'test-assistant-id'
                
                # Set up the mock client
                mock_client.beta.assistants.create.return_value = mock_assistant
                mock_client.beta.assistants.retrieve.return_value = mock_assistant
                
                # Return the mock client when OpenAI is instantiated
                mock_openai_class.return_value = mock_client
                
                agent = RetrievalAgent()
                
                # Mock the requests.post method
                mock_response = Mock()
                mock_response.status_code = 200
                mock_response.json.return_value = {
                    'results': [
                        {
                            "id": "chunk_1",
                            "content": "Test content from API.",
                            "source_url": "https://example.com/test",
                            "score": 0.9,
                            "metadata": {}
                        }
                    ]
                }
                
                with patch('requests.post', return_value=mock_response) as mock_post:
                    result = agent._retrieve_content("Test query")
                    
                    # Verify the API was called with correct parameters
                    mock_post.assert_called_once_with(
                        'http://test-api.local/query',
                        json={'query': 'Test query', 'top_k': 5},
                        headers={"Content-Type": "application/json"},
                        timeout=30
                    )
                    
                    # Verify the result
                    assert len(result) == 1
                    assert result[0]['content'] == "Test content from API."

    def test_api_timeout_error(self):
        """Test that the agent handles API timeout correctly"""
        with patch.dict('os.environ', {
            'OPENAI_API_KEY': 'test-key',
            'TIMEOUT_SECONDS': '5'
        }):
            with patch('backend.agent.OpenAI') as mock_openai_class:
                # Create mock objects
                mock_client = Mock()
                mock_assistant = Mock()
                mock_assistant.id = 'test-assistant-id'
                
                # Set up the mock client
                mock_client.beta.assistants.create.return_value = mock_assistant
                mock_client.beta.assistants.retrieve.return_value = mock_assistant
                
                # Return the mock client when OpenAI is instantiated
                mock_openai_class.return_value = mock_client
                
                agent = RetrievalAgent()
                
                # Mock the requests.post method to raise a timeout
                with patch('requests.post', side_effect=Timeout()):
                    with pytest.raises(RetrievalError) as exc_info:
                        agent._retrieve_content("Test query")
                    
                    assert "timed out after 5 seconds" in str(exc_info.value)

    def test_api_request_exception(self):
        """Test that the agent handles API request exceptions correctly"""
        with patch.dict('os.environ', {'OPENAI_API_KEY': 'test-key'}):
            with patch('backend.agent.OpenAI') as mock_openai_class:
                # Create mock objects
                mock_client = Mock()
                mock_assistant = Mock()
                mock_assistant.id = 'test-assistant-id'
                
                # Set up the mock client
                mock_client.beta.assistants.create.return_value = mock_assistant
                mock_client.beta.assistants.retrieve.return_value = mock_assistant
                
                # Return the mock client when OpenAI is instantiated
                mock_openai_class.return_value = mock_client
                
                agent = RetrievalAgent()
                
                # Mock the requests.post method to raise a RequestException
                with patch('requests.post', side_effect=RequestException("Connection error")):
                    with pytest.raises(RetrievalError) as exc_info:
                        agent._retrieve_content("Test query")
                    
                    assert "Error calling retrieval API" in str(exc_info.value)

    def test_api_non_200_status_code(self):
        """Test that the agent handles non-200 status codes correctly"""
        with patch.dict('os.environ', {'OPENAI_API_KEY': 'test-key'}):
            with patch('backend.agent.OpenAI') as mock_openai_class:
                # Create mock objects
                mock_client = Mock()
                mock_assistant = Mock()
                mock_assistant.id = 'test-assistant-id'
                
                # Set up the mock client
                mock_client.beta.assistants.create.return_value = mock_assistant
                mock_client.beta.assistants.retrieve.return_value = mock_assistant
                
                # Return the mock client when OpenAI is instantiated
                mock_openai_class.return_value = mock_client
                
                agent = RetrievalAgent()
                
                # Mock the requests.post method to return a 500 status code
                mock_response = Mock()
                mock_response.status_code = 500
                
                with patch('requests.post', return_value=mock_response):
                    with pytest.raises(RetrievalError) as exc_info:
                        agent._retrieve_content("Test query")
                    
                    assert "returned status code 500" in str(exc_info.value)

    def test_api_missing_results_field(self):
        """Test that the agent handles API responses without 'results' field correctly"""
        with patch.dict('os.environ', {'OPENAI_API_KEY': 'test-key'}):
            with patch('backend.agent.OpenAI') as mock_openai_class:
                # Create mock objects
                mock_client = Mock()
                mock_assistant = Mock()
                mock_assistant.id = 'test-assistant-id'
                
                # Set up the mock client
                mock_client.beta.assistants.create.return_value = mock_assistant
                mock_client.beta.assistants.retrieve.return_value = mock_assistant
                
                # Return the mock client when OpenAI is instantiated
                mock_openai_class.return_value = mock_client
                
                agent = RetrievalAgent()
                
                # Mock the requests.post method to return a response without 'results' field
                mock_response = Mock()
                mock_response.status_code = 200
                mock_response.json.return_value = {
                    'other_field': 'some_value'
                }
                
                with patch('requests.post', return_value=mock_response):
                    with pytest.raises(RetrievalError) as exc_info:
                        agent._retrieve_content("Test query")
                    
                    assert "missing 'results' field" in str(exc_info.value)

    def test_api_json_decode_error(self):
        """Test that the agent handles JSON decode errors correctly"""
        with patch.dict('os.environ', {'OPENAI_API_KEY': 'test-key'}):
            with patch('backend.agent.OpenAI') as mock_openai_class:
                # Create mock objects
                mock_client = Mock()
                mock_assistant = Mock()
                mock_assistant.id = 'test-assistant-id'
                
                # Set up the mock client
                mock_client.beta.assistants.create.return_value = mock_assistant
                mock_client.beta.assistants.retrieve.return_value = mock_assistant
                
                # Return the mock client when OpenAI is instantiated
                mock_openai_class.return_value = mock_client
                
                agent = RetrievalAgent()
                
                # Mock the requests.post method to return invalid JSON
                mock_response = Mock()
                mock_response.status_code = 200
                mock_response.json.side_effect = ValueError("Invalid JSON")
                
                with patch('requests.post', return_value=mock_response):
                    with pytest.raises(RetrievalError) as exc_info:
                        agent._retrieve_content("Test query")
                    
                    assert "Error parsing retrieval API response" in str(exc_info.value)