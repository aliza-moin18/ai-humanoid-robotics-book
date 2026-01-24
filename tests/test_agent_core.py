"""
Unit tests for core query functionality
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.agent import RetrievalAgent
from backend.agent_types import AgentResponse, Citation
from backend.exceptions import InvalidQueryError


class TestRetrievalAgentCore:
    """Test suite for core query functionality of the RetrievalAgent"""

    def test_initialization(self):
        """Test that the agent initializes with proper configuration"""
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
                assert agent.config is not None
                assert agent.config['openai_api_key'] == 'test-key'
                assert agent.client is not None
                # Verify that assistant was created
                mock_client.beta.assistants.create.assert_called()

    def test_process_query_valid_input(self):
        """Test that process_query handles valid input correctly"""
        with patch.dict('os.environ', {'OPENAI_API_KEY': 'test-key'}):
            with patch('backend.agent.OpenAI') as mock_openai_class:
                # Create mock objects
                mock_client = Mock()
                mock_assistant = Mock()
                mock_assistant.id = 'test-assistant-id'
                
                # Set up the mock client
                mock_client.beta.assistants.create.return_value = mock_assistant
                mock_client.beta.assistants.retrieve.return_value = mock_assistant
                
                # Mock thread operations
                mock_thread = Mock()
                mock_thread.id = 'test-thread-id'
                mock_client.beta.threads.create.return_value = mock_thread
                
                # Mock message operations
                mock_message = Mock()
                mock_client.beta.threads.messages.create.return_value = mock_message
                
                # Mock run operations
                mock_run = Mock()
                mock_run.id = 'test-run-id'
                mock_run.status = 'completed'
                mock_client.beta.threads.runs.create.return_value = mock_run
                mock_client.beta.threads.runs.retrieve.return_value = mock_run
                
                # Mock message listing
                mock_text_content = Mock()
                mock_text_content.type = 'text'
                mock_text_content.text.value = 'This is a test answer.'
                
                mock_assistant_message = Mock()
                mock_assistant_message.role = 'assistant'
                mock_assistant_message.content = [mock_text_content]
                
                mock_messages_list = Mock()
                mock_messages_list.data = [mock_assistant_message]
                mock_client.beta.threads.messages.list.return_value = mock_messages_list
                
                # Mock run steps
                mock_run_steps = Mock()
                mock_run_steps.data = []
                mock_client.beta.threads.runs.steps.list.return_value = mock_run_steps
                
                # Return the mock client when OpenAI is instantiated
                mock_openai_class.return_value = mock_client
                
                agent = RetrievalAgent()
                
                # Mock the internal retrieval method
                with patch.object(agent, '_retrieve_content', return_value=[
                    {
                        "id": "test_chunk_1",
                        "content": "Test content for robotics.",
                        "source_url": "https://example.com/test",
                        "score": 0.9,
                        "metadata": {}
                    }
                ]):
                    
                    result = agent.process_query("What is robotics?")
                    
                    assert isinstance(result, AgentResponse)
                    assert result.answer == "This is a test answer."

    def test_process_query_invalid_long_query(self):
        """Test that process_query raises exception for queries exceeding length limit"""
        with patch.dict('os.environ', {'OPENAI_API_KEY': 'test-key', 'QUERY_MAX_LENGTH': '100'}):
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
                
                long_query = "This is a very long query that exceeds the maximum allowed length limit " * 10
                
                with pytest.raises(InvalidQueryError):
                    agent.process_query(long_query)

    def test_process_query_empty_query(self):
        """Test that process_query raises exception for empty query"""
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
                
                with pytest.raises(InvalidQueryError):
                    agent.process_query("")

    def test_process_query_none_query(self):
        """Test that process_query raises exception for None query"""
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
                
                with pytest.raises(InvalidQueryError):
                    agent.process_query(None)