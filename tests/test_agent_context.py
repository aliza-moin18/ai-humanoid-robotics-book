"""
Unit tests for follow-up query functionality
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.agent import RetrievalAgent
from backend.agent_types import AgentResponse, Citation


class TestFollowUpQueries:
    """Test suite for follow-up query functionality"""

    def test_process_query_with_existing_thread(self):
        """Test that process_query handles existing thread correctly for follow-up queries"""
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
                mock_thread.id = 'existing-thread-id'
                mock_client.beta.threads.retrieve.return_value = mock_thread
                
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
                mock_text_content.text.value = 'Robots use machine learning to improve their performance.'
                
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
                
                # Mock the retrieval method
                with patch.object(agent, '_retrieve_content', return_value=[
                    {
                        "id": "test_chunk_1",
                        "content": "Robots often incorporate machine learning algorithms to improve their performance.",
                        "source_url": "https://example.com/robotics-ml",
                        "score": 0.88,
                        "metadata": {}
                    }
                ]):
                    
                    result = agent.process_query("How do they learn?", thread_id="existing-thread-id")
                    
                    assert isinstance(result, AgentResponse)
                    assert result.answer == "Robots use machine learning to improve their performance."
                    # Verify that the existing thread was retrieved
                    mock_client.beta.threads.retrieve.assert_called_once_with("existing-thread-id")

    def test_process_query_creates_new_thread_when_none_provided(self):
        """Test that process_query creates a new thread when none is provided"""
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
                mock_thread.id = 'new-thread-id'
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
                mock_text_content.text.value = 'Robots are programmable machines that execute tasks automatically.'
                
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
                
                # Mock the retrieval method
                with patch.object(agent, '_retrieve_content', return_value=[
                    {
                        "id": "test_chunk_1",
                        "content": "Robots are programmable machines that execute tasks automatically.",
                        "source_url": "https://example.com/robotics-basics",
                        "score": 0.92,
                        "metadata": {}
                    }
                ]):
                    
                    result = agent.process_query("What is a robot?")
                    
                    assert isinstance(result, AgentResponse)
                    assert result.answer == "Robots are programmable machines that execute tasks automatically."
                    # Verify that a new thread was created
                    mock_client.beta.threads.create.assert_called_once()