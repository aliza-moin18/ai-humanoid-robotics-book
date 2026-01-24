"""
Integration tests for end-to-end functionality
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.agent import RetrievalAgent
from backend.agent_types import AgentResponse, Citation


class TestEndToEndFunctionality:
    """Test suite for end-to-end functionality"""

    def test_complete_query_workflow(self):
        """Test the complete workflow from query to response with citations"""
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
                mock_text_content.text.value = "Artificial intelligence is demonstrated by machines as opposed to natural intelligence in humans. It involves intelligent agents that perceive their environment and take actions to achieve goals."
                
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
                
                # Mock all external dependencies
                mock_retrieved_chunks = [
                    {
                        "id": "chunk_1",
                        "content": "Artificial intelligence (AI) is intelligence demonstrated by machines, in contrast to the natural intelligence displayed by humans and animals.",
                        "source_url": "https://example.com/ai-definition",
                        "score": 0.95,
                        "metadata": {}
                    },
                    {
                        "id": "chunk_2",
                        "content": "Leading AI textbooks define the field as the study of 'intelligent agents': any device that perceives its environment and takes actions that maximize its chance of successfully achieving its goals.",
                        "source_url": "https://example.com/ai-textbook",
                        "score": 0.89,
                        "metadata": {}
                    }
                ]
                
                with patch.object(agent, '_retrieve_content', return_value=mock_retrieved_chunks):
                    # Execute the complete workflow
                    result = agent.process_query("What is artificial intelligence?")
                    
                    # Verify the complete result
                    assert isinstance(result, AgentResponse)
                    assert "machines" in result.answer.lower()
                    assert "intelligent agents" in result.answer.lower()

    def test_complete_workflow_with_follow_up(self):
        """Test the complete workflow with follow-up query context"""
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
                mock_client.beta.threads.create.return_value = mock_thread
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
                mock_text_content.text.value = "Machine learning is a subset of AI that enables systems to learn from experience without explicit programming."
                
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
                
                # Mock all external dependencies
                mock_retrieved_chunks = [
                    {
                        "id": "chunk_1",
                        "content": "Machine learning is a subset of AI that enables systems to learn and improve from experience without being explicitly programmed.",
                        "source_url": "https://example.com/ml-definition",
                        "score": 0.92,
                        "metadata": {}
                    }
                ]
                
                with patch.object(agent, '_retrieve_content', return_value=mock_retrieved_chunks):
                    # Execute the complete workflow with thread ID for follow-up
                    result = agent.process_query("How does it learn?", thread_id="existing-thread-id")
                    
                    # Verify the complete result
                    assert isinstance(result, AgentResponse)
                    assert "machine learning" in result.answer.lower()
                    assert "learn" in result.answer.lower()

    def test_error_handling_throughout_workflow(self):
        """Test that errors are properly handled throughout the workflow"""
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
                
                from backend.exceptions import RetrievalError
                
                # Mock retrieval to raise an error
                with patch.object(agent, '_retrieve_content', side_effect=RetrievalError("API unavailable")):
                    with pytest.raises(RetrievalError) as exc_info:
                        agent.process_query("Test query")
                    
                    assert "API unavailable" in str(exc_info.value)