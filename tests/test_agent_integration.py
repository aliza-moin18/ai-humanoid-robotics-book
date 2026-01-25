"""
Integration tests for query processing
"""
import pytest
from unittest.mock import Mock, patch, MagicMock
import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from backend.agent import RetrievalAgent
from backend.agent_types import AgentResponse, Citation


class TestRetrievalAgentIntegration:
    """Test suite for integration of query processing components"""

    def test_end_to_end_query_processing_with_mocked_external_services(self):
        """Test end-to-end query processing with mocked external services"""
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
                mock_text_content.text.value = 'Artificial intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence. Machine learning is a subset of AI that focuses on building systems that learn from data.'
                
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
                
                # Mock the retrieval method to return test data
                mock_retrieved_chunks = [
                    {
                        "id": "chunk_1",
                        "content": "Artificial intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence.",
                        "source_url": "https://example.com/ai-intro",
                        "score": 0.95,
                        "metadata": {}
                    },
                    {
                        "id": "chunk_2",
                        "content": "Machine learning is a subset of artificial intelligence that focuses on building systems that learn from data.",
                        "source_url": "https://example.com/ml-overview",
                        "score": 0.89,
                        "metadata": {}
                    }
                ]
                
                with patch.object(agent, '_retrieve_content', return_value=mock_retrieved_chunks):
                    # Execute the query
                    result = agent.process_query("What is artificial intelligence?")
                    
                    # Verify the result
                    assert isinstance(result, AgentResponse)
                    assert result.answer == 'Artificial intelligence is a branch of computer science that aims to create software or machines that exhibit human-like intelligence. Machine learning is a subset of AI that focuses on building systems that learn from data.'

    def test_query_processing_with_follow_up_context(self):
        """Test query processing with follow-up context"""
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
                mock_retrieved_chunks = [
                    {
                        "id": "chunk_1",
                        "content": "Robots often incorporate machine learning algorithms to improve their performance.",
                        "source_url": "https://example.com/robotics-ml",
                        "score": 0.88,
                        "metadata": {}
                    }
                ]
                
                with patch.object(agent, '_retrieve_content', return_value=mock_retrieved_chunks):
                    # Execute the query with thread_id to simulate follow-up
                    result = agent.process_query("How do they learn?", thread_id="existing-thread-id")
                    
                    # Verify the result
                    assert isinstance(result, AgentResponse)
                    assert result.answer == 'Robots use machine learning to improve their performance.'