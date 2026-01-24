"""
Service layer to interface with the RAG agent
Handles the interaction with the agent from agent.py
"""

from typing import Dict, Any, Optional
import uuid
from datetime import datetime
from ..models.query_request import QueryRequest
from ..models.rag_response import RAGAgentResponse
from ..models.source_reference import SourceReference
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from agent import RoboticsBookAgent


class RAGService:
    """
    Service class to handle interactions with the RAG agent.
    """

    def __init__(self):
        """
        Initialize the RAG service.
        """
        try:
            self.agent = RoboticsBookAgent()
            self.agent_available = True
        except Exception as e:
            print(f"Failed to initialize agent: {e}")
            self.agent_available = False

    async def process_query(self, query_request: QueryRequest) -> RAGAgentResponse:
        """
        Process a query using the RAG agent and return a response.

        Args:
            query_request: The query request containing the user's query

        Returns:
            RAGAgentResponse: The response from the RAG agent
        """
        if not self.agent_available:
            raise Exception("RAG service is not available")

        try:
            # Process the query using the agent
            result = self.agent.process_query(query_request.query)

            # Extract the response and sources
            response_text = result["response"]
            sources = [source.dict() for source in result["sources"]]

            # Create and return the response
            response = RAGAgentResponse(
                response=response_text,
                sources=sources,
                session_id=query_request.session_id,
                query_id=str(uuid.uuid4()),
                timestamp=datetime.now().isoformat()
            )

            return response
        except Exception as e:
            # Handle any errors that occur during query processing
            error_msg = f"An error occurred while processing the query: {str(e)}"
            response_text = f"Sorry, I encountered an issue while processing your query: {error_msg}"

            response = RAGAgentResponse(
                response=response_text,
                sources=[],
                session_id=query_request.session_id,
                query_id=str(uuid.uuid4()),
                timestamp=datetime.now().isoformat()
            )

            return response

    async def health_check(self) -> bool:
        """
        Check if the RAG service is available.

        Returns:
            bool: True if the service is available, False otherwise
        """
        return self.agent_available