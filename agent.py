"""
AI Robotics Book Agent
Uses OpenAI Agent SDK to create a conversational agent that can answer questions about robotics
by retrieving relevant information from the book content.
"""

import os
from typing import Dict, List, Optional, Any
from dotenv import load_dotenv
from openai import OpenAI
from retrieve import RetrievalService
from backend.src.models.source_reference import SourceReference

# Load environment variables
load_dotenv()

class RoboticsBookAgent:
    """
    An agent that uses OpenAI's API to answer questions about robotics
    by retrieving relevant information from the book content.
    """
    
    def __init__(self):
        """
        Initialize the Robotics Book Agent.
        """
        # Get OpenAI API key from environment
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        if not self.openai_api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")
        
        # Initialize OpenAI client
        self.client = OpenAI(api_key=self.openai_api_key)
        
        # Initialize the retrieval service
        try:
            self.retrieval_service = RetrievalService()
        except Exception as e:
            print(f"Warning: Could not initialize retrieval service: {e}")
            self.retrieval_service = None
        
        # Store conversation history
        self.conversation_history = []
    
    def retrieve_context(self, query: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """
        Retrieve relevant context from the book using the retrieval service.

        Args:
            query: The user's query
            top_k: Number of top results to retrieve

        Returns:
            List of retrieved documents with metadata
        """
        if not self.retrieval_service:
            return []

        try:
            # Import the actual retrieval service and call its search method
            # We'll need to capture the output of the search method
            import io
            import contextlib

            # Capture the output of the search method
            f = io.StringIO()
            with contextlib.redirect_stdout(f):
                # Call the search method with the query
                # Since the search method in retrieve.py is interactive, we'll need to call it differently
                # For now, we'll use the embed and query_points methods directly
                embedding = self.retrieval_service.embed(query)
                if embedding is None:
                    print("Embedding failed. Cannot search.")
                    return []

                try:
                    results = self.retrieval_service.qdrant.query_points(
                        collection_name=self.retrieval_service.collection,
                        query=embedding,
                        limit=top_k,
                        with_payload=True
                    ).points

                    if not results:
                        print("No relevant chunks found.")
                        return []

                    # Process the results into the format we need
                    processed_results = []
                    for i, point in enumerate(results, start=1):
                        payload = point.payload or {}

                        text = (
                            payload.get("content")
                            or payload.get("text")
                            or payload.get("page_content")
                            or payload.get("body")
                            or "No text available"
                        )
                        title = payload.get("title", "Untitled")
                        source_url = payload.get("source_url", "No URL available")
                        page_number = payload.get("page_number", None)

                        result = {
                            "title": title,
                            "url": source_url,
                            "page_number": page_number,
                            "relevance_score": point.score,
                            "content_snippet": text[:400] + "..." if len(text) > 400 else text
                        }

                        processed_results.append(result)

                    return processed_results

                except Exception as e:
                    print(f"Qdrant search error: {str(e)}")
                    return []

        except Exception as e:
            print(f"Error during retrieval: {e}")
            return []
    
    def generate_response(self, query: str, context: List[Dict[str, Any]]) -> str:
        """
        Generate a response using OpenAI's API based on the query and retrieved context.
        Falls back to a simple response if OpenAI API is unavailable.

        Args:
            query: The user's query
            context: Retrieved context from the book

        Returns:
            Generated response string
        """
        # Format the context for the prompt
        context_str = ""
        if context:
            context_str = "Relevant information from the robotics book:\n\n"
            for i, doc in enumerate(context, 1):
                context_str += f"{i}. {doc['title']} (Page {doc.get('page_number', 'N/A')}):\n"
                context_str += f"   {doc['content_snippet']}\n\n"
        else:
            context_str = "No specific information was found in the book for this query. "

        # Create the full prompt
        prompt = f"""
        {context_str}

        User Query: {query}

        Please provide a comprehensive answer to the user's query based on the provided context from the AI Robotics Book.
        If the context doesn't fully address the query, use general robotics knowledge to supplement the response.
        Always cite the relevant chapters or sections when possible.

        Format your response in a clear, educational manner suitable for someone learning about robotics.
        """

        try:
            # Call OpenAI API to generate response
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",  # You can change this to gpt-4 if preferred
                messages=[
                    {"role": "system", "content": "You are an expert assistant for the AI Robotics Book. Your role is to answer questions about robotics based on the content of the book and general robotics knowledge. Always provide educational, accurate, and helpful responses."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.7,
                max_tokens=1000
            )

            return response.choices[0].message.content.strip()

        except Exception as e:
            print(f"Error generating response with OpenAI: {e}")
            # Fallback to a response based on the retrieved context
            if context:
                fallback_response = f"Regarding your query '{query}', I found the following information in the robotics book:\n\n"
                for doc in context:
                    fallback_response += f"- {doc['title']}: {doc['content_snippet']}\n\n"
                fallback_response += "This information comes from the AI Robotics Book. For more details, please refer to the cited sections."
                return fallback_response
            else:
                return f"I'm sorry, but I couldn't find specific information about '{query}' in the robotics book. Please consult the relevant chapters for more information."
    
    def process_query(self, query: str) -> Dict[str, Any]:
        """
        Process a user query end-to-end: retrieve context and generate response.

        Args:
            query: The user's query

        Returns:
            Dictionary containing response and sources
        """
        # Retrieve relevant context
        context = self.retrieve_context(query)

        # Generate response based on query and context
        response_text = self.generate_response(query, context)

        # Convert context to SourceReference objects
        sources = []
        for ctx in context:
            source_ref = SourceReference(
                title=ctx["title"],
                url=ctx["url"],
                page_number=ctx.get("page_number"),
                relevance_score=float(ctx["relevance_score"]) if isinstance(ctx["relevance_score"], (int, float)) else 0.0,
                content_snippet=str(ctx["content_snippet"]) if ctx["content_snippet"] else ""
            )
            sources.append(source_ref)

        # Return the response with sources
        return {
            "response": response_text,
            "sources": sources
        }
    
    def reset_conversation(self):
        """
        Reset the conversation history.
        """
        self.conversation_history = []


# Example usage
if __name__ == "__main__":
    agent = RoboticsBookAgent()
    
    # Example query
    query = "What is robot localization?"
    result = agent.process_query(query)
    
    print(f"Query: {query}")
    print(f"Response: {result['response']}")
    print(f"Sources: {len(result['sources'])} sources found")