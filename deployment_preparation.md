# Deployment Preparation for RAG Agent API

This document outlines the steps to prepare the RAG Agent API for deployment to a staging environment.

## Environment Variables

Before deploying, ensure the following environment variables are set:

- `COHERE_API_KEY`: Your Cohere API key for embeddings
- `QDRANT_URL`: URL for the Qdrant vector database
- `QDRANT_API_KEY`: API key for the Qdrant database
- `QDRANT_COLLECTION_NAME`: Name of the collection in QDRant (default: rag_chatbot_book)

## Deployment Steps

1. **Install Dependencies**:
   ```bash
   pip install -r requirements.txt
   ```

2. **Set Environment Variables**:
   ```bash
   export COHERE_API_KEY=your_cohere_api_key
   export QDRANT_URL=your_qdrant_url
   export QDRANT_API_KEY=your_qdrant_api_key
   export QDRANT_COLLECTION_NAME=rag_chatbot_book
   ```

3. **Run the Application**:
   ```bash
   uvicorn api:app --host 0.0.0.0 --port 8000 --workers 4
   ```

## Docker Deployment (Optional)

A Dockerfile is provided for containerized deployment:

1. Build the image:
   ```bash
   docker build -t rag-agent-api .
   ```

2. Run the container:
   ```bash
   docker run -d -p 8000:8000 \
     -e COHERE_API_KEY=your_cohere_api_key \
     -e QDRANT_URL=your_qdrant_url \
     -e QDRANT_API_KEY=your_qdrant_api_key \
     rag-agent-api
   ```

## Health Checks

The API provides a health check endpoint at `/health` that returns a 200 status when the service is operational.

## Monitoring

The application logs key events and errors to standard output. For production deployments, ensure that logs are properly collected and monitored.