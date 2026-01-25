# Quickstart Guide: RAG-FastAPI Integration

## Overview
This guide provides instructions for setting up and running the RAG-FastAPI integration for the AI Robotics Book project.

## Prerequisites
- Python 3.11+
- pip package manager
- Git
- Node.js and npm (for frontend development)

## Setup Instructions

### 1. Clone the Repository
```bash
git clone <repository-url>
cd ai-robotics-book
```

### 2. Set Up Backend (FastAPI Server)
1. Navigate to the project root:
   ```bash
   cd /path/to/project/root
   ```

2. Create a virtual environment:
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install fastapi uvicorn openai python-dotenv
   # Install any other dependencies required by the agent
   pip install -r requirements.txt
   ```

4. Run the FastAPI server:
   ```bash
   uvicorn api:app --reload --port 8000
   ```
   
   The API will be available at `http://localhost:8000`

### 3. Set Up Frontend (Docusaurus)
1. Navigate to the frontend directory:
   ```bash
   cd book/frontend
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Start the development server:
   ```bash
   npm start
   ```
   
   The frontend will be available at `http://localhost:3000`

## API Usage

### Making Queries
Send a POST request to the `/query` endpoint:

```bash
curl -X POST http://localhost:8000/query \
  -H "Content-Type: application/json" \
  -d '{"query": "How does robot localization work?"}'
```

### Expected Response
```json
{
  "response": "Robot localization is the process of determining a robot's position and orientation in a given environment...",
  "sources": [
    {
      "title": "Chapter 5: Robot Localization Techniques",
      "url": "/module-3/ai-robot-brain/chapter-5-localization",
      "page_number": 127,
      "relevance_score": 0.92
    }
  ],
  "session_id": "a1b2c3d4-e5f6-7890-1234-567890abcdef",
  "query_id": "q1r2s3t4-u5v6-7890-1234-567890fedcba",
  "timestamp": "2023-10-01T12:00:05Z"
}
```

## Frontend Integration

### Adding the Chat Interface
The frontend includes a chatbot UI that communicates with the backend API. To use it:

1. The chat interface is available on the main page of the book
2. Enter your question in the input field
3. The system will send the query to the backend and display the response

### Configuration
The frontend is configured to communicate with the backend API at `http://localhost:8000` by default. To change this, update the API endpoint in the frontend configuration.

## Troubleshooting

### Common Issues
1. **Port Already in Use**: If port 8000 is already in use, change the port in the uvicorn command:
   ```bash
   uvicorn api:app --reload --port 8001
   ```

2. **Dependency Issues**: If you encounter dependency conflicts, try creating a fresh virtual environment:
   ```bash
   deactivate
   rm -rf venv
   python -m venv venv
   source venv/bin/activate
   pip install -r requirements.txt
   ```

3. **CORS Issues**: If you encounter CORS errors, ensure the backend is configured to allow requests from the frontend origin.

## Development Tips

### Backend Development
- The main API code is in `api.py` at the project root
- The RAG agent integration is handled in the query endpoint
- Use the `/docs` endpoint to view the automatically generated API documentation

### Frontend Development
- The Docusaurus configuration is in `docusaurus.config.ts`
- The chat interface components are in the `src/components` directory
- Use hot reloading during development for faster iteration

## Next Steps
1. Explore the API documentation at `http://localhost:8000/docs`
2. Customize the chat interface to match your branding
3. Add additional endpoints as needed for your use case
4. Implement authentication if required for production use