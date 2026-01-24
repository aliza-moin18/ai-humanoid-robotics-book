# Book Embeddings Ingestion System

This project implements a system for ingesting book content from URLs, generating embeddings using Cohere, and storing them in a Qdrant vector database for semantic search.

## Features

- **Content Ingestion**: Crawl and extract text content from book URLs
- **Text Processing**: Clean and chunk text for optimal embedding
- **Embedding Generation**: Generate vector embeddings using Cohere's models
- **Vector Storage**: Store embeddings in Qdrant for efficient retrieval
- **Search Capability**: Perform semantic search on stored content

## Prerequisites

- Python 3.11 or higher
- `uv` package manager
- Cohere API key
- Qdrant Cloud account and API key

## Setup

### 1. Clone the repository
```bash
git clone <repository-url>
cd ai-robotics-book
```

### 2. Navigate to the backend directory
```bash
cd backend
```

### 3. Install dependencies using uv
```bash
uv venv  # Create virtual environment
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install -r requirements.txt
```

### 4. Set up environment variables
Create a `.env` file in the backend directory with the following content:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_HOST=your_qdrant_cluster_url
QDRANT_COLLECTION_NAME=book_embeddings
CHUNK_SIZE=512
CHUNK_OVERLAP=50
LOG_LEVEL=INFO
```

## Usage

### Run the full pipeline
```bash
python -m src.cli.main --url https://example-book.vercel.app --process-all
```

### Run individual steps
```bash
# Step 1: Crawl and extract content
python -m src.cli.main --url https://example-book.vercel.app --crawl-only

# Step 2: Process and chunk text
python -m src.cli.main --url https://example-book.vercel.app --process-text

# Step 3: Generate embeddings
python -m src.cli.main --url https://example-book.vercel.app --generate-embeddings

# Step 4: Store in vector database
python -m src.cli.main --url https://example-book.vercel.app --store-embeddings
```

## Configuration Options

### Command-line arguments:
- `--url`: The book URL to process
- `--process-all`: Run the entire pipeline end-to-end
- `--crawl-only`: Only crawl and extract content
- `--process-text`: Only process and chunk text
- `--generate-embeddings`: Only generate embeddings
- `--store-embeddings`: Only store embeddings in the database
- `--search`: Search query for testing
- `--top-k`: Number of top results to return in search (default: 5)

### Environment variables:
- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_API_KEY`: Your Qdrant API key
- `QDRANT_HOST`: Your Qdrant cluster URL
- `QDRANT_COLLECTION_NAME`: Name of the collection to store embeddings
- `CHUNK_SIZE`: Size of text chunks (default: 512)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 50)
- `LOG_LEVEL`: Logging level (default: INFO)

## Architecture

The system is organized into the following modules:

- `src/models/`: Data models for book content, text chunks, and embeddings
- `src/services/`: Core services for crawling, embedding, and storage
- `src/cli/`: Command-line interface
- `src/lib/`: Configuration, logging, and utility functions
- `tests/`: Unit and integration tests

## Testing

### Run unit tests:
```bash
python -m pytest tests/unit/
```

### Run integration tests:
```bash
python -m pytest tests/integration/
```

### Run all tests:
```bash
python -m pytest
```

## Troubleshooting

### Common Issues:

1. **API Rate Limits**: If you encounter rate limit errors, implement appropriate delays or upgrade your API plan.

2. **Memory Issues**: For large books, consider processing in smaller batches or increasing available memory.

3. **Connection Errors**: Verify your Cohere and Qdrant API keys and connection settings.

### Enable Debug Logging:
```bash
export LOG_LEVEL=DEBUG
python -m src.cli.main --url https://example-book.vercel.app --process-all
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.