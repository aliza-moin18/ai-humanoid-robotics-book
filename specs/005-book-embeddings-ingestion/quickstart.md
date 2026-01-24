# Quickstart Guide: Book Embeddings Ingestion

## Overview
This guide provides a quick introduction to setting up and running the book embeddings ingestion pipeline.

## Prerequisites
- Python 3.11 or higher
- `uv` package manager installed
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
```

## Running the Ingestion Pipeline

### 1. Run the full pipeline
```bash
python -m src.cli.main --url https://example-book.vercel.app --process-all
```

### 2. Run individual steps
```bash
# Step 1: Crawl and extract content
python -m src.cli.main --url https://example-book.vercel.app --crawl-only

# Step 2: Process and chunk text
python -m src.cli.main --process-text

# Step 3: Generate embeddings
python -m src.cli.main --generate-embeddings

# Step 4: Store in vector database
python -m src.cli.main --store-embeddings
```

## Configuration Options

### Command-line arguments:
- `--url`: The book URL to process
- `--process-all`: Run the entire pipeline end-to-end
- `--crawl-only`: Only crawl and extract content
- `--process-text`: Only process and chunk text
- `--generate-embeddings`: Only generate embeddings
- `--store-embeddings`: Only store embeddings in the database
- `--config`: Path to a custom configuration file

### Environment variables:
- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_API_KEY`: Your Qdrant API key
- `QDRANT_HOST`: Your Qdrant cluster URL
- `QDRANT_COLLECTION_NAME`: Name of the collection to store embeddings
- `CHUNK_SIZE`: Size of text chunks (default: 512)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 50)

## Example Usage

### Ingest a book and run the full pipeline:
```bash
python -m src.cli.main --url https://my-book.vercel.app --process-all
```

### Validate with test search queries:
```bash
python -m src.cli.main --search "machine learning concepts" --top-k 5
```

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