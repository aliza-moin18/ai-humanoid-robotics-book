"""
Main CLI entry point for the book embeddings ingestion system.
"""
import argparse
import sys
import time
from typing import List

from src.services.crawler_service import CrawlerService
from src.services.embedding_service import EmbeddingService
from src.services.storage_service import StorageService
from src.config import config  
from src.lib.logger import app_logger
from src.lib.errors import BookIngestionError
from src.models.book_content import BookContent  

def run_full_pipeline(url: str):
    """
    Run the complete ingestion pipeline: crawl, chunk, embed, store.
    Now saves actual text and source_url in payload.
    """
    print("Starting full ingestion pipeline...")

    # Step 1: Crawl content
    print("Step 1: Crawling content...")
    crawler = CrawlerService()
    book_contents: List[BookContent] = crawler.crawl(url)

    if not book_contents:
        raise BookIngestionError("No content was crawled from the provided URL.")

    print(f"Successfully crawled {len(book_contents)} page(s).")
    for bc in book_contents:
        print(f"   - Page: {bc.title[:60]}... (ID: {bc.id}, Words: {bc.word_count})")

    # Step 2: Process and chunk text
    print("Step 2: Processing and chunking text...")
    text_chunks = crawler.chunk_content(
        book_contents,
        config.CHUNK_SIZE,
        config.CHUNK_OVERLAP
    )
    print(f"Created {len(text_chunks)} text chunks from all pages.")

    # Step 3: Generate embeddings
    print("Step 3: Generating embeddings...")
    embedding_service = EmbeddingService()
    embedding_vectors = embedding_service.generate_embeddings(text_chunks)
    print(f"Generated {len(embedding_vectors)} embeddings.")

    # Step 4: Storing embeddings in vector database...
    print("Step 4: Storing embeddings in vector database...")
    storage_service = StorageService()
    stored_ids = []

    for idx, (chunk_text, embedding) in enumerate(zip(text_chunks, embedding_vectors)):
        payload = {
            "text": chunk_text,  # Real chunk text
            "source_url": book_contents[0].source_url if book_contents else "https://ai-humanoid-robotics-book-tau.vercel.app",
            "title": book_contents[0].title if book_contents else "Untitled",
            "chunk_index": idx,
            "created_at": time.time()
        }

        # Store with payload
        stored = storage_service.store_embeddings([embedding], payload=[payload])
        stored_ids.extend(stored)

    print(f"Successfully stored {len(stored_ids)} embeddings in Qdrant!")
    print("Full pipeline completed successfully! 🎉")
    print(f"Total pages crawled: {len(book_contents)}")
    print(f"Total chunks created: {len(text_chunks)}")
    print(f"Total vectors stored: {len(stored_ids)}")


def main():
    parser = argparse.ArgumentParser(description="Book Embeddings Ingestion CLI")
    parser.add_argument("--url", type=str, help="URL of the book to process (e.g. Docusaurus site)")
    parser.add_argument("--process-all", action="store_true", help="Run the entire pipeline end-to-end")
    parser.add_argument("--crawl-only", action="store_true", help="Only crawl and extract content")
    parser.add_argument("--process-text", action="store_true", help="Only process and chunk text")
    parser.add_argument("--generate-embeddings", action="store_true", help="Only generate embeddings")
    parser.add_argument("--store-embeddings", action="store_true", help="Only store embeddings")
    parser.add_argument("--search", type=str, help="Search query for testing")
    parser.add_argument("--top-k", type=int, default=5, help="Number of top results")

    args = parser.parse_args()

    if not any([args.process_all, args.crawl_only, args.process_text,
                args.generate_embeddings, args.store_embeddings, args.search, args.url]):
        if len(sys.argv) == 1:
            parser.print_help()
            sys.exit(1)

    try:
        if args.url and args.process_all:
            run_full_pipeline(args.url)

        elif args.url and args.crawl_only:
            crawler = CrawlerService()
            book_contents = crawler.crawl(args.url)
            print(f"Successfully crawled {len(book_contents)} page(s):")
            for bc in book_contents:
                print(f"   - {bc.title} (ID: {bc.id}, Words: {bc.word_count})")

        elif args.url and args.process_text:
            crawler = CrawlerService()
            book_contents = crawler.crawl(args.url)
            text_chunks = crawler.chunk_content(book_contents, config.CHUNK_SIZE, config.CHUNK_OVERLAP)
            print(f"Created {len(text_chunks)} text chunks from {len(book_contents)} pages.")

        elif args.url and args.generate_embeddings:
            crawler = CrawlerService()
            book_contents = crawler.crawl(args.url)
            text_chunks = crawler.chunk_content(book_contents, config.CHUNK_SIZE, config.CHUNK_OVERLAP)
            embedding_service = EmbeddingService()
            vectors = embedding_service.generate_embeddings(text_chunks)
            print(f"Generated {len(vectors)} embeddings.")

        elif args.url and args.store_embeddings:
            crawler = CrawlerService()
            book_contents = crawler.crawl(args.url)
            text_chunks = crawler.chunk_content(book_contents, config.CHUNK_SIZE, config.CHUNK_OVERLAP)
            embedding_service = EmbeddingService()
            vectors = embedding_service.generate_embeddings(text_chunks)
            storage_service = StorageService()
            stored = storage_service.store_embeddings(vectors)
            print(f"Stored {len(stored)} embeddings in Qdrant.")

        elif args.search:
            print(f"Search functionality for: '{args.search}' (Top {args.top_k})")
            # Future search implementation
            print("Search would return relevant chunks here.")

        else:
            if args.url:
                print(f"URL provided but no action specified: {args.url}")
            parser.print_help()

    except BookIngestionError as e:
        app_logger.error(f"Ingestion error: {str(e)}")
        print(f"Error: {str(e)}")
        sys.exit(1)
    except Exception as e:
        app_logger.error(f"Unexpected error: {str(e)}")
        print(f"Unexpected error: {str(e)}")
        sys.exit(1)


if __name__ == "__main__":
    main()