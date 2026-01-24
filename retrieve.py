"""
Book Retrieval Terminal Tool
Clean, Video-like Output
API-ready (future HTTPS support)
"""

from dotenv import load_dotenv
import os
import time
import json
from cohere import Client as CohereClient
from qdrant_client import QdrantClient

load_dotenv()

COHERE_KEY = os.getenv("COHERE_API_KEY")
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION = os.getenv("QDRANT_COLLECTION_NAME", "rag_chatbot_book")
DEPLOY_URL = os.getenv(
    "DEPLOY_VERCEL_URL",
    "https://ai-humanoid-robotics-book-tau.vercel.app"
)

print("COHERE KEY LOADED:", "YES" if COHERE_KEY else "MISSING!")
print("QDRANT URL:", QDRANT_URL or "MISSING!")
print("Collection:", COLLECTION)

class RetrievalService:
    def __init__(self):
        if not COHERE_KEY:
            raise RuntimeError("COHERE_API_KEY missing in .env")

        if not QDRANT_URL or not QDRANT_API_KEY:
            raise RuntimeError("QDRANT_URL or QDRANT_API_KEY missing in .env")

        self.cohere = CohereClient(api_key=COHERE_KEY)
        self.qdrant = QdrantClient(
            url=QDRANT_URL,
            api_key=QDRANT_API_KEY
        )
        self.collection = COLLECTION
        print("Ready! Collection:", self.collection)

    def embed(self, text: str):
        try:
            return self.cohere.embed(
                texts=[text],
                model="embed-multilingual-v3.0",  # yeh wahi model hai jo tumne upload mein use kiya tha
                input_type="search_query"
            ).embeddings[0]
        except Exception as e:
            print("Embedding error:", str(e))
            return None

    def search(self, query: str, top_k: int = 5):
        start_time = time.time()

        # Header (video style)
        print("\n" + "=" * 80)
        print(f"Source URL : {DEPLOY_URL}")
        print("Title      : Physical AI & Humanoid Robotics Textbook")
        print(f"Query      : {query}")
        print("=" * 80)

        embedding = self.embed(query)
        if embedding is None:
            print("Embedding failed. Cannot search.")
            return

        try:
            results = self.qdrant.query_points(
                collection_name=self.collection,
                query=embedding,
                limit=top_k,
                with_payload=True
            ).points

            if not results:
                print("No relevant chunks found.")
                return

            print(f"\nFound {len(results)} relevant chunks:\n")

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

                print(f"Result {i}")
                print(f"Score      : {point.score:.4f}")
                print(f"Title      : {title}")
                print(f"Source URL : {source_url}")
                print("Text Preview:")
                print(text[:400] + "..." if len(text) > 400 else text)
                print("-" * 80)

            elapsed = time.time() - start_time
            print(f"Performance: OK ({elapsed:.3f}s)")

        except Exception as e:
            print("Qdrant search error:", str(e))

# ---------------- RUNNER ---------------- #

if __name__ == "__main__":
    try:
        service = RetrievalService()
        print("\nEnter query (type 'exit' to quit):")

        while True:
            q = input("> ").strip()
            if q.lower() == "exit":
                break
            if not q:
                continue

            try:
                service.search(q)
            except Exception as e:
                print("Error:", str(e))
    except RuntimeError as e:
        print("Setup failed:", str(e))