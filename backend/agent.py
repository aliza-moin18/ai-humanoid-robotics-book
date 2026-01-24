"""
RAG Agent – Video Style Output (OpenRouter + Qdrant Cloud + Cohere)
No quota, no Agents SDK
"""

import os
import json
import time
from dotenv import load_dotenv
import cohere
from openai import OpenAI
import requests

load_dotenv()

client = OpenAI(
    api_key=os.getenv("OPENROUTER_API_KEY"),
    base_url="https://openrouter.ai/api/v1",
)

co = cohere.Client(os.getenv("COHERE_API_KEY"))

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "chatbot")
CITATION_COUNT = int(os.getenv("CITATION_COUNT", "5"))
DEPLOY_URL = os.getenv("DEPLOY_VERCEL_URL", "https://ai-humanoid-robotics-book-tau.vercel.app")

MODEL_NAME = "mistralai/devstral-2512:free"

def embed_query(query: str):
    embed_response = co.embed(
        texts=[query],
        model="embed-multilingual-v3.0",
        input_type="search_query"
    )
    return embed_response.embeddings[0]

def search_qdrant(query_vector):
    response = requests.post(
        f"{QDRANT_URL}/collections/{COLLECTION_NAME}/points/search",
        headers={"api-key": QDRANT_API_KEY},
        json={"vector": query_vector, "limit": CITATION_COUNT, "with_payload": True},
        timeout=10
    )
    response.raise_for_status()
    return response.json()["result"]

def run_agent(query: str):
    print("\n" + "=" * 80)
    print(f"Source URL : {DEPLOY_URL}")
    print("Title      : Physical AI & Humanoid Robotics Textbook")
    print(f"Query      : {query}")
    print("=" * 80)

    start_time = time.time()

    query_vector = embed_query(query)
    results = search_qdrant(query_vector)

    if not results:
        print("No relevant chunks found.")
        return "No relevant information found in the book."

    print(f"\nFound {len(results)} relevant chunks:\n")

    context_parts = []
    sources = []
    for i, hit in enumerate(results, 1):
        payload = hit["payload"] or {}
        text = payload.get("content") or payload.get("text") or "No text available"
        title = payload.get("title", "Untitled")
        source_url = payload.get("source_url", "No URL available")
        score = hit["score"]

        print(f"Result {i}")
        print(f"Score      : {score:.4f}")
        print(f"Title      : {title}")
        print(f"Source URL : {source_url}")
        print("Text Preview:")
        print(text[:400] + "..." if len(text) > 400 else text)
        print("-" * 80)

        context_parts.append(f"Chunk {i} (score: {score:.4f}): {text}")
        sources.append({"title": title, "url": source_url, "score": score})

    elapsed = time.time() - start_time
    print(f"Performance: OK ({elapsed:.3f}s)")

    context = "\n\n".join(context_parts)

    prompt = f"""You are an expert assistant for the book "Physical AI & Humanoid Robotics Textbook".
Answer ONLY using the provided book content below. Do NOT add external knowledge or hallucinate.
If no relevant information, say "No information found in the book."

Question: {query}

Book content:
{context}

Answer clearly and concisely. Use bullet points if helpful. End with Sources:"""

    try:
        response = client.chat.completions.create(
            model=MODEL_NAME,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.25,
            max_tokens=2048
        )
        answer = response.choices[0].message.content.strip()

        sources_text = "\n\nSources:\n" + "\n".join(
            f"- {s['title']} (score: {s['score']:.2f}) - {s['url']}" for s in sources
        )

        final_answer = answer + sources_text

        print("\n" + "=" * 80)
        print("FINAL ANSWER")
        print("=" * 80)
        print(final_answer)
        print("=" * 80)

        return final_answer  # Added return here to fix null response in API

    except Exception as e:
        error_msg = f"Error generating answer: {str(e)}"
        print(error_msg)
        return error_msg

if __name__ == "__main__":
    print("RAG Agent (Video Style Output) Ready! Type 'exit' to quit\n")
    while True:
        q = input("> ").strip()
        if q.lower() == "exit":
            break
        if not q:
            continue
        run_agent(q)



















# import os
# os.environ["OPENAI_DISABLE_TELEMETRY"] = "true"
# os.environ["OPENAI_LOG_LEVEL"] = "error"
# os.environ["OPENAI_API_KEY"] = ""  # empty kar do – tracing call nahi jayegi

# # mypy: disable-error-code=import-not-found, attr-defined, misc, assignment, index, call-arg, union-attr, import-untyped

# """
# RAG Chatbot using OpenAI Agents SDK + OpenRouter + Qdrant Cloud + Cohere
# Video-style output
# """

# import json
# import time
# from dotenv import load_dotenv
# import cohere
# from qdrant_client import QdrantClient
# from agents import Agent, Runner, function_tool, ModelSettings
# from openai import AsyncOpenAI

# load_dotenv()

# # OpenRouter client
# openrouter_client = AsyncOpenAI(
#     api_key=os.getenv("OPENROUTER_API_KEY"),
#     base_url="https://openrouter.ai/api/v1",
# )

# # Cohere client
# co = cohere.Client(os.getenv("COHERE_API_KEY"))

# # Qdrant config
# QDRANT_URL = os.getenv("QDRANT_URL")
# QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
# COLLECTION_NAME = os.getenv("QDRANT_COLLECTION_NAME", "chatbot")
# CITATION_COUNT = int(os.getenv("CITATION_COUNT", "5"))
# DEPLOY_URL = os.getenv("DEPLOY_VERCEL_URL", "https://ai-humanoid-robotics-book-tau.vercel.app")

# MODEL_NAME = "mistralai/devstral-2512:free"

# @function_tool
# def retrieve_content(query: str) -> str:
#     try:
#         embed_response = co.embed(
#             texts=[query],
#             model="embed-multilingual-v3.0",
#             input_type="search_query"
#         )
#         query_vector = embed_response.embeddings[0]

#         qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

#         search_result = qdrant.search(
#             collection_name=COLLECTION_NAME,
#             query_vector=query_vector,
#             limit=CITATION_COUNT,
#             with_payload=True
#         )

#         results = []
#         for hit in search_result:
#             payload = hit.payload or {}
#             text = payload.get("content") or payload.get("text") or "No text available"
#             results.append({
#                 "content": text,
#                 "score": hit.score,
#                 "source": payload.get("source_url", f"Chunk {hit.id}")
#             })

#         return json.dumps({"results": results})
#     except Exception as e:
#         return json.dumps({"error": str(e)})

# # Agent (video wale jaisa structure – ModelSettings mein direct model string daala)
# rag_agent = Agent(
#     name="RAG Book Assistant",
#     model_settings=ModelSettings(
#         model=MODEL_NAME,  # ← yeh line video mein wrapper ke bajaye direct string hai
#         temperature=0.25,
#         max_tokens=2048,
#         openai_client=openrouter_client
#     ),
#     instructions="""
# You are a helpful RAG assistant for the book. ALWAYS call retrieve_content FIRST. Answer ONLY from retrieved content. End with Sources section.
# """,
#     tools=[retrieve_content]
# )

# def run_agent(query: str):
#     print("\n" + "=" * 80)
#     print(f"Source URL : {DEPLOY_URL}")
#     print("Title      : Physical AI & Humanoid Robotics Textbook")
#     print(f"Query      : {query}")
#     print("=" * 80)

#     start_time = time.time()

#     try:
#         result = Runner.run_sync(rag_agent, query)
#         answer = result.final_output

#         print(f"\nGenerated Answer:\n{answer}\n")
#         print(f"Performance: OK ({time.time() - start_time:.3f}s)")
#     except Exception as e:
#         print("Error:", str(e))

# if __name__ == "__main__":
#     print("RAG Chatbot Ready! Type 'exit' to quit\n")
#     while True:
#         q = input("> ").strip()
#         if q.lower() == "exit":
#             break
#         if not q:
#             continue
#         run_agent(q)