from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from agent import run_agent

app = FastAPI(title="AI Robotics Book RAG API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

class Query(BaseModel):
    query: str

@app.post("/query")
async def query_endpoint(q: Query):
    print("DEBUG: Query received in api.py:", q.query)  # ← debug
    try:
        print("DEBUG: Calling run_agent...")  # ← debug
        answer = run_agent(q.query)
        print("DEBUG: run_agent returned:", answer)  # ← debug
        return {"response": answer}
    except Exception as e:
        print("DEBUG: Error in api.py:", str(e))  # ← debug
        return {"error": str(e), "response": null}