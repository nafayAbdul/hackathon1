from fastapi import FastAPI
from dotenv import load_dotenv
from src.middleware.rate_limit import rate_limit_middleware
from fastapi.middleware.cors import CORSMiddleware

# Load environment variables
load_dotenv()

app = FastAPI(
    title="RAG Chatbot API",
    description="API for the RAG Chatbot integrated with Docusaurus documentation sites",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, change this to your specific domain
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add middleware
app.middleware("http")(rate_limit_middleware)

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

# Include API routes
from src.api import chat, ingestion
app.include_router(chat.router, prefix="/api")
app.include_router(ingestion.router, prefix="/api")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)