#!/usr/bin/env python3
"""
Production-ready startup script for the RAG Chatbot API
"""
import os
import sys
import asyncio
from contextlib import asynccontextmanager

from fastapi import FastAPI
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import API routers after environment is loaded
from src.api import chat, ingestion

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    print("🚀 Initializing RAG Chatbot services...")
    
    # Initialize services if needed
    # This is where you could set up connections or load models
    
    print("✅ RAG Chatbot services initialized")
    yield
    # Shutdown
    print("🛑 Shutting down RAG Chatbot services...")

app = FastAPI(
    title="RAG Chatbot API",
    description="API for the RAG Chatbot integrated with Docusaurus documentation sites",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware if needed
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}

# Include API routes
app.include_router(chat.router, prefix="/api")
app.include_router(ingestion.router, prefix="/api")

def main():
    import uvicorn
    
    # Check for required environment variables
    required_vars = ['GOOGLE_API_KEY', 'QDRANT_URL', 'NEON_DB_URL']
    missing_vars = [var for var in required_vars if not os.getenv(var)]
    
    if missing_vars:
        print(f"❌ ERROR: Missing required environment variables: {', '.join(missing_vars)}")
        print("💡 Please set these variables in your .env file or environment")
        sys.exit(1)
    
    print("✅ All required environment variables are set")
    print("🚀 Starting RAG Chatbot API server...")
    
    uvicorn.run(
        "rag_server:app",
        host="0.0.0.0",
        port=8000,
        reload=False,  # Set to True for development
        log_level="info"
    )

if __name__ == "__main__":
    main()