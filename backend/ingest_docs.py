#!/usr/bin/env python3
"""
Documentation Ingestion Script for RAG Chatbot
Crawls documentation and indexes it in the vector database
"""
import os
import sys
from pathlib import Path

def main():
    print("📚 Starting Documentation Ingestion Process")
    print("="*50)
    
    # Check if required environment variables are set
    required_vars = [
        'GOOGLE_API_KEY',
        'QDRANT_URL',
        'QDRANT_API_KEY',
        'NEON_DB_URL'
    ]
    
    missing_vars = []
    for var in required_vars:
        if not os.getenv(var):
            missing_vars.append(var)
    
    if missing_vars:
        print(f"ERROR: Missing required environment variables: {', '.join(missing_vars)}")
        print("\nPlease set these variables in your .env file or environment:")
        print("- GOOGLE_API_KEY: Your Google API key for Gemini")
        print("- QDRANT_URL: URL for your Qdrant instance")
        print("- QDRANT_API_KEY: API key for Qdrant")
        print("- NEON_DB_URL: Connection string for Neon PostgreSQL database")
        sys.exit(1)
    
    print("✓ All required environment variables are set")
    
    # Change to backend directory
    backend_dir = Path(__file__).parent
    os.chdir(backend_dir)
    
    # Import and run the ingestion service
    try:
        from src.services.ingestion_service import IngestionService
        from src.services.embedding_service import EmbeddingService
        from src.services.qdrant_service import QdrantService
        
        print("🔗 Initializing services...")
        
        # Initialize services
        embedding_service = EmbeddingService()
        qdrant_service = QdrantService()
        ingestion_service = IngestionService(embedding_service, qdrant_service)
        
        print("✅ Services initialized successfully")
        
        # Get source URL from command line argument or environment
        if len(sys.argv) > 1:
            source_url = sys.argv[1]
        else:
            source_url = os.getenv("DOC_SOURCE_URL", "https://your-documentation-site.com")
        
        print(f"🔍 Starting ingestion from: {source_url}")
        
        # Perform the ingestion
        result = ingestion_service.ingest_documentation(source_url, force_reindex=True)
        
        print("🎉 Ingestion completed successfully!")
        print(f"📊 Results: {result}")
        
    except ImportError as e:
        print(f"❌ Error importing ingestion service: {e}")
        print("💡 Make sure all dependencies are installed and paths are correct")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error during ingestion: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()