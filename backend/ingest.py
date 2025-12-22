#!/usr/bin/env python3
"""
Ingestion script for the RAG Chatbot.
This script crawls a Docusaurus site, chunks the content, creates embeddings,
and stores them in Qdrant for later retrieval.
"""

import sys
import os
import argparse
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Add the backend src directory to the path so we can import our modules
sys.path.append(os.path.join(os.path.dirname(__file__), 'src'))

from src.services.ingestion_service import IngestionService
from src.services.embedding_service import EmbeddingService
from src.services.qdrant_service import QdrantService


def main():
    parser = argparse.ArgumentParser(description='Ingest documentation content for RAG Chatbot')
    parser.add_argument('--source-url', required=True, help='URL of the documentation site to crawl')
    parser.add_argument('--force-reindex', action='store_true', help='Force reindexing of existing content')
    
    args = parser.parse_args()
    
    print(f"Starting ingestion from: {args.source_url}")
    
    try:
        # Initialize services
        embedding_service = EmbeddingService()
        qdrant_service = QdrantService()
        ingestion_service = IngestionService(embedding_service, qdrant_service)
        
        # Perform the ingestion
        result = ingestion_service.ingest_documentation(args.source_url, args.force_reindex)
        
        print(f"Ingestion completed successfully: {result['message']}")
        print(f"Documents processed: {result['documents_processed']}")
        print(f"Chunks created: {result['chunks_processed']}")
        
    except Exception as e:
        print(f"Error during ingestion: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()