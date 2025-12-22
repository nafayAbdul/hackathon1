from fastapi import APIRouter, HTTPException
from typing import Optional
import logging

from src.services.ingestion_service import IngestionService
from src.services.embedding_service import EmbeddingService
from src.services.qdrant_service import QdrantService

logger = logging.getLogger(__name__)
router = APIRouter()

# Initialize services - in a real app, use dependency injection
embedding_service = EmbeddingService()
qdrant_service = QdrantService()
ingestion_service = IngestionService(embedding_service, qdrant_service)

@router.post("/ingest")
async def ingest_documentation(source_url: str, force_reindex: Optional[bool] = False):
    """Crawl and index documentation content into the vector database"""
    try:
        # Perform the ingestion
        result = ingestion_service.ingest_documentation(source_url, force_reindex)
        
        logger.info(f"Ingestion completed: {result['message']}")
        return result
    except Exception as e:
        logger.error(f"Error during ingestion: {e}")
        raise HTTPException(status_code=500, detail=f"Ingestion failed: {str(e)}")