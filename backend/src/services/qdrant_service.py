from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Optional
import logging
from dotenv import load_dotenv
import os

load_dotenv()

logger = logging.getLogger(__name__)

class QdrantService:
    def __init__(self, host: str = None, port: int = None, api_key: Optional[str] = None, url: Optional[str] = None):
        # Use environment variables if not provided
        if url is None:
            url = os.getenv("QDRANT_URL")
        if api_key is None:
            api_key = os.getenv("QDRANT_API_KEY")
        if host is None:
            host = os.getenv("QDRANT_HOST", "localhost")
        if port is None:
            port = int(os.getenv("QDRANT_PORT", 6333))

        if url:
            self.client = QdrantClient(url=url, api_key=api_key)
        else:
            self.client = QdrantClient(host=host, port=port, api_key=api_key)

        # Create the collection for documentation chunks if it doesn't exist
        self.collection_name = "documentation_chunks"
        self._create_collection()

    def _create_collection(self):
        """Create the collection for storing documentation chunks if it doesn't exist"""
        try:
            # Check if collection exists
            self.client.get_collection(self.collection_name)
            logger.info(f"Collection {self.collection_name} already exists")
        except:
            # Create the collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=768, distance=models.Distance.COSINE),  # Assuming 768-dim embeddings
            )
            logger.info(f"Created collection {self.collection_name}")

    def store_embedding(self, chunk_id: str, content: str, embedding: List[float], metadata: dict = None):
        """Store a documentation chunk with its embedding in Qdrant"""
        if metadata is None:
            metadata = {}

        # Add content and chunk_id to metadata
        metadata["content"] = content
        metadata["chunk_id"] = chunk_id

        self.client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=chunk_id,
                    vector=embedding,
                    payload=metadata
                )
            ]
        )

    def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[dict]:
        """Search for similar documentation chunks based on the query embedding"""
        results = self.client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=limit
        )

        # Format the results
        formatted_results = []
        for result in results:
            formatted_results.append({
                "id": result.id,
                "content": result.payload.get("content", ""),
                "source_url": result.payload.get("source_url", ""),
                "source_title": result.payload.get("source_title", ""),
                "relevance_score": result.score,
                "metadata": result.payload
            })

        return formatted_results