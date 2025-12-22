from src.services.qdrant_service import QdrantService
from src.services.embedding_service import EmbeddingService
from typing import List, Dict
import logging

logger = logging.getLogger(__name__)

class RetrievalService:
    def __init__(self, qdrant_service: QdrantService, embedding_service: EmbeddingService):
        self.qdrant_service = qdrant_service
        self.embedding_service = embedding_service
    
    def retrieve_relevant_documents(self, query: str, limit: int = 5) -> List[Dict]:
        """Retrieve relevant documentation chunks based on the user query"""
        try:
            # Create an embedding for the query
            query_embedding = self.embedding_service.create_embedding(query)

            # Search for similar documents in Qdrant
            results = self.qdrant_service.search_similar(query_embedding, limit)

            logger.info(f"Retrieved {len(results)} relevant documents for query: {query[:50]}...")
            return results
        except Exception as e:
            logger.error(f"Error retrieving documents: {e}")
            # Return empty list instead of raising to allow graceful degradation
            return []
    
    def retrieve_with_context(self, query: str, context: str = None, limit: int = 5) -> List[Dict]:
        """Retrieve relevant documents, optionally incorporating additional context"""
        if context:
            # Combine the query with the context for better retrieval
            enhanced_query = f"{context} {query}"
        else:
            enhanced_query = query

        return self.retrieve_relevant_documents(enhanced_query, limit)

    def retrieve_with_highlighted_text(self, query: str, highlighted_text: str = None, limit: int = 5) -> List[Dict]:
        """Retrieve relevant documents, giving priority to those related to highlighted text"""
        if highlighted_text:
            # First, search specifically for content related to the highlighted text
            high_priority_results = self.retrieve_relevant_documents(highlighted_text, limit=limit//2)

            # Then search for content related to the query
            query_results = self.retrieve_relevant_documents(query, limit=limit)

            # Combine results, giving priority to those related to highlighted text
            combined_results = high_priority_results
            existing_ids = {result['id'] for result in high_priority_results}

            for result in query_results:
                if result['id'] not in existing_ids:
                    combined_results.append(result)
                    if len(combined_results) >= limit:
                        break

            return combined_results
        else:
            # If no highlighted text, just search with the query
            return self.retrieve_relevant_documents(query, limit)