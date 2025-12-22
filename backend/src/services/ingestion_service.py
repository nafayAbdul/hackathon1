import requests
from bs4 import BeautifulSoup
from typing import List, Dict
from langchain_text_splitters import RecursiveCharacterTextSplitter
from src.services.embedding_service import EmbeddingService
from src.services.qdrant_service import QdrantService
from src.models.documentation_chunk import DocumentationChunk
import logging
import os
from urllib.parse import urljoin, urlparse

logger = logging.getLogger(__name__)

class IngestionService:
    def __init__(self, embedding_service: EmbeddingService, qdrant_service: QdrantService):
        self.embedding_service = embedding_service
        self.qdrant_service = qdrant_service
        
        # Initialize text splitter
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=1000,
            chunk_overlap=200,
            length_function=len,
        )
    
    def scrape_docusaurus_site(self, base_url: str) -> List[Dict]:
        """Scrape a Docusaurus site to extract documentation content"""
        all_content = []
        
        # Get the main page to find links
        response = requests.get(base_url)
        soup = BeautifulSoup(response.content, 'html.parser')
        
        # Find all internal links
        links = soup.find_all('a', href=True)
        urls_to_scrape = set()
        
        for link in links:
            href = link['href']
            full_url = urljoin(base_url, href)
            
            # Only add URLs from the same domain
            if urlparse(full_url).netloc == urlparse(base_url).netloc:
                urls_to_scrape.add(full_url)
        
        # Add the base URL as well
        urls_to_scrape.add(base_url)
        
        # Scrape each URL
        for url in urls_to_scrape:
            try:
                logger.info(f"Scraping: {url}")
                response = requests.get(url)
                soup = BeautifulSoup(response.content, 'html.parser')
                
                # Extract the main content (usually in main or article tags, or specific Docusaurus class)
                content_element = soup.find('main') or soup.find('article') or soup.find(class_='container')
                
                if content_element:
                    # Remove any script and style elements
                    for script in content_element(["script", "style"]):
                        script.decompose()
                    
                    # Get text content
                    content = content_element.get_text(strip=True, separator=' ')
                    
                    # Get the page title
                    title = soup.title.string if soup.title else url
                    
                    all_content.append({
                        'url': url,
                        'title': title,
                        'content': content
                    })
            except Exception as e:
                logger.error(f"Error scraping {url}: {e}")
        
        return all_content
    
    def chunk_and_embed_documentation(self, docs_content: List[Dict]) -> List[DocumentationChunk]:
        """Chunk documentation content and create embeddings"""
        chunks = []
        
        for doc in docs_content:
            # Split the content into chunks
            content_chunks = self.text_splitter.split_text(doc['content'])
            
            for i, chunk_text in enumerate(content_chunks):
                # Create an embedding for the chunk
                embedding = self.embedding_service.create_embedding(chunk_text)
                
                # Create a DocumentationChunk object
                chunk = DocumentationChunk(
                    content=chunk_text,
                    source_url=doc['url'],
                    source_title=doc['title'],
                    embedding=embedding,
                    metadata={
                        'chunk_index': i,
                        'total_chunks': len(content_chunks),
                        'original_doc_length': len(doc['content'])
                    }
                )
                
                chunks.append(chunk)
        
        return chunks
    
    def ingest_documentation(self, source_url: str, force_reindex: bool = False) -> Dict:
        """Main method to ingest documentation from a source URL"""
        try:
            logger.info(f"Starting ingestion from: {source_url}")
            
            # Scrape the site
            docs_content = self.scrape_docusaurus_site(source_url)
            logger.info(f"Scraped {len(docs_content)} pages")
            
            # Chunk and embed the content
            chunks = self.chunk_and_embed_documentation(docs_content)
            logger.info(f"Created {len(chunks)} content chunks with embeddings")
            
            # Store each chunk in Qdrant
            for chunk in chunks:
                self.qdrant_service.store_embedding(
                    chunk_id=chunk.id,
                    content=chunk.content,
                    embedding=chunk.embedding,
                    metadata={
                        'source_url': chunk.source_url,
                        'source_title': chunk.source_title,
                        'chunk_index': chunk.metadata.get('chunk_index'),
                        'total_chunks': chunk.metadata.get('total_chunks'),
                        'original_doc_length': chunk.metadata.get('original_doc_length')
                    }
                )
            
            logger.info(f"Successfully ingested {len(chunks)} chunks from {source_url}")
            
            return {
                "status": "success",
                "message": f"Ingested {len(chunks)} documentation chunks from {source_url}",
                "chunks_processed": len(chunks),
                "documents_processed": len(docs_content)
            }
        except Exception as e:
            logger.error(f"Error during ingestion: {e}")
            raise