import requests
from typing import List
import logging
from dotenv import load_dotenv
import os

load_dotenv()

logger = logging.getLogger(__name__)

class EmbeddingService:
    def __init__(self):
        self.api_key = os.getenv("GOOGLE_API_KEY")  # Use the correct env var name
        if not self.api_key:
            raise ValueError("GOOGLE_API_KEY environment variable is required")

        # Using the Google Generative Language endpoint for embeddings
        self.base_url = "https://generativelanguage.googleapis.com/v1beta/"

    def create_embedding(self, text: str) -> List[float]:
        """Create an embedding for the given text using Google's embedding API"""
        try:
            # Prepare the request to the Google embedding API
            headers = {
                "Content-Type": "application/json"
            }

            # Use the embedding API directly via Google's endpoint
            url = f"{self.base_url}models/text-embedding-004:embedContent"

            data = {
                "content": {
                    "parts": [
                        {
                            "text": text
                        }
                    ]
                },
                "outputDimensionality": 768  # Optional: specify output dimensionality
            }

            response = requests.post(
                f"{url}?key={self.api_key}",
                headers=headers,
                json=data
            )

            if response.status_code != 200:
                raise Exception(f"API request failed with status {response.status_code}: {response.text}")

            result = response.json()
            embedding = result['embedding']['values']

            logger.info(f"Successfully created embedding for text of length {len(text)}")
            return embedding
        except requests.exceptions.RequestException as e:
            logger.error(f"Network error creating embedding: {e}")
            raise
        except Exception as e:
            logger.error(f"Error creating embedding: {e}")
            raise

    def create_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """Create embeddings for a batch of texts"""
        embeddings = []
        for text in texts:
            embedding = self.create_embedding(text)
            embeddings.append(embedding)
        return embeddings