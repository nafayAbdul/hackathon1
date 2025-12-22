import requests
from typing import List, Dict
import logging
from dotenv import load_dotenv
import os

load_dotenv()

logger = logging.getLogger(__name__)

class LLMService:
    def __init__(self):
        self.api_key = os.getenv("GOOGLE_API_KEY")
        if not self.api_key:
            raise ValueError("GOOGLE_API_KEY environment variable is required")

        # Using the Gemini 2.0 Flash model as specified in the requirements
        self.model = "gemini-2.0-flash"
        self.base_url = "https://generativelanguage.googleapis.com/v1beta"

    def generate_response(self, prompt: str, context: List[Dict] = None) -> str:
        """Generate a response using the LLM based on the prompt and optional context"""
        try:
            # Prepare the request to the Google Generative Language API
            headers = {
                "Content-Type": "application/json"
            }

            # Construct the full prompt with context if provided
            full_prompt = prompt
            if context:
                context_str = "Relevant documentation:\n"
                for doc in context:
                    context_str += f"- {doc.get('source_title', 'Title not available')}: {doc.get('content', '')[:200]}...\n"
                full_prompt = f"{context_str}\nUser question: {prompt}"

            # Create the request body
            data = {
                "contents": [{
                    "role": "user",
                    "parts": [{
                        "text": full_prompt
                    }]
                }],
                "generationConfig": {
                    "temperature": 0.7,
                    "maxOutputTokens": 500,
                    "candidateCount": 1
                }
            }

            # Make the API request
            url = f"{self.base_url}/models/{self.model}:generateContent?key={self.api_key}"
            response = requests.post(
                url,
                headers=headers,
                json=data
            )

            if response.status_code != 200:
                raise Exception(f"API request failed with status {response.status_code}: {response.text}")

            result = response.json()

            # Extract the generated text
            generated_text = result['candidates'][0]['content']['parts'][0]['text']
            logger.info(f"Successfully generated response for prompt of length {len(prompt)}")

            return generated_text
        except Exception as e:
            logger.error(f"Error generating response: {e}")
            # Return a helpful error message to the user instead of raising
            return "I'm sorry, but I'm currently unable to process your request. Please try again later."

    def generate_response_with_sources(self, prompt: str, context: List[Dict] = None) -> Dict:
        """Generate a response and return it with source information"""
        response_text = self.generate_response(prompt, context)

        # Extract source information from context
        sources = []
        if context:
            for doc in context:
                sources.append({
                    "url": doc.get("source_url", ""),
                    "title": doc.get("source_title", ""),
                    "snippet": doc.get("content", "")[:200] + "..." if len(doc.get("content", "")) > 200 else doc.get("content", "")
                })

        return {
            "response": response_text,
            "sources": sources
        }