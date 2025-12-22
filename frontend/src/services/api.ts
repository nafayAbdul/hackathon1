// API service for chatbot functionality
const API_BASE_URL = process.env.REACT_APP_API_URL || 'http://localhost:8000/api';

class ApiService {
  // Start a new chat session
  static async startSession(userData = {}) {
    try {
      const response = await fetch(`${API_BASE_URL}/chat/start`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(userData),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error starting session:', error);
      throw error;
    }
  }

  // Send a message in a chat session
  static async sendMessage(sessionId, message, highlightedText = null) {
    try {
      const response = await fetch(`${API_BASE_URL}/chat/${sessionId}/message`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          content: message,
          highlighted_text: highlightedText
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error sending message:', error);
      throw error;
    }
  }

  // Get chat history for a session
  static async getChatHistory(sessionId) {
    try {
      const response = await fetch(`${API_BASE_URL}/chat/${sessionId}/history`);

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error getting chat history:', error);
      throw error;
    }
  }

  // Ingest documentation content
  static async ingestDocumentation(sourceUrl, forceReindex = false) {
    try {
      const response = await fetch(`${API_BASE_URL}/ingest`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          source_url: sourceUrl,
          force_reindex: forceReindex
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      return await response.json();
    } catch (error) {
      console.error('Error ingesting documentation:', error);
      throw error;
    }
  }
}

export default ApiService;