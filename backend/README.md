# RAG Chatbot for Docusaurus Documentation

This project implements a Retrieval-Augmented Generation (RAG) chatbot for Docusaurus documentation sites. The system allows users to ask questions about documentation content via a floating chat widget and receives AI-generated responses based on the site's content.

## Architecture

The system consists of:
- **Frontend**: React-based chat widget component for Docusaurus
- **Backend**: FastAPI-based API server
- **Vector Database**: Qdrant for storing document embeddings
- **Session Storage**: PostgreSQL (via Neon) for chat history
- **LLM**: Google Gemini via API

## Prerequisites

- Python 3.11+
- Node.js (for Docusaurus integration)
- Google API key for Gemini
- Qdrant cloud account or local instance
- Neon PostgreSQL database

## Setup Instructions

### 1. Backend Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install Python dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Create a `.env` file in the `backend` directory with your credentials:
   ```env
   # Google API Configuration
   GOOGLE_API_KEY=your_valid_google_api_key_here

   # Qdrant Configuration
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key

   # Neon Database Configuration
   NEON_DB_URL=your_neon_db_connection_string
   ```

### 2. Ingest Documentation

Before the chatbot can answer questions, you need to index your documentation:

```bash
python ingest_docs.py [your_documentation_url]
```

For example:
```bash
python ingest_docs.py https://your-docs-site.com
```

### 3. Start the Backend Server

Run the backend API server:

```bash
python start_server.py
```

The server will start at `http://localhost:8000`.

### 4. Frontend Integration

The chat widget is designed to integrate with Docusaurus. To integrate it into your Docusaurus site:

1. The chat widget component is located at `src/theme/ChatWidget.js`
2. It should be automatically integrated via the Docusaurus theme system
3. Make sure your Docusaurus site can reach the backend API at the configured URL

## Environment Variables

Create a `.env` file in the `backend` directory with the following variables:

- `GOOGLE_API_KEY`: Your Google API key for accessing Gemini
- `QDRANT_URL`: URL to your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant
- `NEON_DB_URL`: Connection string for Neon PostgreSQL database

## API Endpoints

- `GET /`: Health check
- `GET /health`: Health status
- `POST /api/chat/start`: Start a new chat session
- `POST /api/chat/{session_id}/message`: Send a message and get a response
- `GET /api/chat/{session_id}/history`: Get chat history
- `POST /api/ingest`: Ingest documentation content

## Troubleshooting

### Common Issues

1. **API Key Issues**: Ensure your Google API key is valid and has access to the Generative Language API
2. **Database Connection**: Verify that your Neon DB connection string is correct
3. **Qdrant Connection**: Check that your Qdrant URL and API key are correct
4. **Documentation Not Indexed**: Make sure you've run the ingestion script before asking questions

### Testing Connectivity

Run the connectivity test script to verify all services are accessible:

```bash
python test_connectivity.py
```

## Running in Production

For production deployment:
1. Use environment variables for all sensitive configuration
2. Implement proper rate limiting
3. Set up monitoring and logging
4. Use SSL certificates for secure connections