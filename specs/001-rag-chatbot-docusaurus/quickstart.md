# Quickstart Guide: RAG Chatbot for Docusaurus

**Feature**: 001-rag-chatbot-docusaurus  
**Date**: 2025-12-21

## Overview

This guide will help you set up and run the RAG Chatbot for Docusaurus documentation sites. The system consists of a backend API built with FastAPI and a frontend chat widget component for Docusaurus.

## Prerequisites

- Python 3.11+
- Node.js 18+ and npm/yarn
- Qdrant vector database (local or cloud)
- Neon PostgreSQL database
- Google API key for Gemini 2.0 Flash

## Backend Setup

### 1. Clone and Navigate to Backend Directory

```bash
cd backend
```

### 2. Create Virtual Environment and Install Dependencies

```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt
```

### 3. Set Up Environment Variables

Create a `.env` file in the backend directory with the following:

```env
GEMINI_API_KEY=your_gemini_api_key_here
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DB_URL=your_neon_db_connection_string
QDRANT_HOST=localhost
QDRANT_PORT=6333
```

### 4. Run the Backend Server

```bash
python main.py
```

The backend API will be available at `http://localhost:8000`.

## Frontend Setup

### 1. Navigate to Frontend Directory

```bash
cd frontend
```

### 2. Install Dependencies

```bash
npm install
# or
yarn install
```

### 3. Configure Docusaurus

Update your `docusaurus.config.js` to include the chat widget:

```js
// docusaurus.config.js
module.exports = {
  // ... other config
  themes: [
    // ... other themes
    [require.resolve('./src/components/ChatWidget'), {
      backendUrl: 'http://localhost:8000'
    }]
  ],
};
```

### 4. Run the Docusaurus Development Server

```bash
npm run start
# or
yarn start
```

## Ingesting Documentation

Before users can ask questions, you need to index your documentation content:

1. Prepare your documentation in the `/docs` folder of your Docusaurus site
2. Run the ingestion script:

```bash
cd backend
python ingest.py --source-url https://your-docs-url.com
```

This will crawl your documentation site, chunk the content, generate embeddings, and store them in Qdrant.

## Testing the API

Once the backend is running, you can test the endpoints:

1. Start a new chat session:
```bash
curl -X POST http://localhost:8000/api/chat/start \
  -H "Content-Type: application/json" \
  -d '{"user_id": "test-user"}'
```

2. Send a message:
```bash
curl -X POST http://localhost:8000/api/chat/{session_id}/message \
  -H "Content-Type: application/json" \
  -d '{"content": "What is the main purpose of this documentation?"}'
```

## Integration with Docusaurus

To integrate the chat widget into your Docusaurus site:

1. The chat widget is implemented as a React component in `frontend/src/components/ChatWidget`
2. It appears as a floating button in the bottom-right corner of the screen
3. When clicked, it expands to show the chat interface
4. The component handles communication with the backend API

## Troubleshooting

### Common Issues

1. **API Key Issues**: Ensure your GEMINI_API_KEY is correctly set in the environment variables.
2. **Database Connection**: Verify that your Qdrant and Neon DB connection strings are correct.
3. **CORS Issues**: Make sure your backend allows requests from your frontend domain.

### Checking System Status

- Backend health check: `GET /health`
- Qdrant connection: Check if the vector database is accessible
- Database connection: Verify Neon DB connectivity

## Next Steps

1. Customize the chat widget UI to match your documentation site's theme
2. Implement user authentication if needed
3. Set up monitoring and logging for production use
4. Add rate limiting to prevent API abuse