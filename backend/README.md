# RAG Content Ingestion Pipeline

This project implements a backend service that extracts content from book URLs, processes the text, generates embeddings using Cohere, and stores them in Qdrant vector database with metadata. This enables semantic search capabilities for the book content as required by the project's RAG system.

## Setup

1. Clone the repository
2. Create a virtual environment (optional but recommended):
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```
3. Install dependencies:
   ```bash
   uv sync  # or pip install -e .
   ```
4. Copy the `.env.example` file to `.env` and update with your configuration:
   ```bash
   cp .env.example .env
   ```
5. Update the `.env` file with your Cohere API key and Qdrant configuration

## Usage

Run the ingestion pipeline:
```bash
cd backend
python src/main.py
```

## Configuration

The application uses the following environment variables:

- `COHERE_API_KEY`: Your Cohere API key
- `COHERE_EMBED_MODEL`: Cohere embedding model to use (default "embed-english-v3.0")
- `QDRANT_URL`: URL of your Qdrant instance
- `QDRANT_API_KEY`: API key for Qdrant (if using cloud)
- `QDRANT_PORT`: Port for Qdrant (default 6333)
- `CHUNK_SIZE`: Size of text chunks in tokens (default 512)
- `CHUNK_OVERLAP`: Number of characters to overlap between chunks (default 20)
- `SOURCE_URL`: Base URL for the book content (default https://book1-eight.vercel.app/)

## Architecture Overview

The ingestion pipeline follows this process:

1. **Fetch Content**: Retrieve all pages from the sitemap at https://book1-eight.vercel.app/sitemap.xml
2. **Parse & Clean**: Extract clean text content while removing navigation elements
3. **Chunk**: Split content into appropriately sized segments
4. **Embed**: Generate vector embeddings using Cohere
5. **Store**: Save embeddings to Qdrant with metadata
6. **Validate**: Test retrieval with sample queries