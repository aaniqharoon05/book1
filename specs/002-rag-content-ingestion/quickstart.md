# Quickstart: RAG Content Ingestion and Vectorization

## Overview
This guide will help you set up and run the RAG content ingestion pipeline that converts book content into searchable vector embeddings using Cohere and Qdrant.

## Prerequisites

Before getting started, ensure you have:

1. **Python 3.11+** installed on your system
2. **uv** package manager installed (https://github.com/astral-sh/uv)
3. **Cohere API Key** - Get one from https://cohere.ai/
4. **Qdrant Instance** - Either local or cloud-hosted

## Setup

### 1. Clone and Navigate to Project
```bash
cd hackathon1/book
```

### 2. Create Backend Directory Structure
```bash
mkdir -p backend/src
```

### 3. Initialize Project with uv
```bash
cd backend
uv init
```

### 4. Create Environment File
Create a `.env` file in the backend directory with your configuration:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_URL=your_qdrant_url_here  # e.g., http://localhost:6333 or https://your-cluster.us-east.aws.cloud.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_if_using_cloud  # Optional for local instances
QDRANT_PORT=6333  # Default is 6333
CHUNK_SIZE=512  # Default token size for text chunks
SOURCE_URL=https://book1-eight.vercel.app/
SITEMAP_URL = https://book1-eight.vercel.app/sitemap.xml
```

Create an example file as well:
```bash
cp .env .env.example
```

### 5. Install Dependencies
Add the required dependencies to your project:

```bash
uv add cohere qdrant-client python-dotenv beautifulsoup4 requests
uv add --dev python-lsp-server
```

## Environment Configuration

1. Get your Cohere API key from the [Cohere Dashboard](https://dashboard.cohere.ai/api-keys)
2. Set up Qdrant - you can run it locally with Docker:
   ```bash
   docker run -p 6333:6333 qdrant/qdrant
   ```
3. Or use Qdrant Cloud - sign up at [qdrant.tech](https://qdrant.tech/)

## Running the Ingestion Pipeline

### 1. Create the Main Implementation Files
Create the following files in `backend/src/`:

**main.py** - Main ingestion pipeline implementation:
```python
# This will be implemented based on the functional requirements
# The main components will be:
# - get_all_urls: Fetches all URLs from the deployed book
# - extract_text_from_url: Extracts clean text from a given URL
# - chunk_text: Splits text into appropriately sized chunks
# - embed: Generates embeddings using Cohere
# - create_collection: Creates the "chatbot_embedding" collection in Qdrant
# - save_chunk_to_qdrant: Saves embeddings and metadata to Qdrant
# - main function: Orchestrates the entire pipeline
```

**config.py** - Configuration settings:
```python
# Configuration settings for the ingestion pipeline
# Will be populated with settings based on environment variables
```

### 2. Run the Ingestion Pipeline
```bash
cd backend
python src/main.py
```

## Verification

After running the pipeline, verify that:

1. The "chatbot_embedding" collection exists in Qdrant
2. Embeddings have been stored with appropriate metadata
3. You can perform test queries to validate retrieval


## Architecture Overview

The ingestion pipeline follows this process:

1. **Fetch Content**: Retrieve all pages from the deployed book URL 
2. **Parse & Clean**: Extract clean text content while removing navigation elements
3. **Chunk**: Split content into appropriately sized segments
4. **Embed**: Generate vector embeddings using Cohere
5. **Store**: Save embeddings to Qdrant with metadata
6. **Validate**: Test retrieval with sample queries

## Troubleshooting

### Common Issues

1. **API Key Issues**: Ensure your Cohere and Qdrant API keys are correct and have the necessary permissions
2. **Connection Issues**: Verify that your Qdrant instance is accessible
3. **Rate Limits**: Cohere may have rate limits; implement appropriate backoff if needed

### Logs
Check the application logs for detailed information about the ingestion process and any errors that occur.