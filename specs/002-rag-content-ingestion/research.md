# Research: RAG Content Ingestion and Vectorization

## Overview
This research document addresses the technical requirements for implementing a RAG content ingestion pipeline that fetches content from deployed URLs, processes it, and stores vector embeddings in Qdrant using Cohere.

## Decision: Technology Stack
**Rationale**: Selected based on project constitution requirements and industry standards for RAG systems.
- **Backend Language**: Python 3.11 (matches project standards and has excellent ecosystem for NLP)
- **Embeddings**: Cohere (as required by constitution)
- **Vector Database**: Qdrant (as required by constitution)
- **Package Manager**: uv (fast, reliable Python package management)

## Decision: Project Structure
**Rationale**: Following a backend service structure to ensure separation from frontend as required by the constitution.
- **Location**: `/backend` directory to separate from frontend
- **Main File**: `main.py` containing the complete ingestion pipeline
- **Configuration**: Using environment variables via `.env` file

## Decision: Sitemap URL for Content Discovery
**Rationale**: Use the sitemap to systematically discover all book content URLs.
- **Sitemap URL**: `https://book1-eight.vercel.app/sitemap.xml` (as provided by user)
- **Content URLs**: Extracted from sitemap and fetched from `https://book1-eight.vercel.app/`

## Decision: Text Processing Approach
**Rationale**: Using BeautifulSoup4 for HTML parsing and text extraction to handle various HTML structures cleanly.
- **Library**: BeautifulSoup4
- **Purpose**: Extract clean text content from HTML documents
- **Features**: Remove navigation, headers, footers, and other non-content elements

## Decision: Embedding Generation and Storage Process
**Rationale**: Implement the following pipeline to meet functional requirements:
1. Fetch all URLs from the deployed site
2. Extract text content from each URL
3. Chunk the text into appropriate sizes (default 512 tokens)
4. Generate embeddings using Cohere
5. Store embeddings in Qdrant with metadata
6. Create a collection named "chatbot_embedding"
7. Save chunks to Qdrant with metadata

## Decision: Error Handling Strategy
**Rationale**: Implement comprehensive error handling to meet constitution requirements for robust RAG systems.
- Handle failed URL fetches
- Manage malformed HTML content
- Handle Cohere API errors
- Handle Qdrant connection issues
- Log all errors for monitoring

## Decision: Configuration Management
**Rationale**: Using environment variables to securely manage API keys and configuration values.
- Store Cohere API key securely
- Store Qdrant connection details
- Allow configuration of chunk size and other parameters
- Create .env.example for documentation

## Alternatives Considered

### Alternative 1: Different Embedding Services
- **Option**: OpenAI embeddings, Hugging Face, Sentence Transformers
- **Rejected**: Constitution requires Cohere embeddings

### Alternative 2: Different Vector Databases
- **Option**: Pinecone, Weaviate, Milvus
- **Rejected**: Constitution requires Qdrant

### Alternative 3: Frontend-Integrated Pipeline
- **Option**: Run ingestion directly in frontend
- **Rejected**: Constitution requires secure and decoupled architecture; API keys would be exposed

### Alternative 4: Different Chunking Strategy
- **Option**: Fixed character length vs token-based chunking
- **Chosen**: Token-based chunking (default 512 tokens) to align with LLM context windows
- **Tool**: Use tiktoken or similar for accurate tokenization

## Implementation Approach Summary

Based on the research and requirements, the implementation will follow these steps:

1. **Fetch All URLs**: Create a function to get all URLs from the deployed book site
2. **Extract Text**: Use BeautifulSoup to clean text from each URL
3. **Chunk Text**: Split content into chunks of appropriate size (default 512 tokens)
4. **Embed Content**: Generate embeddings using Cohere API
5. **Save to Qdrant**: Create "chatbot_embedding" collection and store embeddings with metadata
6. **Validation**: Test retrieval with sample queries
7. **Logging**: Log process results for monitoring

This approach ensures compliance with project constitution while implementing a robust, scalable RAG content ingestion pipeline.