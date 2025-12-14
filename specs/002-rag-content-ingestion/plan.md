
# Implementation Plan: RAG Content Ingestion and Vectorization

**Branch**: `002-rag-content-ingestion` | **Date**: 2025-12-14 | **Spec**: [link](spec.md)
**Input**: Feature specification from `/specs/[002-rag-content-ingestion]/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The RAG Content Ingestion and Vectorization feature implements a backend service that extracts URLs from the sitemap at https://book1-eight.vercel.app/sitemap.xml to discover all book content URLs. The system fetches content from these URLs, cleans and chunks the text, generates embeddings using Cohere, and stores them in Qdrant vector database with metadata. This enables semantic search capabilities for the book content as required by the project's RAG system.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: Cohere client, Qdrant client, python-dotenv, BeautifulSoup4, Requests, Uvicorn
**Storage**: Qdrant vector database
**Testing**: pytest
**Target Platform**: Linux server
**Project Type**: backend
**Performance Goals**: Process at least 100 pages of content per hour with standard configurations, ingest 95% of provided URLs successfully
**Constraints**: Embeddings must use Cohere, vector storage must use Qdrant, API keys must not be exposed in frontend
**Scale/Scope**: Support the book content with approximately 1000+ pages

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Principle 10 (Standardized RAG Technology Stack)**: PASS - Using Cohere for embeddings and Qdrant for vector database as required
- **Principle 12 (Secure and Decoupled Architecture)**: PASS - Backend service will be separate from frontend with no exposed API keys
- **Principle 14 (Robust RAG Error Handling)**: PASS - Implementation will include error handling for failed URL fetches, cleaning, and embedding generation
- **Principle 8 (Spec-Driven RAG Development)**: PASS - Following proper spec → plan → tasks workflow

## Project Structure

### Documentation (this feature)

```text
specs/[002-rag-content-ingestion]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── src/
│   ├── main.py          # Main ingestion pipeline implementation
│   └── config.py        # Configuration settings
├── .env                 # Environment variables
├── .env.example         # Example environment variables
├── pyproject.toml       # Project dependencies and configuration
└── README.md            # Instructions for running the ingestion pipeline
```

**Structure Decision**: Backend service structure chosen to separate the ingestion pipeline from the frontend, supporting the decoupled architecture required by the constitution.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
