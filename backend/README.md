# RAG Retrieval Validation API

This API provides validation capabilities for RAG (Retrieval-Augmented Generation) retrieval pipelines. It allows testing the accuracy and performance of the retrieval system by submitting test queries and evaluating the relevance of returned document chunks.

## Features

- Validate retrieval accuracy by submitting test queries and evaluating relevance of returned results
- Performance benchmarking with configurable load testing
- Integration testing to verify all system components
- Comprehensive logging and monitoring capabilities

## Architecture

The system consists of the following components:

- **Models** (`src/models/`): Pydantic models for data validation
- **Services** (`src/services/`): Core business logic in `rag_validation_service.py`
- **API** (`src/api/`): FastAPI endpoints in `endpoints/validation.py`
- **Configuration** (`src/config.py`): Environment configuration

## Installation

1. Clone the repository
2. Navigate to the backend directory
3. Install dependencies:

```bash
pip install -r requirements.txt
```

4. Set up environment variables (see `.env.example`)

## API Endpoints

### Validation Endpoints

- `POST /api/v1/validate` - Submit a test query for validation
- `POST /api/v1/validate/configurable` - Submit a test query with configurable parameters (top-k, similarity threshold)
- `POST /api/v1/benchmark` - Run performance benchmark tests
- `POST /api/v1/benchmark/concurrent` - Run concurrent benchmark tests

### Health Check Endpoints

- `GET /api/v1/health` - Check API health
- `GET /api/v1/health/services` - Check health of external services (Cohere, Qdrant)
- `GET /api/v1/health/integration` - Run comprehensive integration tests

## Usage Examples

### Validate a Query

```bash
curl -X POST "http://localhost:8000/api/v1/validate" \
  -H "Content-Type: application/json" \
  -d '{
    "query_id": "test1",
    "text": "What are the benefits of RAG?",
    "expected_chunks": ["RAG combines retrieval with generation...", "Benefits include..."]
  }'
```

### Run a Benchmark

```bash
curl -X POST "http://localhost:8000/api/v1/benchmark" \
  -H "Content-Type: application/json" \
  -d '[
    {
      "query_id": "test1",
      "text": "What are the benefits of RAG?"
    },
    {
      "query_id": "test2",
      "text": "How does vector search work?"
    }
  ]'
```

## Environment Variables

Create a `.env` file with the following variables:

- `QDRANT_HOST`: Host for Qdrant vector database (default: localhost)
- `QDRANT_PORT`: Port for Qdrant vector database (default: 6333)
- `QDRANT_COLLECTION_NAME`: Name of the collection to search (default: document_chunks)
- `COHERE_API_KEY`: API key for Cohere embedding service
- `LOG_LEVEL`: Logging level (default: INFO)

## Running the Application

```bash
python -m src.api.main
```

Or with uvicorn:

```bash
uvicorn src.api.main:app --host 0.0.0.0 --port 8000 --reload
```

## Running Tests

```bash
pytest tests/
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request