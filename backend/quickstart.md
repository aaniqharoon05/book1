# Quickstart Guide: RAG Retrieval Validation API

This guide will help you get started with the RAG Retrieval Validation API quickly.

## Prerequisites

- Python 3.11 or higher
- Qdrant vector database (running locally or accessible via network)
- Cohere API key

## Setup

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd <repository-directory>
   ```

2. **Navigate to the backend directory**
   ```bash
   cd backend
   ```

3. **Set up virtual environment (optional but recommended)**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

4. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

5. **Configure environment variables**
   Create a `.env` file based on `.env.example`:
   ```bash
   cp .env.example .env
   ```
   
   Edit the `.env` file and add your Cohere API key and other configuration.

## Running the API

Start the API server:
```bash
uvicorn src.api.main:app --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`.

## Basic Usage

### 1. Validate a Query

Test the retrieval accuracy with a simple query:

```bash
curl -X POST "http://localhost:8000/api/v1/validate" \
  -H "Content-Type: application/json" \
  -d '{
    "query_id": "quick-test-1",
    "text": "What are the benefits of RAG systems?",
    "expected_chunks": ["RAG combines retrieval and generation...", "Benefits include..."]
  }'
```

### 2. Run a Performance Benchmark

Test the performance with multiple queries:

```bash
curl -X POST "http://localhost:8000/api/v1/benchmark" \
  -H "Content-Type: application/json" \
  -d '[
    {
      "query_id": "perf-test-1",
      "text": "What are the benefits of RAG systems?"
    },
    {
      "query_id": "perf-test-2",
      "text": "How does vector search work?"
    }
  ]'
```

### 3. Check System Health

Verify that all services are running:

```bash
curl "http://localhost:8000/api/v1/health/services"
```

## API Endpoints Overview

- `POST /api/v1/validate` - Validate a single query
- `POST /api/v1/validate/configurable` - Validate with custom parameters (top-k, threshold)
- `POST /api/v1/benchmark` - Run performance benchmarks
- `POST /api/v1/benchmark/concurrent` - Run concurrent load tests
- `GET /api/v1/health` - Basic health check
- `GET /api/v1/health/services` - Check external services
- `GET /api/v1/health/integration` - Run integration tests

## Next Steps

1. Add your own test datasets to `backend/tests/test_dataset.json`
2. Customize the validation parameters based on your requirements
3. Set up monitoring for production deployment
4. Refer to the full documentation in the README.md for more detailed information