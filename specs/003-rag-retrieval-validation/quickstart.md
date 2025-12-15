# Quickstart Guide: RAG Retrieval Pipeline Validation

## Overview
This guide provides instructions for setting up and using the RAG retrieval pipeline validation system. This system allows backend engineers to validate the accuracy and performance of the RAG retrieval pipeline.

## Prerequisites
- Python 3.11+
- Access to the Qdrant vector database
- Access to the Cohere embedding model API
- The knowledge base already indexed in the vector database

## Setup

### 1. Clone the Repository
```bash
git clone <repository-url>
cd book/backend
```

### 2. Install Dependencies
```bash
pip install -r requirements.txt
```

### 3. Configure Environment Variables
Create a `.env` file with the following variables:
```env
QDRANT_URL=<qdrant-server-url>
QDRANT_API_KEY=<qdrant-api-key>
COHERE_API_KEY=<cohere-api-key>
VALIDATION_API_KEY=<validation-api-key>  # Optional, but recommended for production
```

### 4. Run the Service
```bash
python -m src.api.main
```
The API will be available at `http://localhost:8000`

## Basic Usage

### 1. Submit a Test Query
```bash
curl -X POST http://localhost:8000/validation/test-query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <validation-api-key>" \
  -d '{
    "query_text": "What is the main concept of vector embeddings?",
    "expected_chunks": ["Vector embeddings represent data in high-dimensional space"],
    "test_scenario": "Basic concept retrieval"
  }'
```

### 2. Create Test Queries
```bash
curl -X POST http://localhost:8000/validation/test-queries \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <validation-api-key>" \
  -d '{
    "query_text": "Explain how the RAG system retrieves information",
    "expected_chunks": ["RAG retrieves relevant information from the knowledge base"],
    "test_scenario": "System functionality"
  }'
```

### 3. List Available Test Queries
```bash
curl -X GET http://localhost:8000/validation/test-queries \
  -H "Authorization: Bearer <validation-api-key>"
```

### 4. Run Performance Benchmark
```bash
curl -X POST http://localhost:8000/validation/run-benchmark \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <validation-api-key>" \
  -d '{
    "queries": [
      {
        "query_text": "What are vector databases?",
        "test_scenario": "Basic concept"
      }
    ],
    "concurrent_users": 5,
    "iterations": 10
  }'
```

### 5. Run Integration Tests
Check the health of all system components:
```bash
curl -X GET http://localhost:8000/validation/integration-test \
  -H "Authorization: Bearer <validation-api-key>"
```

### 6. Get System Metrics
Monitor system performance:
```bash
curl -X GET http://localhost:8000/validation/metrics \
  -H "Authorization: Bearer <validation-api-key>"
```

### 7. Health Check (No Authentication Required)
Check if the service is running:
```bash
curl -X GET http://localhost:8000/validation/health
```

## Validation Results
The validation API returns reports with:
- `validation_status`: Either "pass", "fail", or "partial"
- `accuracy_metrics`: Including relevance percentages
- `performance_metrics`: Response times and throughput
- `issues_found`: Any problems encountered during validation

## Configuration Options
You can customize validation parameters:
- `top_k`: Number of top results to retrieve (default: 5)
- `similarity_threshold`: Minimum similarity threshold (default: 0.7)

Example with parameters:
```bash
curl -X POST http://localhost:8000/validation/test-query \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer <validation-api-key>" \
  -d '{
    "query_text": "How does the system work?",
    "parameters": {
      "top_k": 10,
      "similarity_threshold": 0.8
    }
  }'
```

## Security
The validation endpoints require authentication using an API key. Set the `VALIDATION_API_KEY` environment variable and include it in requests as a Bearer token in the Authorization header. The health check endpoint does not require authentication and can be used for monitoring tools.