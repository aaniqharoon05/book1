# Data Model: RAG Retrieval Pipeline Validation

## Entities

### TestQuery
**Description**: Input provided by engineers for validation, consisting of a text string to be matched against stored embeddings
**Fields**:
- `id`: Unique identifier for the test query
- `query_text`: The actual text of the query to be submitted to the RAG system
- `expected_chunks`: List of expected document chunks that should be retrieved
- `test_scenario`: Description of the testing scenario this query represents
- `parameters`: Optional retrieval parameters (top-k, similarity threshold, etc.)
- `created_at`: Timestamp when the test query was created

### RetrievedChunk
**Description**: Document segment returned by the retrieval system, containing text content and relevance score
**Fields**:
- `id`: Unique identifier for the retrieved chunk
- `content`: The actual text content of the document chunk
- `metadata`: Additional metadata associated with the chunk (source document, page number, etc.)
- `relevance_score`: Numerical value indicating the semantic similarity to the query
- `position_in_results`: The rank position of this chunk in the retrieval results
- `vector_id`: Identifier for the vector representation in the database

### RelevanceScore
**Description**: Numerical value indicating the semantic similarity between the query and retrieved chunk
**Fields**:
- `score`: The similarity score (typically between 0 and 1)
- `method`: The method used to calculate the relevance score
- `query_id`: Reference to the test query this score is associated with
- `chunk_id`: Reference to the retrieved chunk this score is associated with

### PerformanceMetric
**Description**: Quantitative measurement of system performance including response time and throughput
**Fields**:
- `query_id`: Reference to the test query that was measured
- `response_time_ms`: Time taken to retrieve results in milliseconds
- `throughput_qps`: Queries per second processed (if testing multiple queries)
- `concurrent_queries`: Number of concurrent queries during the test
- `timestamp`: When the metric was recorded
- `test_environment`: Environment where the test was run (dev/staging/prod)

### ValidationReport
**Description**: Summary of test results including accuracy metrics and system health indicators
**Fields**:
- `id`: Unique identifier for the validation report
- `test_query_id`: Reference to the test query
- `retrieved_chunks`: List of all chunks retrieved during the test
- `accuracy_metrics`: Detailed metrics on retrieval accuracy
- `performance_metrics`: Performance data collected during the test
- `validation_status`: Pass/fail status of the validation test
- `issues_found`: List of any issues identified during validation
- `engineer_notes`: Any additional notes from the validating engineer
- `created_at`: Timestamp when the report was generated

## Relationships
- A `TestQuery` can be associated with multiple `RetrievedChunk` objects through retrieval operations
- A `RelevanceScore` connects a specific `TestQuery` to a specific `RetrievedChunk`
- A `ValidationReport` contains one `TestQuery` and multiple `RetrievedChunk` objects
- `PerformanceMetric` objects are associated with specific `TestQuery` executions

## Validation Rules
- `TestQuery.query_text` must not be empty
- `RetrievedChunk.relevance_score` must be between 0 and 1
- `ValidationReport.validation_status` must be either "pass", "fail", or "partial"
- `PerformanceMetric.response_time_ms` must be a positive number