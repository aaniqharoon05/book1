import pytest
from src.models.validation_models import TestQuery, RetrievedChunk, ValidationReport, PerformanceMetrics


def test_test_query_model():
    """Test the TestQuery model"""
    query = TestQuery(
        query_id="test1",
        text="This is a test query",
        expected_chunks=["expected chunk 1", "expected chunk 2"]
    )
    
    assert query.query_id == "test1"
    assert query.text == "This is a test query"
    assert query.expected_chunks == ["expected chunk 1", "expected chunk 2"]
    assert query.metadata is None


def test_retrieved_chunk_model():
    """Test the RetrievedChunk model"""
    chunk = RetrievedChunk(
        chunk_id="chunk1",
        text="This is a retrieved chunk",
        similarity_score=0.85,
        source_document="doc1.pdf",
        metadata={"page": 1, "section": "introduction"}
    )
    
    assert chunk.chunk_id == "chunk1"
    assert chunk.text == "This is a retrieved chunk"
    assert chunk.similarity_score == 0.85
    assert chunk.source_document == "doc1.pdf"
    assert chunk.metadata == {"page": 1, "section": "introduction"}


def test_validation_report_model():
    """Test the ValidationReport model"""
    # Create a mock RetrievedChunk for testing
    chunk = RetrievedChunk(
        chunk_id="chunk1",
        text="This is a retrieved chunk",
        similarity_score=0.85,
        source_document="doc1.pdf",
        metadata={"page": 1, "section": "introduction"}
    )
    
    report = ValidationReport(
        query_id="test1",
        query_text="This is a test query",
        retrieved_chunks=[chunk],
        accuracy_score=0.9,
        total_retrieved=1,
        relevant_retrieved=1,
        total_expected=2
    )
    
    assert report.query_id == "test1"
    assert report.query_text == "This is a test query"
    assert len(report.retrieved_chunks) == 1
    assert report.accuracy_score == 0.9
    assert report.total_retrieved == 1
    assert report.relevant_retrieved == 1
    assert report.total_expected == 2


def test_performance_metrics_model():
    """Test the PerformanceMetrics model"""
    metrics = PerformanceMetrics(
        query_id="test1",
        response_time_ms=150.0,
        throughput_qps=10.5,
        p95_response_time=200.0,
        p99_response_time=300.0,
        total_queries=100
    )
    
    assert metrics.query_id == "test1"
    assert metrics.response_time_ms == 150.0
    assert metrics.throughput_qps == 10.5
    assert metrics.p95_response_time == 200.0
    assert metrics.p99_response_time == 300.0
    assert metrics.total_queries == 100