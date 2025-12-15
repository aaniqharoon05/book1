from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime


class TestQuery(BaseModel):
    """Model for a test query submitted for validation"""
    query_id: str
    text: str
    expected_chunks: Optional[List[str]] = None
    metadata: Optional[dict] = None


class RetrievedChunk(BaseModel):
    """Model for a document chunk retrieved from the vector database"""
    chunk_id: str
    text: str
    similarity_score: float
    source_document: str
    metadata: Optional[dict] = None


class ValidationReport(BaseModel):
    """Model for the validation report returned after testing"""
    query_id: str
    query_text: str
    retrieved_chunks: List[RetrievedChunk]
    accuracy_score: float
    total_retrieved: int
    relevant_retrieved: int
    total_expected: Optional[int] = None
    timestamp: datetime = datetime.now()


class PerformanceMetrics(BaseModel):
    """Model for performance metrics collected during validation"""
    query_id: str
    response_time_ms: float
    throughput_qps: Optional[float] = None
    p95_response_time: Optional[float] = None
    p99_response_time: Optional[float] = None
    total_queries: int
    timestamp: datetime = datetime.now()