import os
import logging
import time
from typing import List, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
import cohere
from ..models.validation_models import TestQuery, RetrievedChunk, ValidationReport, PerformanceMetrics
from ..config import (
    QDRANT_HOST,
    QDRANT_PORT,
    QDRANT_URL,
    QDRANT_API_KEY,
    QDRANT_COLLECTION_NAME,
    COHERE_API_KEY,
)



class RAGValidationService:
    """
    Service class for validating RAG retrieval pipeline accuracy and performance
    """

    def __init__(self):
    # Set up logging FIRST
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)

        # Initialize Qdrant client (cloud-first, fallback to local)
        if QDRANT_URL:
            self.logger.info("Using Qdrant Cloud")
            self.qdrant_client = QdrantClient(
                url=QDRANT_URL,
                api_key=QDRANT_API_KEY,
            )
        else:
            self.logger.info("Using local Qdrant")
            self.qdrant_client = QdrantClient(
                host=QDRANT_HOST,
                port=QDRANT_PORT,
            )

        # Initialize Cohere client for embeddings
        self.cohere_client = cohere.Client(COHERE_API_KEY)

        # Collection name for document chunks
        self.collection_name = QDRANT_COLLECTION_NAME

    def query_vector_db(self, query_text: str, top_k: int = 10, similarity_threshold: float = 0.5) -> List[RetrievedChunk]:
        self.logger.info(f"Starting vector database query for: '{query_text[:50]}...'")
        query_start_time = time.time()

        try:
            response = self.cohere_client.embed(
                texts=[query_text],
                model="embed-english-v3.0",
                input_type="search_query",
            )
            query_embedding = response.embeddings[0]

            search_results = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                # score_threshold=similarity_threshold,
            )

            points = search_results.points
            self.logger.debug(f"Retrieved {len(points)} results from vector database")

            retrieved_chunks = []
            for i, result in enumerate(points):
                retrieved_chunks.append(
                    RetrievedChunk(
                        chunk_id=result.id,
                        text=result.payload.get("text", ""),
                        similarity_score=result.score,
                        source_document=result.payload.get("source", ""),
                        metadata=result.payload,
                    )
                )
                self.logger.debug(f"Retrieved {len(points)} points from Qdrant")
            self.logger.info(
                f"Vector database query completed in {time.time() - query_start_time:.3f}s"
            )
            return retrieved_chunks

        except Exception as e:
            self.logger.error(f"Error querying vector database: {str(e)}")
            raise


    def calculate_relevance_score(self, query_text: str, chunk_text: str) -> float:
        """
        Calculate relevance score between query and chunk using embedding similarity
        """
        self.logger.debug(f"Calculating relevance score between query and chunk (first 30 chars: '{chunk_text[:30]}...')")
        calc_start_time = time.time()

        try:
            # Generate embeddings for both query and chunk
            response = self.cohere_client.embed(
                texts=[query_text, chunk_text],
                model='embed-english-v3.0',
                input_type="search_document"
            )

            query_embedding = response.embeddings[0]
            chunk_embedding = response.embeddings[1]

            # Calculate cosine similarity
            import numpy as np
            query_norm = np.linalg.norm(query_embedding)
            chunk_norm = np.linalg.norm(chunk_embedding)

            if query_norm == 0 or chunk_norm == 0:
                calc_time = time.time() - calc_start_time
                self.logger.warning(f"Zero norm detected in relevance calculation, returning 0.0 after {calc_time:.3f}s")
                return 0.0

            similarity = np.dot(query_embedding, chunk_embedding) / (query_norm * chunk_norm)
            calc_time = time.time() - calc_start_time
            self.logger.debug(f"Relevance score calculated as {similarity:.3f} in {calc_time:.3f}s")
            return float(similarity)

        except Exception as e:
            calc_time = time.time() - calc_start_time
            self.logger.error(f"Error calculating relevance score after {calc_time:.3f}s: {str(e)}")
            raise

    def generate_validation_report(self, test_query: TestQuery, retrieved_chunks: List[RetrievedChunk]) -> ValidationReport:
        """
        Generate a validation report for the test query and retrieved chunks
        """
        self.logger.info(f"Generating validation report for query ID: {test_query.query_id}")
        report_start_time = time.time()

        try:
            relevant_retrieved = 0

            # If expected chunks are provided, calculate relevance against them
            if test_query.expected_chunks:
                self.logger.debug(f"Validating against {len(test_query.expected_chunks)} expected chunks")
                for i, chunk in enumerate(retrieved_chunks):
                    is_relevant = False
                    for expected in test_query.expected_chunks:
                        relevance = self.calculate_relevance_score(test_query.text, expected)
                        if relevance > 0.7:  # Threshold for considering chunk relevant
                            relevant_retrieved += 1
                            is_relevant = True
                            self.logger.debug(f"Retrieved chunk {i} is relevant to an expected chunk (relevance: {relevance:.3f})")
                            break
                    if not is_relevant:
                        self.logger.debug(f"Retrieved chunk {i} is not relevant to any expected chunks")
            else:
                self.logger.debug("No expected chunks provided, skipping relevance validation")

            accuracy_score = (
                relevant_retrieved / len(retrieved_chunks)
                if retrieved_chunks
                else 0.0
            )

            report = ValidationReport(
                query_id=test_query.query_id,
                query_text=test_query.text,
                retrieved_chunks=retrieved_chunks,
                accuracy_score=accuracy_score,
                total_retrieved=len(retrieved_chunks),
                relevant_retrieved=relevant_retrieved,
                total_expected=len(test_query.expected_chunks) if test_query.expected_chunks else None
            )

            report_time = time.time() - report_start_time
            self.logger.info(f"Validation report generated in {report_time:.3f}s for query ID: {test_query.query_id}")
            return report

        except Exception as e:
            report_time = time.time() - report_start_time
            self.logger.error(f"Error generating validation report after {report_time:.3f}s: {str(e)}")
            raise

    def run_performance_benchmark(self, queries: List[TestQuery], top_k: int = 10) -> PerformanceMetrics:
        """
        Run performance benchmark on a set of queries
        """
        import time
        start_time = time.time()

        total_queries = len(queries)
        response_times = []

        for query in queries:
            query_start = time.time()
            try:
                retrieved_chunks = self.query_vector_db(query.text, top_k=top_k)
                query_time = time.time() - query_start
                response_times.append(query_time * 1000)  # Convert to milliseconds
            except Exception as e:
                self.logger.error(f"Error running benchmark for query {query.query_id}: {str(e)}")
                continue

        if not response_times:
            raise ValueError("No successful queries to benchmark")

        # Calculate performance metrics
        response_times.sort()
        total_time = time.time() - start_time

        metrics = PerformanceMetrics(
            query_id="benchmark_" + str(int(time.time())),
            response_time_ms=sum(response_times) / len(response_times),  # Average response time
            throughput_qps=len(response_times) / total_time if total_time > 0 else 0,
            p95_response_time=response_times[int(0.95 * len(response_times))] if response_times else 0,
            p99_response_time=response_times[int(0.99 * len(response_times))] if response_times else 0,
            total_queries=len(response_times)
        )

        return metrics

    def run_integration_tests(self) -> dict:
        """
        Run comprehensive integration tests for all components
        """
        import time
        start_time = time.time()

        results = {
            "timestamp": time.time(),
            "tests_passed": 0,
            "tests_failed": 0,
            "total_tests": 0,
            "components_tested": [],
            "failures": [],
            "details": {}
        }

        # Test 1: Cohere client connection
        try:
            # Test embedding generation
            test_text = "test integration"
            response = self.cohere_client.embed(
                texts=[test_text],
                model='embed-english-v3.0',
                input_type="search_query"
            )
            cohere_ok = len(response.embeddings) > 0
            results["details"]["cohere"] = {"status": "OK" if cohere_ok else "FAILED"}
            results["components_tested"].append("cohere")
            results["total_tests"] += 1
            if cohere_ok:
                results["tests_passed"] += 1
            else:
                results["tests_failed"] += 1
                results["failures"].append("Cohere embedding service")
        except Exception as e:
            self.logger.error(f"Integration test failed for Cohere: {str(e)}")
            results["details"]["cohere"] = {"status": "FAILED", "error": str(e)}
            results["components_tested"].append("cohere")
            results["tests_failed"] += 1
            results["total_tests"] += 1
            results["failures"].append("Cohere embedding service")

        # Test 2: Qdrant client connection
        try:
            # Test collection existence check
            collections = self.qdrant_client.get_collections()
            qdrant_ok = True  # Just checking if we can connect and get collections
            results["details"]["qdrant"] = {"status": "OK" if qdrant_ok else "FAILED"}
            results["components_tested"].append("qdrant")
            results["total_tests"] += 1
            if qdrant_ok:
                results["tests_passed"] += 1
            else:
                results["tests_failed"] += 1
                results["failures"].append("Qdrant vector database")
        except Exception as e:
            self.logger.error(f"Integration test failed for Qdrant: {str(e)}")
            results["details"]["qdrant"] = {"status": "FAILED", "error": str(e)}
            results["components_tested"].append("qdrant")
            results["tests_failed"] += 1
            results["total_tests"] += 1
            results["failures"].append("Qdrant vector database")

        # Test 3: Query processing flow
        try:
            # Test the entire flow with a simple query
            test_chunks = self.query_vector_db("test query for integration")
            flow_ok = True  # If we got here without exception, the flow works
            results["details"]["query_flow"] = {"status": "OK", "retrieved_chunks": len(test_chunks)}
            results["components_tested"].append("query_flow")
            results["total_tests"] += 1
            results["tests_passed"] += 1
        except Exception as e:
            self.logger.error(f"Integration test failed for query flow: {str(e)}")
            results["details"]["query_flow"] = {"status": "FAILED", "error": str(e)}
            results["components_tested"].append("query_flow")
            results["tests_failed"] += 1
            results["total_tests"] += 1
            results["failures"].append("Query processing flow")

        results["execution_time"] = time.time() - start_time
        return results

    def generate_integration_test_report(self) -> dict:
        """
        Generate a detailed integration test report with specific failure point identification
        """
        import time
        start_time = time.time()

        # Run the integration tests
        integration_results = self.run_integration_tests()

        # Generate detailed report
        report = {
            "report_id": f"integration_report_{int(time.time())}",
            "timestamp": time.time(),
            "summary": {
                "total_tests": integration_results["total_tests"],
                "tests_passed": integration_results["tests_passed"],
                "tests_failed": integration_results["tests_failed"],
                "success_rate": integration_results["tests_passed"] / integration_results["total_tests"] if integration_results["total_tests"] > 0 else 0,
                "execution_time": integration_results["execution_time"]
            },
            "components": integration_results["details"],
            "failures": {
                "count": len(integration_results["failures"]),
                "list": integration_results["failures"]
            },
            "recommendations": []
        }

        # Add recommendations based on failures
        if integration_results["failures"]:
            for failure in integration_results["failures"]:
                if "Cohere" in failure:
                    report["recommendations"].append("Verify COHERE_API_KEY is correctly set in environment")
                elif "Qdrant" in failure:
                    report["recommendations"].append(
                        "Verify QDRANT_URL, QDRANT_API_KEY, and collection existence in Qdrant Cloud")
                elif "Query processing" in failure:
                    report["recommendations"].append("Verify Qdrant collection exists and contains data")

        # Add overall status
        if integration_results["tests_failed"] == 0:
            report["overall_status"] = "PASS"
            report["message"] = "All integration tests passed successfully"
        else:
            report["overall_status"] = "FAIL"
            report["message"] = f"{integration_results['tests_failed']} out of {integration_results['total_tests']} tests failed"

        report["report_generation_time"] = time.time() - start_time
        self.logger.info(f"Integration test report generated in {report['report_generation_time']:.3f}s")
        return report
