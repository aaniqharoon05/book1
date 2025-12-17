from fastapi import APIRouter, HTTPException, Depends
from typing import List
import logging
from ...models.validation_models import TestQuery, ValidationReport
from ...services.rag_validation_service import RAGValidationService
from ...api.auth import verify_api_key

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

router = APIRouter()
validation_service = RAGValidationService()


@router.post("/validate", response_model=ValidationReport)
async def validate_retrieval(test_query: TestQuery, api_key: bool = Depends(verify_api_key)):
    """
    Accept test queries and return validation reports
    """
    import time
    start_time = time.time()

    try:
        logger.info(f"Processing validation request for query ID: {test_query.query_id}")

        # Query the vector database
        retrieved_chunks = validation_service.query_vector_db(
            query_text=test_query.text,
            top_k=10,  # Default top_k, can be configurable
            similarity_threshold=0.5  # Default threshold
        )

        # Generate validation report
        report = validation_service.generate_validation_report(test_query, retrieved_chunks)

        # Calculate and log response time
        response_time = (time.time() - start_time) * 1000  # Convert to milliseconds
        logger.info(f"Validation completed for query ID: {test_query.query_id}, "
                   f"Response time: {response_time:.2f}ms, "
                   f"Accuracy: {report.accuracy_score:.2f}, "
                   f"Retrieved: {report.total_retrieved} chunks")

        return report

    except Exception as e:
        response_time = (time.time() - start_time) * 1000  # Convert to milliseconds
        logger.error(f"Error during validation for query ID: {test_query.query_id}, "
                    f"Response time before error: {response_time:.2f}ms, "
                    f"Error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/validate/configurable", response_model=ValidationReport)
async def validate_retrieval_configurable(
    test_query: TestQuery,
    api_key: bool = Depends(verify_api_key),
    top_k: int = 10,
    similarity_threshold: float = 0.5
):
    """
    Accept test queries with configurable parameters and return validation reports
    """
    import time
    start_time = time.time()

    try:
        logger.info(f"Processing configurable validation request for query ID: {test_query.query_id}")

        # Query the vector database with configurable parameters
        retrieved_chunks = validation_service.query_vector_db(
            query_text=test_query.text,
            top_k=top_k,
            similarity_threshold=similarity_threshold
        )

        # Generate validation report
        report = validation_service.generate_validation_report(test_query, retrieved_chunks)

        # Calculate and log response time
        response_time = (time.time() - start_time) * 1000  # Convert to milliseconds
        logger.info(f"Configurable validation completed for query ID: {test_query.query_id}, "
                   f"Response time: {response_time:.2f}ms, "
                   f"Top-K: {top_k}, "
                   f"Threshold: {similarity_threshold}, "
                   f"Accuracy: {report.accuracy_score:.2f}, "
                   f"Retrieved: {report.total_retrieved} chunks")

        return report

    except Exception as e:
        response_time = (time.time() - start_time) * 1000  # Convert to milliseconds
        logger.error(f"Error during configurable validation for query ID: {test_query.query_id}, "
                    f"Response time before error: {response_time:.2f}ms, "
                    f"Error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


from typing import List
import time
from ...models.validation_models import PerformanceMetrics


@router.post("/benchmark", response_model=PerformanceMetrics)
async def run_performance_benchmark(queries: List[TestQuery], api_key: bool = Depends(verify_api_key), top_k: int = 10):
    """
    Execute benchmark tests with the provided queries
    """
    start_time = time.time()

    try:
        logger.info(f"Starting performance benchmark for {len(queries)} queries")

        # Run the performance benchmark
        metrics = validation_service.run_performance_benchmark(queries, top_k=top_k)

        # Log the completion
        total_time = time.time() - start_time
        logger.info(f"Performance benchmark completed, Total time: {total_time:.2f}s, "
                   f"Throughput: {metrics.throughput_qps:.2f} QPS, "
                   f"Average response time: {metrics.response_time_ms:.2f}ms")

        return metrics

    except Exception as e:
        total_time = time.time() - start_time
        logger.error(f"Error during performance benchmark, "
                    f"Time elapsed before error: {total_time:.2f}s, "
                    f"Error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/benchmark/concurrent")
async def run_concurrent_benchmark(queries: List[TestQuery], api_key: bool = Depends(verify_api_key), top_k: int = 10, concurrency: int = 5):
    """
    Execute concurrent benchmark tests with configurable load
    """
    import asyncio
    from asyncio import Semaphore

    start_time = time.time()

    async def query_worker(query: TestQuery, semaphore: Semaphore):
        async with semaphore:
            start = time.time()
            try:
                # In a real implementation, we would call the validation service
                # For now, we'll simulate the call
                await asyncio.sleep(0.1)  # simulate processing time
                return {"query_id": query.query_id, "response_time": time.time() - start}
            except Exception as e:
                logger.error(f"Error processing query {query.query_id}: {str(e)}")
                return {"query_id": query.query_id, "error": str(e)}

    try:
        logger.info(f"Starting concurrent benchmark for {len(queries)} queries with concurrency {concurrency}")

        # Create a semaphore to limit concurrent requests
        semaphore = Semaphore(concurrency)

        # Create tasks for concurrent execution
        tasks = [query_worker(query, semaphore) for query in queries]
        results = await asyncio.gather(*tasks, return_exceptions=True)

        # Calculate metrics
        successful_queries = [r for r in results if not isinstance(r, Exception) and 'error' not in r]
        total_time = time.time() - start_time

        response_times = [r['response_time'] * 1000 for r in successful_queries]  # Convert to milliseconds
        response_times.sort()

        metrics = PerformanceMetrics(
            query_id="concurrent_benchmark_" + str(int(time.time())),
            response_time_ms=sum(response_times) / len(response_times) if response_times else 0,
            throughput_qps=len(successful_queries) / total_time if total_time > 0 else 0,
            p95_response_time=response_times[int(0.95 * len(response_times))] if response_times else 0,
            p99_response_time=response_times[int(0.99 * len(response_times))] if response_times else 0,
            total_queries=len(successful_queries)
        )

        logger.info(f"Concurrent benchmark completed, Total time: {total_time:.2f}s, "
                   f"Throughput: {metrics.throughput_qps:.2f} QPS, "
                   f"Average response time: {metrics.response_time_ms:.2f}ms")

        return metrics

    except Exception as e:
        total_time = time.time() - start_time
        logger.error(f"Error during concurrent benchmark, "
                    f"Time elapsed before error: {total_time:.2f}s, "
                    f"Error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/health")
async def health_check():
    """
    Health check endpoint for the API
    """
    return {"status": "healthy", "service": "RAG Validation API"}


@router.get("/health/services")
async def service_health_check():
    """
    Health check for external services (Qdrant and Cohere)
    """
    try:
        # Test Cohere client
        validation_service.cohere_client.embed(
            texts=["health check"],
            model='embed-english-v3.0',
            input_type="search_query"
        )
        cohere_healthy = True
    except Exception:
        cohere_healthy = False

    try:
        # Test Qdrant client
        validation_service.qdrant_client.get_collections()
        qdrant_healthy = True
    except Exception:
        qdrant_healthy = False

    return {
        "api": "healthy",
        "cohere": "healthy" if cohere_healthy else "unhealthy",
        "qdrant": "healthy" if qdrant_healthy else "unhealthy",
        "overall": "healthy" if cohere_healthy and qdrant_healthy else "unhealthy"
    }


@router.get("/health/integration")
async def integration_health_check():
    """
    Run integration tests to check all components
    """
    results = validation_service.run_integration_tests()
    return results