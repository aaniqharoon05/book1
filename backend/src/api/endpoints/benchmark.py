from fastapi import APIRouter, HTTPException
from typing import List
import logging
import time
from ...models.validation_models import TestQuery, PerformanceMetrics
from ...services.rag_validation_service import RAGValidationService

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

router = APIRouter()
validation_service = RAGValidationService()


@router.post("/benchmark", response_model=PerformanceMetrics)
async def run_performance_benchmark(queries: List[TestQuery], top_k: int = 10):
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
async def run_concurrent_benchmark(queries: List[TestQuery], top_k: int = 10, concurrency: int = 5):
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