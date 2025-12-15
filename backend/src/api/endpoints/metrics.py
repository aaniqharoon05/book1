from fastapi import APIRouter
import time
from typing import Dict, Any

router = APIRouter()

# Simple in-memory storage for metrics (in production, use a proper metrics system like Prometheus)
metrics_storage = {
    "requests_count": 0,
    "requests_by_endpoint": {},
    "response_times": []
}


@router.get("/metrics")
async def get_metrics() -> Dict[str, Any]:
    """
    Get system metrics for monitoring and observability
    """
    avg_response_time = (
        sum(metrics_storage["response_times"]) / len(metrics_storage["response_times"])
        if metrics_storage["response_times"]
        else 0
    )
    
    return {
        "requests_total": metrics_storage["requests_count"],
        "requests_by_endpoint": metrics_storage["requests_by_endpoint"],
        "average_response_time_ms": avg_response_time,
        "active_connections": 0  # Simplified for this example
    }


# Middleware to track metrics
def add_metrics_middleware(app):
    @app.middleware("http")
    async def metrics_middleware(request, call_next):
        start_time = time.time()
        
        # Count the request
        metrics_storage["requests_count"] += 1
        
        # Track endpoint requests
        endpoint = f"{request.method} {request.url.path}"
        if endpoint not in metrics_storage["requests_by_endpoint"]:
            metrics_storage["requests_by_endpoint"][endpoint] = 0
        metrics_storage["requests_by_endpoint"][endpoint] += 1
        
        response = await call_next(request)
        
        # Calculate response time and store it
        response_time = (time.time() - start_time) * 1000  # Convert to milliseconds
        metrics_storage["response_times"].append(response_time)
        
        # Keep only the last 1000 response times to prevent memory issues
        if len(metrics_storage["response_times"]) > 1000:
            metrics_storage["response_times"] = metrics_storage["response_times"][-1000:]
        
        return response