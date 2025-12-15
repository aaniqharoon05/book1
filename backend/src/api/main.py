from fastapi import FastAPI
from .endpoints import validation
from .endpoints import benchmark  # if benchmark endpoints are in a separate file
from .endpoints import metrics

app = FastAPI(
    title="RAG Retrieval Validation API",
    description="API for validating RAG retrieval pipeline accuracy and performance",
    version="1.0.0"
)

# Include the validation endpoints
app.include_router(validation.router, prefix="/api/v1", tags=["validation"])
app.include_router(benchmark.router, prefix="/api/v1", tags=["benchmark"])
app.include_router(metrics.router, prefix="/api/v1", tags=["metrics"])

# Add metrics middleware
metrics.add_metrics_middleware(app)


@app.get("/")
async def root():
    return {"message": "RAG Retrieval Validation API", "version": "1.0.0"}


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)