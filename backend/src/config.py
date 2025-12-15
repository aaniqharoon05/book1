import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

class Settings:
    """Configuration settings for the ingestion pipeline."""

    # Cohere settings
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "3heuCI9Lu3kwwJX8IYElMiZKjtQl7OHSSyGZycRC")
    COHERE_EMBED_MODEL: str = os.getenv("COHERE_EMBED_MODEL", "embed-english-v3.0")

    # Qdrant settings
    QDRANT_URL: str = os.getenv("QDRANT_URL", "https://ded50be7-2685-4d17-8e24-55934390ea6a.us-east4-0.gcp.cloud.qdrant.io")
    QDRANT_API_KEY: str = os.getenv("QDRANT_API_KEY", "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.OWMxnns5DydpJlKrJu6TdF8VV8XLcFeE8BbxP6MH4Mk")
    QDRANT_PORT: int = int(os.getenv("QDRANT_PORT", 6333))

    # Content processing settings
    CHUNK_SIZE: int = int(os.getenv("CHUNK_SIZE", 512))
    CHUNK_OVERLAP: int = int(os.getenv("CHUNK_OVERLAP", 20))  # Number of characters to overlap
    _SOURCE_URL: str = os.getenv("SOURCE_URL", "https://physical-ai-kappa.vercel.app/")
    SITEMAP_URL : str = str("https://physical-ai-kappa.vercel.app/sitemap.xml")

    @property
    def SOURCE_URL(self) -> str:
        return self._SOURCE_URL

    @property
    def SITEMAP_URL(self) -> str:
        return f"{self.SOURCE_URL.rstrip('/')}/sitemap.xml"

    # Validation
    def validate(self):
        """Validate that required settings are present."""
        errors = []
        if not self.COHERE_API_KEY:
            errors.append("COHERE_API_KEY is required")
        if not self.QDRANT_URL:
            errors.append("QDRANT_URL is required")

        if errors:
            raise ValueError(f"Configuration errors: {'; '.join(errors)}")

# Global settings instance
settings = Settings()

# Validate settings on import
settings.validate()