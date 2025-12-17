import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from app.services.validation_service import validation_service

info = validation_service.qdrant_client.get_collection(
    validation_service.collection_name
)

print(info)
print("Points count:", info.points_count)
