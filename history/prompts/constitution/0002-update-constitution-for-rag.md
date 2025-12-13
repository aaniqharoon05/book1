---
id: 0002
title: update-constitution-for-rag
stage: constitution
date: 2025-12-13
surface: CLI
model: gemini-pro
feature: rag-chatbot
branch: main
user:
command: /sp.constitution
labels: [constitution, rag, governance]
links:
  spec:
  ticket:
  adr:
  pr:
files:
  - .specify/memory/constitution.md
tests:
---

## Prompt

/sp.constitution update the constitution.md to include governance rules for integrating a Retrieval-Augmented Generation (RAG) chatbot into the Docusaurus book.

The constitution must enforce:
- Spec-driven development (Specify → Plan → Tasks) for all RAG work
- Strict grounding of responses in embedded book content or user-selected text only
- Use of Cohere models for embeddings and Qdrant as the vector database
- A retrieval pipeline that supports both full-book and selected-text queries
- An agent built with OpenAI Agents SDK and FastAPI that must retrieve context before answering
- Clear separation between backend (agent + retrieval) and frontend (Docusaurus UI)
- Secure API usage with no keys exposed in the frontend
- Local and production compatibility for frontend-backend integration
- Validation, error handling, and graceful failure when no relevant context is found

The constitution should define architectural boundaries, quality standards, and completion criteria for the RAG chatbot integration.

## Response snapshot

I have updated the constitution to version 1.1.0 to include the new principles for the RAG chatbot integration. I have also verified that the existing templates do not require modification.

## Outcome

- ✅ Impact: The project constitution was updated to include specific governance for RAG chatbot development, ensuring alignment with architectural and quality standards.
- 🧪 Tests: No tests were run.
- 📁 Files: 1 file was modified: `.specify/memory/constitution.md`.
- 🔁 Next prompts: The next logical step would be to create a specification for the RAG feature using `/sp.specify`.
- 🧠 Reflection: The process was smooth. The new principles were clearly defined and integrated into the existing constitution. The fallback plan for PHR creation worked as expected.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script was not found, requiring the use of the fallback manual creation method.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
