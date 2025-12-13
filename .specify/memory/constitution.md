<!--
SYNC IMPACT REPORT
- Version: 1.0.0 -> 1.1.0
- Change: Added principles for RAG chatbot integration.
- Added Sections:
  - Principle 8: Spec-Driven RAG Development
  - Principle 9: Grounded RAG Responses
  - Principle 10: Standardized RAG Technology Stack
  - Principle 11: Dual-Mode Retrieval Pipeline
  - Principle 12: Secure and Decoupled Architecture
  - Principle 13: Environment Parity
  - Principle 14: Robust RAG Error Handling
- Removed Sections: None
- Templates Requiring Updates:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
  - ⚠ .claude/commands/sp.constitution.md (and other .claude files should be reviewed for agent-specific language)
- Follow-up TODOs: None
-->

# Constitution of the Book Project

| Version | Ratification Date | Last Amended |
|---|---|---|
| 1.1.0 | 2025-12-12 | 2025-12-13 |

## 1. Project Overview

This document outlines the guiding principles and standards for the "Book" project. The project's goal is to create educational content for beginner to intermediate learners in AI, robotics, and computer science. This constitution ensures that all contributions are clear, accurate, and accessible.

## 2. Governance

This constitution is a living document. Amendments follow this process:
- **Proposal:** Any contributor may propose a change via a pull request.
- **Review:** The proposal must be reviewed and approved by project maintainers.
- **Versioning:** Changes are versioned semantically:
  - **MAJOR:** Incompatible changes (e.g., removing a core principle).
  - **MINOR:** Adding new principles or sections.
  - **PATCH:** Clarifications and typo fixes.

## 3. Principles

### Principle 1: Clarity and Accessibility

- **Rule:** Content MUST be written for a beginner-to-intermediate audience. Explanations must build intuition first before diving into technical details. The target Flesch-Kincaid reading grade is 8-10.
- **Rationale:** To make complex topics in AI, robotics, and computer science approachable for newcomers.

### Principle 2: Structured and Progressive Learning

- **Rule:** Content MUST be presented with step-by-step clarity. Use diagrams, examples, and progressive code samples to build knowledge incrementally.
- **Rationale:** A structured approach helps learners build a strong foundation and connect concepts effectively.

### Principle 3: No Jargon Without Definition

- **Rule:** Technical jargon MUST be defined clearly upon its first use. Avoid assuming prior knowledge.
- **Rationale:** Ensures that readers are not lost due to unfamiliar terminology, promoting a smoother learning experience.

### Principle 4: Factual and Technical Accuracy

- **Rule:** All technical information, claims, and code examples MUST be fact-checked and verified for correctness. Content must be supported by primary sources where applicable.
- **Rationale:** To build trust and provide reliable educational material. The project has zero tolerance for misinformation.

### Principle 5: Authoritative and Ethical Sourcing

- **Rule:** A minimum of 50% of sources MUST be from peer-reviewed papers or authoritative technical documentation. All sources MUST be cited using APA format. The project has a zero-tolerance policy for plagiarism.
- **Rationale:** Upholds academic and ethical standards, ensuring the content is well-researched and gives credit appropriately.

### Principle 6: Executable and Reproducible Code

- **Rule:** All code examples provided MUST be complete, executable, and correct. The project will emphasize reproducibility, ensuring that learners can replicate results.
- **Rationale:** Practical, working code is essential for hands-on learning and reinforcing theoretical concepts.

### Principle 7: Consistency Across Deliverables

- **Rule:** These principles apply to all project deliverables, including the book itself and any associated systems (e.g., a RAG system). Factual correctness and clarity are paramount everywhere.
- **Rationale:** To ensure a consistent and high-quality experience across the entire project ecosystem.

### Principle 8: Spec-Driven RAG Development

- **Rule:** All RAG features and integrations MUST follow a strict Specify → Plan → Tasks workflow. No implementation work should begin without a clear specification and a task breakdown.
- **Rationale:** Ensures that RAG development is deliberate, well-architected, and aligned with project goals before any code is written.

### Principle 9: Grounded RAG Responses

- **Rule:** The RAG chatbot MUST ground its responses exclusively in the embedded book content or user-selected text. The system MUST NOT answer from its own internal knowledge.
- **Rationale:** Guarantees that the chatbot serves as a reliable guide to the book's content, preventing hallucinations and maintaining factual accuracy.

### Principle 10: Standardized RAG Technology Stack

- **Rule:** The RAG system MUST use Cohere models for embeddings, Qdrant as the vector database, and the OpenAI Agents SDK with FastAPI for the agent implementation.
- **Rationale:** Standardizes the technology stack to ensure consistency, maintainability, and focused expertise.

### Principle 11: Dual-Mode Retrieval Pipeline

- **Rule:** The retrieval pipeline MUST support both querying the entire book and querying specific sections or user-selected text.
- **Rationale:** Provides users with flexible tools to either explore the whole knowledge base or get targeted answers on a focused body of text.

### Principle 12: Secure and Decoupled Architecture

- **Rule:** The architecture MUST enforce a strict separation between the backend (agent, retrieval pipeline) and the frontend (Docusaurus UI). API keys and other secrets MUST NOT be exposed in frontend code.
- **Rationale:** Protects sensitive credentials, enhances security, and allows the frontend and backend to be developed and scaled independently.

### Principle 13: Environment Parity

- **Rule:** The RAG integration MUST be designed for local and production compatibility. The connection between the Docusaurus frontend and the FastAPI backend MUST work seamlessly in both environments.
- **Rationale:** Ensures that the system is testable locally and deployable to production with minimal friction.

### Principle 14: Robust RAG Error Handling

- **Rule:** The RAG chatbot MUST implement comprehensive validation and error handling. It MUST gracefully inform the user when no relevant context is found rather than failing silently or providing a poor answer.
- **Rationale:** Creates a reliable and user-friendly experience by managing expectations and providing clear feedback when a query cannot be fulfilled.