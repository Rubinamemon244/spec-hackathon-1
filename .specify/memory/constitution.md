<!--
Sync Impact Report:
Version change: N/A -> 1.0.0 (initial constitution)
Modified principles: None (new constitution)
Added sections: All sections (new constitution)
Removed sections: None
Templates requiring updates: ✅ none (new project)
Follow-up TODOs: None
-->

# AI/Spec-Driven Book with Embedded RAG Chatbot Constitution

## Core Principles

### Spec-First Development
Spec-first, accurate, clear, reproducible approach to development. All features must be defined in specifications before implementation begins, ensuring clarity, testability, and maintainability of deliverables.

### Modular Architecture
Modular, maintainable, developer-focused architecture. Components must be self-contained, independently testable, and well-documented with clear separation of concerns between book content delivery and RAG chatbot functionality.

### Technical Excellence (NON-NEGOTIABLE)
Code readable, commented, linted with no placeholder logic or hallucinated APIs. All implementations must follow established patterns, include comprehensive documentation, and pass quality gates before merging.

### Open Source Compliance
Open-source/free-tier only technology stack. All dependencies and services must comply with open-source licensing and free-tier availability to ensure project accessibility and reproducibility.

### End-to-End Quality Assurance
Focus on complete integration testing covering both book delivery and RAG functionality. All components must work together seamlessly to achieve the success criteria: successful build/deployment and accurate, context-aware chatbot responses.

### Developer-Centric Documentation
Professional, concise, developer-centric documentation with diagrams and examples preferred over long prose. All architectural decisions, APIs, and environment setup procedures must be clearly documented.

## Technical Standards & Constraints

- Document architecture, APIs, and environment setup comprehensively
- Open-source/free-tier only technology stack (Docusaurus, GitHub Pages, OpenAI Agents/ChatKit SDK, FastAPI, Neon Postgres, Qdrant)
- Code must be readable, commented, and linted with consistent formatting
- No placeholder logic or hallucinated APIs - all implementations must be complete and functional
- Project must be reproducible without extra steps beyond documented setup

## Development Workflow

- All features must begin with specification in accordance with Spec-Kit Plus methodology
- Book content and RAG chatbot functionality developed iteratively with continuous integration
- Both deliverables (book and chatbot) must be tested together to ensure proper integration
- Code reviews must verify compliance with architectural decisions and technical standards
- Deployment pipeline must validate successful build and functionality before release

## Governance

This constitution governs all development activities for the AI/Spec-Driven Book with Embedded RAG Chatbot project. All team members must adhere to these principles and standards. Any proposed changes to these principles require formal amendment process with documentation of rationale and impact assessment. All pull requests and code reviews must verify compliance with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-17 | **Last Amended**: 2025-12-17
