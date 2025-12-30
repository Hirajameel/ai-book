# Implementation Plan: FastAPI RAG Integration

**Branch**: `4-fastapi-rag-integration` | **Date**: 2025-01-05 | **Spec**: [specs/4-fastapi-rag-integration/spec.md](spec.md)
**Input**: Feature specification from `/specs/4-fastapi-rag-integration/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a FastAPI backend that exposes a RAG agent endpoint for the book frontend. The system will allow users to query book content via a chatbot UI, with support for both full-book queries and context-restricted queries based on selected text. The backend will integrate with the existing agent from `backend/agent.py` and provide JSON responses to the frontend.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: FastAPI, OpenAI Agents SDK, Qdrant, Cohere, Docusaurus (React-based)
**Storage**: Qdrant Vector Database (for embeddings), Neon Serverless Postgres (for chat history/user data)
**Testing**: pytest for backend API testing
**Target Platform**: Web application (Docusaurus frontend with FastAPI backend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <5 second response time for user queries under normal load
**Constraints**: Local development only, must integrate with existing agent.py, must maintain compatibility with Docusaurus frontend
**Scale/Scope**: Single user local development environment

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the constitution, the following checks apply and have been satisfied:
1. All Python code will follow PEP 8
2. Technical instructions will be verified against official documentation
3. Maintains the educational narrative focus
4. All code will be testable and reproducible
5. Aligns with the defined project architecture (Docusaurus frontend + FastAPI backend + Qdrant vector DB)

## Project Structure

### Documentation (this feature)

```text
specs/4-fastapi-rag-integration/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── api.py               # New FastAPI server with query endpoint
├── agent.py             # Existing RAG agent (to be reused from Spec-3)
├── models.py            # Data models
├── retrieval_tool.py    # Retrieval pipeline (from Spec-2)
├── config.py            # Configuration
└── [other existing files]

frontend-docu/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
└── [other Docusaurus files]
```

**Structure Decision**: Web application structure selected as this involves both frontend (Docusaurus) and backend (FastAPI) components. The backend will be extended with a new api.py file containing the FastAPI server, while the existing agent.py will be integrated. The frontend will use the existing Docusaurus structure with chatbot UI components.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
