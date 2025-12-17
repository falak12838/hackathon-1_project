<!-- SYNC IMPACT REPORT:
Version change: N/A (initial version) → 1.0.0
Added sections: All sections (new constitution)
Removed sections: None
Templates requiring updates: ⚠ pending - .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md
Follow-up TODOs: None
-->
# AI-Spec Driven Book with Integrated RAG Chatbot Constitution

## Core Principles

### Spec-First Development (No Assumptions)
All development must follow spec-driven methodology with no assumptions made; Every feature and component must be defined in specifications before implementation; Implementation must strictly follow documented specifications without deviation.

### Technical Accuracy (No Hallucinated APIs)
All technical claims must be accurate and based on real APIs or official documentation; No fabricated or hallucinated APIs, functions, or capabilities allowed; All examples must be verified against official sources or working implementations.

### Clarity for Beginner–Intermediate Developers
Content and code must be accessible to beginner and intermediate developers; Complex concepts should be explained with practical examples and clear explanations; Documentation must bridge the gap between basic understanding and advanced usage.

### Unified, End-to-End System Design
All components must work together as a unified system; Individual components must be designed with the entire system architecture in mind; Integration points between components must be clearly defined and tested.

### Production-Ready, Reproducible Output
All deliverables must be production-ready quality; The system must be fully reproducible from specifications; Deployment and operational considerations must be addressed in design and implementation.

## Technology Stack Requirements

### Tooling Standards
Must use Spec-Kit Plus for project management and specifications; Must use Claude Code for AI-assisted development; Must use Docusaurus for book generation and publishing.

### Deployment Requirements
Deployment must target GitHub Pages for hosting; Static site generation required for performance and reliability; All deployment processes must be documented and reproducible.

### Content Standards
Content must be modular with clear table of contents; Chapters must be structured for easy navigation and understanding; Runnable examples must be provided for all code snippets and concepts.

### Documentation Alignment
All claims must be aligned with official documentation or recognized standards; No theoretical-only components allowed; All functionality must be verified against actual implementation.

## RAG Chatbot Requirements

### Integration Requirements
Chatbot must be embedded directly in the published book interface; Integration must be seamless with the book's user experience; Chatbot functionality must be accessible from all book pages.

### Query Capabilities
Chatbot must answer full-book queries across all content; Chatbot must support selected-text-only queries as specified by user; Both query modes must provide accurate and contextually relevant responses.

### Technical Stack
Backend must use FastAPI framework for API endpoints; Vector storage must use Qdrant Cloud (Free Tier) for embeddings; Database must use Neon Serverless Postgres for data persistence; AI integration must use OpenAI Agents or ChatKit SDKs.

### Pipeline Requirements
Complete ingestion pipeline must be implemented for content processing; Complete embedding pipeline must be implemented for vector storage; Complete retrieval pipeline must be implemented for answer generation; All pipelines must handle errors gracefully and provide clear feedback.

## Development Standards

### Quality Assurance
No theoretical-only components allowed - all functionality must be implemented and tested; Complete ingestion, embedding, retrieval pipeline must be functional; Security and deployment notes must be clearly documented.

### Success Criteria
Book must build and deploy successfully without errors; Chatbot must retrieve accurate, scoped answers for both full-book and selected-text queries; All components must work as a single integrated system as specified.

### Constraint Adherence
All components must work within free-tier limitations of specified technologies; Architecture must support reproducible setup and deployment; Implementation must follow production-ready patterns and practices.

## Governance

Constitution supersedes all other practices and guidelines; Amendments require formal documentation and approval process; All development must comply with these principles; Spec validation serves as compliance checkpoint.

**Version**: 1.0.0 | **Ratified**: 2025-12-15 | **Last Amended**: 2025-12-15