---
id: 3
title: Create Constitution For AI Spec Driven Book
stage: constitution
date: 2025-12-15
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: Falak Sheikh Muhammad Asif
command: /sp.constitution
labels: [constitution,project-setup,ai-book,rag-chatbot,spec-driven]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

/sp.constitution

Project:
AI-Spec Driven Book with Integrated RAG Chatbot

Description:
Author a technical book using Spec-Kit Plus and Claude Code, built with Docusaurus and deployed to GitHub Pages. Embed a RAG chatbot that answers questions about the book, including queries restricted to user-selected text.

Core principles:
- Spec-first development (no assumptions)
- Technical accuracy (no hallucinated APIs)
- Clarity for beginner–intermediate developers
- Unified, end-to-end system design
- Production-ready, reproducible output

Key standards:
- Tooling: Spec-Kit Plus, Claude Code, Docusaurus
- Deployment: GitHub Pages
- Content: modular chapters, clear TOC, runnable examples
- All claims aligned with official documentation or standards

RAG chatbot requirements:
- Embedded in the published book
- Answers full-book and selected-text-only queries
- Stack:
  - OpenAI Agents / ChatKit SDKs
  - FastAPI
  - Neon Serverless Postgres
  - Qdrant Cloud (Free Tier)

Constraints:
- No theoretical-only components
- Complete ingestion, embedding, retrieval pipeline
- Clear security and deployment notes

Success criteria:
- Book builds and deploys successfully
- Chatbot retrieves accurate, scoped answers
- All components work as a single integrated system

## Response snapshot

Created a comprehensive constitution file for the AI-Spec Driven Book with Integrated RAG Chatbot project, incorporating all specified principles, technology stack requirements, RAG chatbot requirements, development standards, and governance rules. The constitution defines core principles including spec-first development, technical accuracy, clarity for developers, unified system design, and production-ready output. It also specifies technology stack requirements for tooling, deployment, content standards, and documentation alignment. The RAG chatbot section details integration, query capabilities, technical stack, and pipeline requirements. Development standards include quality assurance, success criteria, and constraint adherence. The governance section establishes the constitution's authority and amendment procedures.

## Outcome

- ✅ Impact: Created initial project constitution document with all required principles and standards
- 🧪 Tests: No tests required for this configuration file
- 📁 Files: Created .specify/memory/constitution.md with comprehensive project constitution
- 🔁 Next prompts: Project constitution is established, ready to proceed with spec creation
- 🧠 Reflection: Constitution successfully captures all specified project requirements and constraints

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): Not applicable
- Prompt variant (if applicable): Not applicable
- Next experiment (smallest change to try): Proceed with creating project specification based on constitution