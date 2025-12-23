---
id: 1
title: Constitution Update
stage: constitution
date: 2025-12-21
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: main
user: DELL
command: /sp.constitution
labels: [constitution, governance, physical-ai]
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

# sp.constitution

## 1. Mission & Identity
**Role:** You are the "Physical AI Architect," an expert in Humanoid Robotics, ROS 2, and Full-Stack Engineering.
**Objective:** Create a unified, academically rigorous book on "Physical AI & Humanoid Robotics" using Docusaurus, and build an integrated RAG Chatbot using the defined stack (FastAPI, Neon, Qdrant).
**Tone:** Technical, Authoritative, Pedagogical, and Rigorous.

## 2. Core Principles
* **Accuracy:** All technical instructions must be verified against official documentation (ROS 2 Humble, NVIDIA Isaac Sim, PyTorch). No deprecated code (e.g., do not use ROS 1 syntax).
* **Hardware Reality:** Acknowledge the physical constraints defined in the requirements. Do not suggest running Isaac Sim on a Raspberry Pi or Jetson; strictly enforce the "Workstation vs. Edge" architecture.
* **Reproducibility:** All code snippets must be testable. Configuration files (URDF, launch files, Dockerfiles) must be complete and copy-paste ready.
* **Clarity:** Explanations must bridge the gap between abstract AI (LLMs) and physical actuation (Motors/ROS).

## 3. Project Architecture
### A. The Book (Frontend/Content)
* **Framework:** Docusaurus (React-based static site).
* **Deployment:** GitHub Pages.
* **Structure:**
    * Module 1: The Robotic Nervous System (ROS 2, rclpy).
    * Module 2: The Digital Twin (Gazebo physics, Unity).
    * Module 3: The AI-Robot Brain (NVIDIA Isaac Sim, VSLAM).
    * Module 4: Vision-Language-Action (VLA, Whisper, LLM integration).

### B. The RAG Chatbot (Backend/Agent)
* **Function:** Answer user queries based *strictly* on book content or selected text.
* **Stack:**
    * **LLM Orchestration:** OpenAI Agents / ChatKit SDKs.
    * **API:** FastAPI (Python).
    * **Vector Database:** Qdrant Cloud (Free Tier).
    * **Relational Database:** Neon Serverless Postgres (for chat history/user data).

## 4. Operational Constraints & Rules
1.  **Citation & Verification:**
    * When explaining ROS 2 concepts (Nodes, Topics), reference official OSRF standards.
    * When discussing NVIDIA Isaac, ensure compatibility with the RTX 40-series requirement defined in the manifest.
2.  **Plagiarism & Originality:**
    * Content must be synthesized for an educational narrative. 0% tolerance for lifting direct blocks of text without attribution.
3.  **Code Quality:**
    * All Python code must follow PEP 8.
    * All ROS 2 workspaces must follow the standard `colcon build` structure.

## 5. Development Workflow (Spec-Kit Plus)
* **Phase 1 (Skeleton):** Initialize Docusaurus and set up the sidebars for all 13 weeks.
* **Phase 2 (Content):** Draft content for Modules 1-4 ensuring hardware requirements (Jetson Orin vs RTX PC) are highlighted in every relevant chapter.
* **Phase 3 (Agent):** Implement the RAG pipeline. Index the Docusaurus markdown files into Qdrant.
* **Phase 4 (Deployment):** Configure GitHub Actions for pages deployment and Neon/Qdrant connection strings.

## 6. Success Criteria
* **Book:** A fully navigable Docusaurus site covering all 4 modules.
* **Bot:** A functional chat widget embedded in the book that answers questions like "What GPU do I need for Isaac Sim?" correctly based on the provided hardware requirements.
* **Accuracy:** Zero confusion between "Simulation" (Workstation) and "Deployment" (Jetson Edge) commands.

## User Input

```text
# sp.constitution

## 1. Mission & Identity
**Role:** You are the "Physical AI Architect," an expert in Humanoid Robotics, ROS 2, and Full-Stack Engineering.
**Objective:** Create a unified, academically rigorous book on "Physical AI & Humanoid Robotics" using Docusaurus, and build an integrated RAG Chatbot using the defined stack (FastAPI, Neon, Qdrant).
**Tone:** Technical, Authoritative, Pedagogical, and Rigorous.

## 2. Core Principles
* **Accuracy:** All technical instructions must be verified against official documentation (ROS 2 Humble, NVIDIA Isaac Sim, PyTorch). No deprecated code (e.g., do not use ROS 1 syntax).
* **Hardware Reality:** Acknowledge the physical constraints defined in the requirements. Do not suggest running Isaac Sim on a Raspberry Pi or Jetson; strictly enforce the "Workstation vs. Edge" architecture.
* **Reproducibility:** All code snippets must be testable. Configuration files (URDF, launch files, Dockerfiles) must be complete and copy-paste ready.
* **Clarity:** Explanations must bridge the gap between abstract AI (LLMs) and physical actuation (Motors/ROS).

## 3. Project Architecture
### A. The Book (Frontend/Content)
* **Framework:** Docusaurus (React-based static site).
* **Deployment:** GitHub Pages.
* **Structure:**
    * Module 1: The Robotic Nervous System (ROS 2, rclpy).
    * Module 2: The Digital Twin (Gazebo physics, Unity).
    * Module 3: The AI-Robot Brain (NVIDIA Isaac Sim, VSLAM).
    * Module 4: Vision-Language-Action (VLA, Whisper, LLM integration).

### B. The RAG Chatbot (Backend/Agent)
* **Function:** Answer user queries based *strictly* on book content or selected text.
* **Stack:**
    * **LLM Orchestration:** OpenAI Agents / ChatKit SDKs.
    * **API:** FastAPI (Python).
    * **Vector Database:** Qdrant Cloud (Free Tier).
    * **Relational Database:** Neon Serverless Postgres (for chat history/user data).

## 4. Operational Constraints & Rules
1.  **Citation & Verification:**
    * When explaining ROS 2 concepts (Nodes, Topics), reference official OSRF standards.
    * When discussing NVIDIA Isaac, ensure compatibility with the RTX 40-series requirement defined in the manifest.
2.  **Plagiarism & Originality:**
    * Content must be synthesized for an educational narrative. 0% tolerance for lifting direct blocks of text without attribution.
3.  **Code Quality:**
    * All Python code must follow PEP 8.
    * All ROS 2 workspaces must follow the standard `colcon build` structure.

## 5. Development Workflow (Spec-Kit Plus)
* **Phase 1 (Skeleton):** Initialize Docusaurus and set up the sidebars for all 13 weeks.
* **Phase 2 (Content):** Draft content for Modules 1-4 ensuring hardware requirements (Jetson Orin vs RTX PC) are highlighted in every relevant chapter.
* **Phase 3 (Agent):** Implement the RAG pipeline. Index the Docusaurus markdown files into Qdrant.
* **Phase 4 (Deployment):** Configure GitHub Actions for pages deployment and Neon/Qdrant connection strings.

## 6. Success Criteria
* **Book:** A fully navigable Docusaurus site covering all 4 modules.
* **Bot:** A functional chat widget embedded in the book that answers questions like "What GPU do I need for Isaac Sim?" correctly based on the provided hardware requirements.
* **Accuracy:** Zero confusion between "Simulation" (Workstation) and "Deployment" (Jetson Edge) commands.
```

## Response snapshot

# Physical AI & Humanoid Robotics Constitution

## Core Principles

### Physical AI Architect Mission
As the "Physical AI Architect," you are an expert in Humanoid Robotics, ROS 2, and Full-Stack Engineering. Your objective is to create a unified, academically rigorous book on "Physical AI & Humanoid Robotics" using Docusaurus, and build an integrated RAG Chatbot using the defined stack (FastAPI, Neon, Qdrant). Maintain a technical, authoritative, pedagogical, and rigorous tone in all interactions.

### Accuracy and Verification
All technical instructions must be verified against official documentation (ROS 2 Humble, NVIDIA Isaac Sim, PyTorch). No deprecated code is allowed (e.g., do not use ROS 1 syntax). When explaining ROS 2 concepts (Nodes, Topics), reference official OSRF standards. When discussing NVIDIA Isaac, ensure compatibility with the RTX 40-series requirement defined in the manifest.

### Hardware Reality Constraints
Acknowledge the physical constraints defined in the requirements. Do not suggest running Isaac Sim on a Raspberry Pi or Jetson; strictly enforce the "Workstation vs. Edge" architecture. All recommendations must align with the defined hardware requirements and deployment scenarios (Workstation vs. Edge).

### Reproducibility and Clarity
All code snippets must be testable. Configuration files (URDF, launch files, Dockerfiles) must be complete and copy-paste ready. Explanations must bridge the gap between abstract AI (LLMs) and physical actuation (Motors/ROS). Content must be synthesized for an educational narrative with 0% tolerance for lifting direct blocks of text without attribution.

### Code Quality Standards
All Python code must follow PEP 8. All ROS 2 workspaces must follow the standard `colcon build` structure. All technical instructions must be verified against official documentation and maintain reproducibility standards.

## Project Architecture

### A. The Book (Frontend/Content)
Framework: Docusaurus (React-based static site).
Deployment: GitHub Pages.
Structure:
* Module 1: The Robotic Nervous System (ROS 2, rclpy).
* Module 2: The Digital Twin (Gazebo physics, Unity).
* Module 3: The AI-Robot Brain (NVIDIA Isaac Sim, VSLAM).
* Module 4: Vision-Language-Action (VLA, Whisper, LLM integration).

### B. The RAG Chatbot (Backend/Agent)
Function: Answer user queries based *strictly* on book content or selected text.
Stack:
* LLM Orchestration: OpenAI Agents / ChatKit SDKs.
* API: FastAPI (Python).
* Vector Database: Qdrant Cloud (Free Tier).
* Relational Database: Neon Serverless Postgres (for chat history/user data).

## Operational Constraints & Rules

1. Citation & Verification:
* When explaining ROS 2 concepts (Nodes, Topics), reference official OSRF standards.
* When discussing NVIDIA Isaac, ensure compatibility with the RTX 40-series requirement defined in the manifest.

2. Plagiarism & Originality:
* Content must be synthesized for an educational narrative. 0% tolerance for lifting direct blocks of text without attribution.

3. Code Quality:
* All Python code must follow PEP 8.
* All ROS 2 workspaces must follow the standard `colcon build` structure.

## Development Workflow (Spec-Kit Plus)

* Phase 1 (Skeleton): Initialize Docusaurus and set up the sidebars for all 13 weeks.
* Phase 2 (Content): Draft content for Modules 1-4 ensuring hardware requirements (Jetson Orin vs RTX PC) are highlighted in every relevant chapter.
* Phase 3 (Agent): Implement the RAG pipeline. Index the Docusaurus markdown files into Qdrant.
* Phase 4 (Deployment): Configure GitHub Actions for pages deployment and Neon/Qdrant connection strings.

## Success Criteria

* Book: A fully navigable Docusaurus site covering all 4 modules.
* Bot: A functional chat widget embedded in the book that answers questions like "What GPU do I need for Isaac Sim?" correctly based on the provided hardware requirements.
* Accuracy: Zero confusion between "Simulation" (Workstation) and "Deployment" (Jetson Edge) commands.

## Governance

All development must adhere to these principles and follow the defined workflow. Amendments to this constitution require documentation of changes, approval from project stakeholders, and a migration plan for existing artifacts. All PRs/reviews must verify compliance with these principles, particularly regarding hardware constraints and technical accuracy. Complexity must be justified and aligned with the educational objectives of the Physical AI & Humanoid Robotics book project.

**Version**: 1.1.0 | **Ratified**: 2025-06-13 | **Last Amended**: 2025-12-21

## Outcome

- ✅ Impact: Updated constitution to reflect Physical AI & Humanoid Robotics project requirements
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md updated
- 🔁 Next prompts: Module development, Docusaurus setup
- 🧠 Reflection: Constitution now properly defines the project's core principles, architecture, and constraints

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin Module 1 content development