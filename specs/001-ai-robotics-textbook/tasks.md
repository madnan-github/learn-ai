# Tasks: AI-Native Textbook on Physical AI & Humanoid Robotics

**Input**: Design documents from `/home/ruser/q4/learn-ai/specs/001-ai-robotics-textbook/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Tests are not explicitly requested in the feature specification, therefore, no dedicated test tasks are generated.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `backend/src/`, `frontend/src/`

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create Docusaurus project structure for frontend in `frontend/`
- [ ] T002 Initialize Node.js/TypeScript project for frontend in `frontend/`
- [ ] T003 Create FastAPI project structure for backend in `backend/`
- [ ] T004 Initialize Python 3.11 project for backend in `backend/`
- [ ] T005 [P] Configure linting and formatting tools for frontend and backend (e.g., `frontend/.prettierrc`, `backend/.flake8`)
- [ ] T006 [P] Set up Git repository and initial commit for `001-ai-robotics-textbook` branch

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T007 [P] Setup Neon Serverless Postgres for user data and Qdrant Cloud Free Tier for embeddings (configuration files in `backend/src/config/`)
- [ ] T008 [P] Implement database connection and ORM setup for Postgres in `backend/src/database/`
- [ ] T009 [P] Setup authentication framework using Better-auth in `backend/src/auth/`
- [ ] T010 [P] Configure FastAPI routing and middleware structure in `backend/src/api/`
- [ ] T011 [P] Create base User model and schema for authentication in `backend/src/models/user.py`
- [ ] T012 [P] Create base UserProfile model and schema in `backend/src/models/user_profile.py`
- [ ] T013 [P] Configure error handling and logging infrastructure for backend in `backend/src/utils/logger.py`, `backend/src/middleware/error_handler.py`
- [ ] T014 [P] Setup environment configuration management for backend (e.g., `.env` handling in `backend/src/config/`)
- [ ] T015 [P] Configure Docusaurus for multi-language support (Urdu) in `frontend/docusaurus.config.js`
- [ ] T016 [P] Integrate basic Better-auth client-side components into Docusaurus frontend in `frontend/src/components/auth/`

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Textbook Content Access (Priority: P1) 🎯 MVP

**Goal**: Readers can navigate the textbook, read chapters, and view code examples to learn about Physical AI and humanoid robotics.

**Independent Test**: Can be fully tested by navigating through all modules and chapters, verifying content display, and running provided code examples successfully.

### Implementation for User Story 1

- [ ] T017 [P] [US1] Create Docusaurus pages for 4 modules and 15 chapters in `frontend/docs/`
- [ ] T018 [P] [US1] Add initial content for Module 1 chapters, including basic code examples in `frontend/docs/module1/`
- [ ] T019 [P] [US1] Add initial content for Module 2 chapters, including basic code examples in `frontend/docs/module2/`
- [ ] T020 [P] [US1] Add initial content for Module 3 chapters, including basic code examples in `frontend/docs/module3/`
- [ ] T021 [P] [US1] Add initial content for Module 4 chapters, including basic code examples in `frontend/docs/module4/`
- [ ] T022 [US1] Implement navigation for modules and chapters in `frontend/docusaurus.config.js`
- [ ] T023 [P] [US1] Implement display of code examples within Docusaurus markdown in `frontend/src/components/CodeBlock.tsx`
- [ ] T024 [P] [US1] Implement assessment questions and project ideas for Module 1 in `frontend/docs/module1/assessments.md`
- [ ] T025 [P] [US1] Implement assessment questions and project ideas for Module 2 in `frontend/docs/module2/assessments.md`
- [ ] T026 [P] [US1] Implement assessment questions and project ideas for Module 3 in `frontend/docs/module3/assessments.md`
- [ ] T027 [P] [US1] Implement assessment questions and project ideas for Module 4 in `frontend/docs/module4/assessments.md`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - RAG Chatbot Interaction (Priority: P1) 🎯 MVP

**Goal**: Users can ask questions about the textbook content and receive accurate answers from the RAG chatbot, enhancing their understanding and allowing interactive learning.

**Independent Test**: Can be fully tested by asking a range of questions related to textbook content and verifying the accuracy and relevance of chatbot responses, including queries based on selected text.

### Implementation for User Story 2

- [ ] T028 [P] [US2] Create embeddings for existing textbook content and store in Qdrant in `backend/src/services/embedding_service.py`, `backend/src/database/qdrant_client.py`
- [ ] T029 [P] [US2] Implement RAG pipeline in `backend/src/services/rag_service.py` using OpenAI API
- [ ] T030 [P] [US2] Create API endpoint for chatbot queries in `backend/src/api/chatbot.py`
- [ ] T031 [US2] Implement frontend component for RAG chatbot interface in `frontend/src/components/Chatbot.tsx`
- [ ] T032 [P] [US2] Implement text selection functionality in Docusaurus frontend to provide context to chatbot in `frontend/src/utils/text_selection.ts`
- [ ] T033 [US2] Integrate chatbot component into Docusaurus layout in `frontend/src/theme/Layout.tsx`

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Personalized Learning (Priority: P2)

**Goal**: Authenticated users receive content recommendations and adaptations based on their knowledge level (beginner/intermediate/advanced) and prior experience (e.g., with ROS 1, Python, specific hardware), making the textbook more relevant to their individual needs.

**Independent Test**: Can be fully tested by creating different user profiles, observing how chapter content or recommendations change, and verifying that content adapts appropriately to the simulated backgrounds.

### Implementation for User Story 3

- [ ] T034 [P] [US3] Extend User Profile model to include `knowledge_level` and `prior_experience` fields in `backend/src/models/user_profile.py`
- [ ] T035 [P] [US3] Implement API endpoints for updating user profiles in `backend/src/api/user_profile.py`
- [ ] T036 [US3] Implement content personalization logic in `backend/src/services/personalization_service.py` to adapt content based on user profile
- [ ] T037 [P] [US3] Create frontend components for user profile management in `frontend/src/components/UserProfile.tsx`
- [ ] T038 [US3] Integrate personalization logic into Docusaurus content rendering in `frontend/src/theme/DocItem/Content/index.tsx`

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - Urdu Translation (Priority: P2)

**Goal**: Users can seamlessly translate textbook content into Urdu, with high-quality machine translation reviewed for accuracy, fluency, and cultural appropriateness in key technical terms.

**Independent Test**: Can be fully tested by switching the language to Urdu and verifying that all displayable content, including chapter text and UI elements, is accurately and coherently translated.

### Implementation for User Story 4

- [ ] T039 [P] [US4] Implement translation service using OpenAI API in `backend/src/services/translation_service.py`
- [ ] T040 [P] [US4] Create API endpoint for translating content chunks in `backend/src/api/translation.py`
- [ ] T041 [US4] Implement frontend language switcher and content translation display in `frontend/src/components/LanguageSwitcher.tsx`, `frontend/src/theme/DocItem/Content/index.tsx`
- [ ] T042 [P] [US4] Add placeholders for human review/post-editing workflow for Urdu translations (e.g., dedicated admin interface or external tool integration)

---

## Phase 7: User Story 5 - Capstone Project Engagement (Priority: P3)

**Goal**: Readers can follow comprehensive instructions to implement a voice-controlled humanoid project, applying the learned concepts in a practical, hands-on manner.

**Independent Test**: Can be fully tested by a user following the capstone project guide from start to finish, culminating in a functional voice-controlled humanoid simulation.

### Implementation for User Story 5

- [ ] T043 [P] [US5] Create dedicated Docusaurus section for Capstone Project guide in `frontend/docs/capstone/`
- [ ] T044 [P] [US5] Develop detailed instructions and code examples for voice control integration (e.g., using `speech_recognition` library) in `frontend/docs/capstone/voice_control.md`
- [ ] T045 [P] [US5] Develop detailed instructions and code examples for humanoid simulation setup (Gazebo/Unity) in `frontend/docs/capstone/simulation_setup.md`
- [ ] T046 [US5] Integrate all necessary ROS 2, Gazebo, NVIDIA Isaac components into the capstone project guide.

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T047 [P] Code cleanup and refactoring across backend and frontend
- [ ] T048 [P] Performance optimization for RAG chatbot queries and content loading
- [ ] T049 [P] Additional unit and integration tests for backend services in `backend/tests/`
- [ ] T050 [P] Cross-browser compatibility and mobile responsiveness verification for frontend
- [ ] T051 [P] Security hardening for API endpoints and user authentication in `backend/src/api/`, `backend/src/auth/`
- [ ] T052 [P] Configure CI/CD pipelines for automated testing and deployment (GitHub Actions for frontend, Vercel/Heroku for backend)
- [ ] T053 [P] Update `README.md` with deployment instructions and project overview.
- [ ] T054 [P] Define clear success metrics for each bonus feature (personalization, reusable subagents/skills) and update `spec.md` with these.
- [ ] T055 [P] Specify minimum content coverage for each module and update `spec.md` with these.
- [ ] T056 [P] Determine authentication flow and data storage details and update `plan.md` and `spec.md` with these.
- [ ] T057 [P] Plan chatbot training and evaluation methodology and update `plan.md` with these.
- [ ] T058 [P] Establish content personalization rules engine and update `plan.md` with these.
- [ ] T059 [P] Update `CLAUDE.md` to reference new feature-specific documentation (e.g. `spec.md`, `plan.md`, `tasks.md`).

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P1)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable
- **User Story 4 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1/US2/US3 but should be independently testable
- **User Story 5 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2/US3/US4 but should be independently testable

### Within Each User Story

- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tasks within a story marked [P] can run in parallel (if no explicit dependencies)
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all content creation tasks for User Story 1 together:
Task: "Add initial content for Module 1 chapters, including basic code examples in frontend/docs/module1/"
Task: "Add initial content for Module 2 chapters, including basic code examples in frontend/docs/module2/"
Task: "Add initial content for Module 3 chapters, including basic code examples in frontend/docs/module3/"
Task: "Add initial content for Module 4 chapters, including basic code examples in frontend/docs/module4/"
```

---

## Implementation Strategy

### MVP First (User Stories 1 & 2 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. Complete Phase 4: User Story 2
5. **STOP and VALIDATE**: Test User Stories 1 & 2 independently
6. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo (MVP!)
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Add User Story 5 → Test independently → Deploy/Demo
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
   - Developer D: User Story 4
   - Developer E: User Story 5
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence
