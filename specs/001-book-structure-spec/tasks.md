# Task Breakdown: Docusaurus Book with Personalization

**Feature**: Book Structure, Content, and Personalization
**Branch**: `001-book-structure-spec`
**Spec**: `C:\hackathon1\book\specs\001-book-structure-spec\spec.md`

## Phase 1: Project Setup

- [X] T001 Install Node.js dependencies using `npm install` in the `book/` directory.

## Phase 2: Foundational Content Structure

- [X] T002 [P] Create the main introduction file `book/docs/intro.md`.
- [X] T003 [P] Create the directory `book/docs/chapter-1/`.
- [X] T004 [P] Create the directory `book/docs/chapter-2/`.
- [X] T005 [P] Create the directory `book/docs/chapter-3/`.

## Phase 3: User Story 1 - Structured Learning Path

**Goal**: A beginner learner can access a well-structured book with a clear introduction, chapters, and lessons.
**Independent Test**: The generated Docusaurus site can be navigated from the introduction, to Chapter 1, and to each of its lessons in the correct order.

- [X] T006 [P] [US1] Create the category metadata file `book/docs/chapter-1/_category_.json`.
- [X] T007 [P] [US1] Create the lesson file `book/docs/chapter-1/lesson-1.1.md`.
- [X] T008 [P] [US1] Create the lesson file `book/docs/chapter-1/lesson-1.2.md`.
- [X] T009 [P] [US1] Create the lesson file `book/docs/chapter-1/lesson-1.3.md`.
- [X] T010 [P] [US1] Create the category metadata file `book/docs/chapter-2/_category_.json`.
- [X] T011 [P] [US1] Create the lesson file `book/docs/chapter-2/lesson-2.1.md`.
- [X] T012 [P] [US1] Create the category metadata file `book/docs/chapter-3/_category_.json`.
- [X] T013 [P] [US1] Create the lesson file `book/docs/chapter-3/lesson-3.1.md`.
- [X] T014 [US1] Update `book/sidebars.ts` to reflect the complete book structure (intro, chapters, lessons).

## Phase 4: User Story 2 - Content Localization

**Goal**: A native Urdu speaker can translate the book's content into Urdu with a single click.
**Independent Test**: A user can click a button on a chapter page and the text content of that page is replaced with its Urdu translation.

- [X] T015 [US2] Configure `book/docusaurus.config.ts` for i18n, setting `defaultLocale: 'en'` and `locales: ['en', 'ur']`.
- [X] T016 [US2] Integrate the `localeDropdown` into the navbar in `book/docusaurus.config.ts`.
- [X] T017 [US2] Create the directory structure `book/i18n/ur/docusaurus-plugin-content-docs/current/`.
- [X] T018 [US2] Generate initial Urdu translation JSON files for all existing content using the Docusaurus `write-translations` command.

## Phase 5: User Story 4 - Personalized Learning Experience

**Goal**: A learner can adjust the complexity level of the content using a slider (Beginner, Intermediate, Expert).
**Independent Test**: A user can interact with a skill level slider, and the content's presentation style dynamically updates.

- [X] T019 [P] [US4] Create the React component directory `book/src/components/SkillLevelSlider/`.
- [X] T020 [US4] Implement the slider UI in `book/src/components/SkillLevelSlider/index.tsx`.
- [X] T021 [US4] Implement state management for the skill level, persisting the choice to `localStorage`.
- [X] T022 [US4] Define and document the content authoring format for different skill levels within a single Markdown file (e.g., using custom MDX components like `<BeginnerContent>`).
- [X] T023 [US4] Create custom MDX components (e.g., `BeginnerContent.tsx`) that conditionally render based on the selected skill level.
- [X] T024 [US4] "Swizzle" the `DocItem` component in Docusaurus and wrap it with a context provider for the skill level.
- [X] T025 [US4] Update all chapters to include content for all three skill levels using the new MDX components.

## Phase 6: User Story 3 - Authoring Consistency

**Goal**: A content contributor can follow clear content guidelines.
**Independent Test**: A new contributor can find and read a `CONTRIBUTING.md` file that explains the lesson structure.
- [X] T026 [US3] Create the `book/CONTRIBUTING.md` file, detailing the lesson format and content guidelines from the constitution.

## Phase 7: Polish & Deployment

- [X] T027 Run a full Docusaurus production build using `npm run build` in `book/` to ensure there are no errors.
- [ ] T028 Configure GitHub Pages deployment for the Docusaurus site.
- [ ] T029 Perform final QA checks on the deployed site (navigation, translation, personalization).

## Dependencies

- **US1 (Structured Path)** must be completed before other user stories, as it creates the files they modify.
- **US2 (Localization)** can be worked on in parallel with US4, but both depend on US1.
- **US4 (Personalization)** can be worked on in parallel with US2, but both depend on US1.
- **US3 (Authoring Consistency)** is independent and can be done at any time.

**User Story Completion Order:** US1 -> (US2 || US4) -> US3

## Parallel Execution Examples

- **During Phase 3 (US1)**: Tasks T006-T013 are marked with `[P]` and can be executed in parallel as they involve creating separate files.
- **After Phase 3 (US1)**:
  - **Team A** can begin Phase 4 (US2 - Localization).
  - **Team B** can begin Phase 5 (US4 - Personalization).
  - **Team C** can work on Phase 6 (US3 - `CONTRIBUTING.md`) at any time.

## Implementation Strategy

The implementation will follow an MVP-first approach.
1.  **MVP (User Story 1)**: The initial focus will be on establishing the core book structure and navigation. This delivers a readable, albeit untranslated and unpersonalized, version of the book.
2.  **Incremental Features**: Following the MVP, the localization (US2) and personalization (US4) features will be added. These are the next most valuable features for the end-user.
3.  **Documentation**: The authoring consistency documentation (US3) will be created last, as it is a lower priority for the initial user-facing product.

This strategy ensures that a valuable product is delivered quickly, with features added incrementally based on priority.
