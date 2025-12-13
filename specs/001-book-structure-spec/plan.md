# Implementation Plan: Book Structure and Content Specification

**Branch**: `001-book-structure-spec` | **Date**: 2025-12-12 | **Spec**: `C:\hackathon1\book\specs\001-book-structure-spec\spec.md`
**Input**: Feature specification from `C:\hackathon1\book\specs\001-book-structure-spec\spec.md`

## Summary

This plan details the creation of a Docusaurus-based educational book on Physical AI & Humanoid Robotics, structured with an introduction, chapters, and lessons. It outlines the content development workflow, Docusaurus implementation for structure and i18n, the Urdu translation process, and deployment via GitHub Pages. The plan also includes a new feature for user personalization, allowing readers to adjust content complexity via a skill-level slider. The primary goal is to provide a structured, accessible, translatable, and personalized learning experience.

## Technical Context

**Language/Version**: Python 3.x for code examples (rclpy), Node.js (latest LTS) for Docusaurus, Markdown/MDX for content, TypeScript/React for UI components.
**Primary Dependencies**:
*   Docusaurus (latest stable version)
*   `@docusaurus/preset-classic`
*   `@docusaurus/plugin-content-docs`
*   React.js
*   `rclpy` (ROS 2 client library for Python)
**Storage**: Local filesystem for Docusaurus content (`/docs`, `i18n/ur`). User preferences will be stored in the browser's `localStorage`.
**Testing**: Manual verification of navigation, content structure, personalization, and translation. Docusaurus `build` and `serve` commands for local validation.
**Target Platform**: Web browser (static site hosted on GitHub Pages).
**Project Type**: Web application (static site).
**Performance Goals**: Fast page load times, responsive UI, efficient build times for Docusaurus. Dynamic content updates for personalization should be near-instant.
**Constraints**:
*   Content must adhere to Flesch-Kincaid 8-10 for the "Beginner" level.
*   All technical information must be accurate.
*   Urdu translation via Docusaurus i18n.
*   Specific Docusaurus folder structure.
*   Use of ROS 2, Gazebo, NVIDIA Isaac.
*   User personalization state management.
**Scale/Scope**: Introduction, 3 Chapters, 9 lessons initially. Scalable to more chapters/lessons. Single target translation (Urdu). Three levels of content complexity (Beginner, Intermediate, Expert).

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Principle 1: Clarity and Accessibility
*   **Check**: Content generation will adhere to Flesch-Kincaid 8-10 for the "Beginner" level and build intuition. Higher levels will be more technical.
*   **Evaluation**: PASSED. This is a core requirement of content development.

### Principle 2: Structured and Progressive Learning
*   **Check**: Book structure (Intro, Chapters, Lessons) and lesson format enforce this. The personalization slider further enhances this.
*   **Evaluation**: PASSED. The planned structure directly implements this principle.

### Principle 3: No Jargon Without Definition
*   **Check**: The content generation process will explicitly include defining jargon on first use, especially for "Beginner" level.
*   **Evaluation**: PASSED. This will be a part of the content authoring guidelines.

### Principle 4: Factual and Technical Accuracy
*   **Check**: Requires careful authoring, review loops, and adherence to industry standards for all content levels.
*   **Evaluation**: PASSED. Content development workflow includes explicit review steps for accuracy.

### Principle 5: Authoritative and Ethical Sourcing
*   **Check**: APA citations and plagiarism checks for any external content.
*   **Evaluation**: PASSED. Content guidelines will include this.

### Principle 6: Executable and Reproducible Code
*   **Check**: All code examples will be tested and reproducible. "Expert" level may include more complex examples.
*   **Evaluation**: PASSED. Code examples will be focused and directly executable where possible.

### Principle 7: Consistency Across Deliverables
*   **Check**: All generated book content, Docusaurus configuration, and workflows will follow these principles.
*   **Evaluation**: PASSED. This plan ensures consistency across all aspects.

## Project Structure

### Documentation (this feature)

```text
specs/001-book-structure-spec/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (TBD)
├── data-model.md        # Phase 1 output (TBD - will include personalization attributes)
├── quickstart.md        # Phase 1 output (TBD - setup for book repo and personalization)
├── contracts/           # Phase 1 output (N/A for this feature)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
book/
├── .gitignore
├── docusaurus.config.ts  # Docusaurus configuration, including i18n setup
├── package-lock.json
├── package.json
├── README.md
├── sidebars.ts           # Sidebar configuration for navigation
├── tsconfig.json
├── blog/...
├── docs/                 # All book content resides here
│   ├── intro.md          # Introduction page
│   └── ...
├── node_modules/...
├── src/
│   ├── components/       # Custom React components
│   │   ├── SkillLevelSlider/
│   │   │   ├── index.tsx
│   │   │   └── styles.module.css
│   │   └── LanguageSwitch.tsx
│   ├── theme/            # Docusaurus theme swizzling for personalization
│   │   └── DocItem/
│   │       └── index.js
│   ├── css/
│   └── pages/
│       └── index.tsx
└── static/
    └── img/
i18n/                     # Docusaurus Internationalization files
└── ur/                   # Urdu translation directory
    └── ...
```

**Structure Decision**: The "Single project" option is chosen, with specific Docusaurus conventions applied to the `book/` directory. Custom components for the skill slider and theme swizzling for dynamic content display will be added to `src/`.

## Project Roadmap

### Phase 0-3: Content Drafting (as per previous plan)
*   (Tasks for generating Introduction, Chapters 1-3, and their lessons)

### Phase 4: Personalization Implementation
*   **Task 4.1**: Create a React component `SkillLevelSlider` in `src/components/` that allows users to select between "Beginner", "Intermediate", and "Expert".
*   **Task 4.2**: Implement state management for the selected skill level, persisting the choice in the browser's `localStorage` (FR-012).
*   **Task 4.3**: "Swizzle" the Docusaurus `DocItem` theme component to wrap it with the skill level context/provider.
*   **Task 4.4**: Define a content authoring format for different skill levels within a single Markdown file. A possible approach is using custom admonition blocks or MDX features, e.g., `<BeginnerContent>`, `<IntermediateContent>`, `<ExpertContent>`.
*   **Task 4.5**: Implement logic in the swizzled `DocItem` component to conditionally render the appropriate content block based on the selected skill level (FR-011).
*   **Task 4.6**: Update the content of one lesson (e.g., Lesson 1.1) to include content for all three skill levels to serve as a prototype.
*   **Task 4.7**: Test the dynamic content switching and state persistence across pages.

### Phase 5: Docusaurus i18n & Translation Setup
*   (Tasks for i18n setup, language switch button, and translation)

### Phase 6: Urdu Translation Verification & Refinement
*   (Tasks for manual review and refinement of Urdu translations)

### Phase 7: Final QA & Deployment
*   **Task 7.1**: Run Docusaurus build process.
*   **Task 7.2**: Configure GitHub Pages deployment.
*   **Task 7.3**: Perform final functional QA, including testing of navigation, language switching, and skill level personalization.
*   **Task 7.4**: Accessibility and HTML validation checks.
*   **Task 7.5**: Publish the book.

## Content Development Workflow
(The workflow remains the same as previously defined, with the addition that authors will need to provide content for different skill levels as per the format defined in Task 4.4.)

## Docusaurus Implementation Plan
(The plan remains the same as previously defined, with the addition of the Personalization implementation steps.)

## Translation Workflow (Urdu)
(The workflow remains the same as previously defined.)

## Deployment Plan
(The plan remains the same as previously defined, with the addition of personalization testing in the QA step.)
