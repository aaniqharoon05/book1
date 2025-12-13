# Contributing to the Physical AI & Humanoid Robotics Book

We welcome contributions to the "Physical AI & Humanoid Robotics" book! By following these guidelines, you help ensure consistency, quality, and accessibility for all learners.

## 1. General Principles

All contributions must adhere to the project's **Constitution**, especially the following principles:

*   **Clarity and Accessibility**: Content must be written for a beginner-to-intermediate audience (Flesch-Kincaid grade 8-10). Build intuition first, then introduce technical details.
*   **Structured and Progressive Learning**: Use diagrams, examples, and progressive code samples.
*   **No Jargon Without Definition**: Define technical terms clearly on first use.
*   **Factual and Technical Accuracy**: All information must be fact-checked and verified.
*   **Authoritative and Ethical Sourcing**: Cite sources using APA format. Zero plagiarism.
*   **Executable and Reproducible Code**: Code examples must be complete, executable, and correct.

## 2. Lesson Format and Structure

Each lesson (`.md` or `.mdx` file) must follow this standardized structure:

```markdown
---
sidebar_position: X # Sequential number within the chapter
---

# Lesson X.Y: Your Lesson Title

**Learning Objectives:**
*   Objective 1
*   Objective 2
*   ...

## Main Content Section 1

[Beginner-friendly explanation. Use analogies, simple language.]

**[DIAGRAM: A brief text description of the diagram. e.g., "A diagram illustrating the flow of data from sensor to actuator."]**

### Practical Example: [Short descriptive title]

```python
# Short, relevant code snippet
print("Hello, Robot World!")
```

## Recap:
*   Summary point 1
*   Summary point 2
*   ...

## Try It Yourself:
[A small, actionable activity for the reader.]
```

## 3. Authoring Personalized Content (Skill Levels)

To support personalized learning experiences, content for different skill levels must be wrapped in specific MDX components within your `.mdx` files.

### Skill Level Components:

*   **`<SkillContent level="Beginner">`**: For foundational explanations, basic concepts, and simplified examples. This should be the default entry point for all learners.
*   **`<SkillContent level="Intermediate">`**: For content that builds upon beginner concepts, introduces more technical details, and provides slightly more complex examples or deeper dives.
*   **`<SkillContent level="Expert">`**: For advanced theoretical or implementation details, assuming significant prior knowledge. May include complex code snippets, advanced problem-solving techniques, or discussions on performance optimization.

### Example Usage:

```mdx
---
title: My Lesson Title
---

import SkillContent from '@site/src/components/SkillContent';

# My Lesson Title

<SkillContent level="Beginner">
This is content for beginners. It uses simple language and analogies.
</SkillContent>

<SkillContent level="Intermediate">
This is content for intermediate learners. It introduces more technical details.
</SkillContent>

<SkillContent level="Expert">
This is content for expert learners. It dives deep into implementation details and advanced concepts.
</SkillContent>
```

### Guidelines for Personalized Content:
*   Always provide content for the `Beginner` level.
*   Content within `<SkillContent>` tags should be self-contained.
*   Ensure a logical progression of difficulty between levels.
*   Minimize redundancy; emphasize how higher levels build on lower ones.

## 4. Docusaurus Formatting

*   Use standard Markdown and MDX syntax.
*   Utilize headers, subheaders, bullet points, and code blocks for readability.
*   Ensure all code blocks include a language identifier (e.g., ````python`).
*   Images should be placed in `static/img/` and referenced relatively or absolutely.

## 5. Submitting Contributions

1.  Fork the repository.
2.  Create a new branch for your changes (`git checkout -b feature/your-feature-name`).
3.  Implement your changes, adhering to the guidelines above.
4.  Test your changes locally (`npm run start` or `yarn start`).
5.  Commit your changes (`git commit -m "feat: Add new lesson on X"`).
6.  Push your branch (`git push origin feature/your-feature-name`).
7.  Open a Pull Request (PR) against the `main` branch, describing your changes.

Thank you for contributing to the learning community!
