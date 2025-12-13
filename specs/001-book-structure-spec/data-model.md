# Data Model: Book Content Entities with Personalization

## Objective
To describe the key entities and their relationships for the book's content structure, including personalization attributes.

## Entities

### Book
*   **Description**: The top-level container for all educational content.
*   **Attributes**:
    *   `title`: String (e.g., "Physical AI & Humanoid Robotics")
    *   `introduction`: Reference to an `Introduction` entity.
    *   `chapters`: List of `Chapter` entities, ordered.

### Chapter
*   **Description**: A collection of related `Lesson` entities, forming a logical section of the book.
*   **Attributes**:
    *   `id`: String (e.g., "chapter-1")
    *   `title`: String (e.g., "The Robotic Nervous System (ROS 2)")
    *   `position`: Integer (order within the book)
    *   `lessons`: List of `Lesson` entities, ordered.
    *   `_category_.json`: Metadata file for Docusaurus sidebar.

### Lesson
*   **Description**: The smallest unit of content, focusing on a specific learning objective, with support for multiple complexity levels.
*   **Attributes**:
    *   `id`: String (e.g., "lesson-1.1")
    *   `title`: String (e.g., "What is ROS 2?")
    *   `description`: String (short summary of the lesson)
    *   `content`: A container for different versions of the lesson content:
        *   `Beginner`: Markdown/MDX content tailored for beginners.
        *   `Intermediate`: Markdown/MDX content with more technical depth.
        *   `Expert`: Markdown/MDX content with advanced concepts and code.
    *   `position`: Integer (order within its `Chapter`)

### UserPreference
*   **Description**: Stores the user's selected preferences, persisted in the browser.
*   **Attributes**:
    *   `skillLevel`: Enum (Beginner, Intermediate, Expert) - stored in `localStorage`.
    *   `language`: Enum (en, ur) - handled by Docusaurus i18n, state may be stored in URL path and/or `localStorage`.

## Relationships

*   A `Book` has one `Introduction` and many `Chapters`.
*   A `Chapter` belongs to one `Book` and has many `Lessons`.
*   A `Lesson` belongs to one `Chapter`.
*   Each `Lesson`'s content is conditionally rendered based on the `UserPreference.skillLevel`.
