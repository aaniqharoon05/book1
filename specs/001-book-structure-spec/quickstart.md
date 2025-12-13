# Quickstart Guide

This guide provides the basic steps to set up and run the Docusaurus-based book project locally.

## Prerequisites

*   **Node.js**: Version 18.0 or higher.
*   **npm** or **yarn**: A Node.js package manager.

## Setup Instructions

1.  **Clone the Repository**:
    ```bash
    git clone <repository-url>
    cd book
    ```

2.  **Install Dependencies**:
    Navigate to the `book` directory and install the required Node.js packages.
    ```bash
    npm install
    ```
    or if you use yarn:
    ```bash
    yarn install
    ```

## Running the Development Server

1.  **Start the Docusaurus Site**:
    From within the `book` directory, run the following command to start the local development server:
    ```bash
    npm run start
    ```
    or
    ```bash
    yarn start
    ```

2.  **View the Site**:
    Open your web browser and navigate to `http://localhost:3000`. The site will automatically reload if you make changes to the source files.

## Running Language-Specific Versions

*   **To run the Urdu version**:
    ```bash
    npm run start -- --locale ur
    ```
    Navigate to `http://localhost:3000/ur/`.

## Building the Site for Production

To create a static production build of the site, run:
```bash
npm run build
```
The output will be placed in the `build` directory. This command builds all locales by default.
