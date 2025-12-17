# Quickstart Guide: Physical AI & Humanoid Robotics Textbook Generator

## Prerequisites

- Node.js 20+ installed
- npm or yarn package manager
- Git (for version control)

## Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd <repository-name>
```

2. Install dependencies:
```bash
npm install
# or
yarn install
```

## Basic Usage

1. Create a textbook configuration file (e.g., `config.json`):
```json
{
  "title": "Physical AI & Humanoid Robotics",
  "author": "Your Name",
  "targetAudience": "undergraduate",
  "difficultyLevel": "intermediate",
  "outputFormats": ["html", "markdown"],
  "modules": [
    {
      "id": "introduction",
      "name": "Introduction to Physical AI",
      "description": "Basic concepts of Physical AI"
    }
  ]
}
```

2. Generate the textbook:
```bash
npm run textbook:generate -- --config config.json
# or
yarn textbook:generate --config config.json
```

3. The generated textbook will be available in the `output/` directory.

## Development

1. To run the development server for the textbook:
```bash
npm run textbook:dev
```

2. To build the textbook for production:
```bash
npm run textbook:build
```

3. To serve the built textbook:
```bash
npm run textbook:serve
```

## Adding New Content

1. Create a new module directory in `src/content/modules/`
2. Add chapter files in the module directory
3. Update your configuration to include the new module
4. Regenerate the textbook

## Output Formats

The system supports multiple output formats:
- HTML (for web viewing)
- Markdown (for documentation systems)
- Static site (using Docusaurus)

## Configuration Options

- `title`: Title of the textbook
- `author`: Author name
- `targetAudience`: Intended audience level
- `difficultyLevel`: Overall difficulty
- `outputFormats`: List of desired output formats
- `modules`: List of modules to include