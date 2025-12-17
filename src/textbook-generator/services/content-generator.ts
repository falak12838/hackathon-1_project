/**
 * ContentGenerator service for the Physical AI & Humanoid Robotics Textbook Generator
 * Generates textbook content with chapters, learning objectives, exercises, and quizzes
 */

import { Chapter, LearningObjective, Exercise, Quiz, QuizQuestion } from '../models/chapter';
import { TextbookConfig } from '../models/textbook-config';
import { LearningObjectiveModel } from '../models/learning-objective';
import { ExerciseModel } from '../models/exercise';
import { QuizModel } from '../models/quiz';
import { QuizQuestionModel } from '../models/quiz-question';
import { ModuleModel } from '../models/module';
import { TextbookConfigModel } from '../models/textbook-config';
import { ContentValidator } from './validator';
import { generateId, calculateReadingTime, toTitleCase, sanitizeText } from '../lib/utils';
import fs from 'fs';
import path from 'path';

export interface GeneratedTextbook {
  config: TextbookConfig;
  chapters: Chapter[];
  outputPath: string;
}

export interface GenerationResult {
  success: boolean;
  textbook?: GeneratedTextbook;
  errors: string[];
  warnings: string[];
}

export class ContentGenerator {
  private validator: ContentValidator;

  constructor() {
    this.validator = new ContentValidator();
  }

  /**
   * Generates a complete textbook based on the provided configuration
   */
  public async generateTextbook(config: TextbookConfig): Promise<GenerationResult> {
    const errors: string[] = [];
    const warnings: string[] = [];

    try {
      // Validate the configuration
      const configValidation = this.validator.validateTextbookConfig(config);
      if (!configValidation.isValid) {
        errors.push(...configValidation.issues);
        return {
          success: false,
          errors,
          warnings
        };
      }

      // If no modules are specified, generate default content
      if (!config.modules || config.modules.length === 0) {
        warnings.push('No modules specified, generating default content');
        config.modules = this.generateDefaultModules();
      }

      // Generate chapters for each module
      const allChapters: Chapter[] = [];
      for (const module of config.modules) {
        const moduleChapters = await this.generateChaptersForModule(module, config);
        allChapters.push(...moduleChapters);
      }

      // Sort chapters by order
      allChapters.sort((a, b) => a.order - b.order);

      // Validate content coherence across all chapters
      const coherenceValidation = this.validator.validateContentCoherence(allChapters);
      if (!coherenceValidation.isValid) {
        warnings.push(...coherenceValidation.issues);
      }

      const result: GeneratedTextbook = {
        config,
        chapters: allChapters,
        outputPath: './output'
      };

      return {
        success: true,
        textbook: result,
        errors: [],
        warnings
      };
    } catch (error) {
      errors.push(error instanceof Error ? error.message : String(error));
      return {
        success: false,
        errors,
        warnings
      };
    }
  }

  /**
   * Generates output in the specified format
   */
  public async generateOutput(generatedTextbook: GeneratedTextbook, format: string, outputPath: string): Promise<void> {
    switch (format.toLowerCase()) {
      case 'html':
        await this.generateHtmlOutput(generatedTextbook, outputPath);
        break;
      case 'markdown':
      case 'md':
        await this.generateMarkdownOutput(generatedTextbook, outputPath);
        break;
      case 'pdf':
        await this.generatePdfOutput(generatedTextbook, outputPath);
        break;
      default:
        throw new Error(`Unsupported output format: ${format}`);
    }
  }

  /**
   * Generates HTML output for the textbook
   */
  private async generateHtmlOutput(generatedTextbook: GeneratedTextbook, outputPath: string): Promise<void> {
    const { config, chapters } = generatedTextbook;

    // Create main index.html
    const indexContent = `
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>${config.title}</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; line-height: 1.6; }
        h1, h2, h3 { color: #333; }
        .chapter { margin-bottom: 40px; }
        .learning-objectives { background-color: #f0f8ff; padding: 10px; border-left: 4px solid #007acc; }
        .exercise { background-color: #fff8dc; padding: 10px; border-left: 4px solid #ffa500; }
        .quiz { background-color: #f0fff0; padding: 10px; border-left: 4px solid #32cd32; }
    </style>
</head>
<body>
    <header>
        <h1>${config.title}</h1>
        <p>By: ${config.author}</p>
        <p>Target Audience: ${config.targetAudience}</p>
        <p>Difficulty Level: ${config.difficultyLevel}</p>
    </header>

    <nav>
        <h2>Table of Contents</h2>
        <ul>
            ${chapters.map(ch => `<li><a href="#chapter-${ch.id}">${ch.name}</a></li>`).join('\n            ')}
        </ul>
    </nav>

    <main>
        ${chapters.map(ch => this.generateHtmlChapter(ch)).join('\n        ')}
    </main>

    <footer>
        <p>&copy; ${new Date().getFullYear()} ${config.author}. All rights reserved.</p>
    </footer>
</body>
</html>`;

    const indexPath = path.join(outputPath, 'index.html');
    fs.writeFileSync(indexPath, indexContent);

    // Create individual chapter files
    for (const chapter of chapters) {
      const chapterContent = `
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>${chapter.name} - ${config.title}</title>
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; line-height: 1.6; }
        h1, h2, h3 { color: #333; }
        .learning-objectives { background-color: #f0f8ff; padding: 10px; border-left: 4px solid #007acc; }
        .exercise { background-color: #fff8dc; padding: 10px; border-left: 4px solid #ffa500; }
        .quiz { background-color: #f0fff0; padding: 10px; border-left: 4px solid #32cd32; }
    </style>
</head>
<body>
    <header>
        <h1>${chapter.name}</h1>
        <p>Chapter ${chapter.order}</p>
    </header>

    <main>
        ${this.generateHtmlChapter(chapter)}
    </main>

    <footer>
        <p>&copy; ${new Date().getFullYear()} ${config.author}. All rights reserved.</p>
    </footer>
</body>
</html>`;

      const chapterPath = path.join(outputPath, `chapter-${chapter.id}.html`);
      fs.writeFileSync(chapterPath, chapterContent);
    }
  }

  /**
   * Generates HTML for a single chapter
   */
  private generateHtmlChapter(chapter: Chapter): string {
    let html = `<section id="chapter-${chapter.id}" class="chapter">\n`;
    html += `    <h2>${chapter.name}</h2>\n`;

    // Add estimated reading time
    if (chapter.metadata.estimatedReadingTime) {
      html += `    <p><em>Estimated reading time: ${chapter.metadata.estimatedReadingTime} minutes</em></p>\n`;
    }

    html += `    <div class="content">${chapter.content}</div>\n`;

    // Add learning objectives
    if (chapter.learningObjectives.length > 0) {
      html += `    <div class="learning-objectives">\n`;
      html += `        <h3>Learning Objectives</h3>\n`;
      html += `        <ul>\n`;
      for (const objective of chapter.learningObjectives) {
        html += `            <li>${objective.description}</li>\n`;
      }
      html += `        </ul>\n`;
      html += `    </div>\n`;
    }

    // Add exercises
    if (chapter.exercises.length > 0) {
      html += `    <div class="exercise">\n`;
      html += `        <h3>Exercises</h3>\n`;
      for (const exercise of chapter.exercises) {
        html += `        <div class="exercise-item">\n`;
        html += `            <h4>${exercise.title} (${exercise.difficulty})</h4>\n`;
        html += `            <p>${exercise.description}</p>\n`;
        if (exercise.type === 'coding') {
          html += `            <p><strong>Type:</strong> Coding Exercise</p>\n`;
        }
        html += `        </div>\n`;
      }
      html += `    </div>\n`;
    }

    // Add quizzes
    if (chapter.quizzes.length > 0) {
      html += `    <div class="quiz">\n`;
      html += `        <h3>Quizzes</h3>\n`;
      for (const quiz of chapter.quizzes) {
        html += `        <div class="quiz-item">\n`;
        html += `            <h4>${quiz.title}</h4>\n`;
        if (quiz.timeLimit) {
          html += `            <p><em>Time limit: ${quiz.timeLimit} minutes</em></p>\n`;
        }

        for (const question of quiz.questions) {
          html += `            <div class="question">\n`;
          html += `                <p><strong>Q:</strong> ${question.question}</p>\n`;
          if (question.options && question.options.length > 0) {
            html += `                <ul>\n`;
            for (const option of question.options) {
              html += `                    <li>${option}</li>\n`;
            }
            html += `                </ul>\n`;
          }
          html += `                <p><strong>Answer:</strong> ${question.correctAnswer}</p>\n`;
          html += `                <p><strong>Explanation:</strong> ${question.explanation}</p>\n`;
          html += `            </div>\n`;
        }
        html += `        </div>\n`;
      }
      html += `    </div>\n`;
    }

    html += `</section>`;

    return html;
  }

  /**
   * Generates Markdown output for the textbook
   */
  private async generateMarkdownOutput(generatedTextbook: GeneratedTextbook, outputPath: string): Promise<void> {
    const { config, chapters } = generatedTextbook;

    // Create main README.md
    let content = `# ${config.title}\n\n`;
    content += `**Author:** ${config.author}\n\n`;
    content += `**Target Audience:** ${config.targetAudience}\n\n`;
    content += `**Difficulty Level:** ${config.difficultyLevel}\n\n`;

    // Table of contents
    content += `## Table of Contents\n\n`;
    for (const chapter of chapters) {
      content += `${chapter.order}. [${chapter.name}](./chapter-${chapter.order.toString().padStart(2, '0')}.md)\n`;
    }
    content += '\n';

    const readmePath = path.join(outputPath, 'README.md');
    fs.writeFileSync(readmePath, content);

    // Create individual chapter files
    for (const chapter of chapters) {
      const chapterContent = this.generateMarkdownChapter(chapter);
      const chapterPath = path.join(outputPath, `chapter-${chapter.order.toString().padStart(2, '0')}.md`);
      fs.writeFileSync(chapterPath, chapterContent);
    }
  }

  /**
   * Generates Markdown for a single chapter
   */
  private generateMarkdownChapter(chapter: Chapter): string {
    let md = `# ${chapter.name}\n\n`;

    // Add estimated reading time
    if (chapter.metadata.estimatedReadingTime) {
      md += `*Estimated reading time: ${chapter.metadata.estimatedReadingTime} minutes*\n\n`;
    }

    md += `${chapter.content}\n\n`;

    // Add learning objectives
    if (chapter.learningObjectives.length > 0) {
      md += `## Learning Objectives\n\n`;
      for (const objective of chapter.learningObjectives) {
        md += `- ${objective.description}\n`;
      }
      md += '\n';
    }

    // Add exercises
    if (chapter.exercises.length > 0) {
      md += `## Exercises\n\n`;
      for (const exercise of chapter.exercises) {
        md += `### ${exercise.title} (${exercise.difficulty})\n\n`;
        md += `${exercise.description}\n\n`;
        if (exercise.type === 'coding') {
          md += `**Type:** Coding Exercise\n\n`;
        }
        md += '\n';
      }
    }

    // Add quizzes
    if (chapter.quizzes.length > 0) {
      md += `## Quizzes\n\n`;
      for (const quiz of chapter.quizzes) {
        md += `### ${quiz.title}\n\n`;
        if (quiz.timeLimit) {
          md += `*Time limit: ${quiz.timeLimit} minutes*\n\n`;
        }

        for (const question of quiz.questions) {
          md += `#### ${question.question}\n\n`;
          if (question.options && question.options.length > 0) {
            for (const option of question.options) {
              md += `- ${option}\n`;
            }
            md += '\n';
          }
          md += `**Answer:** ${question.correctAnswer}\n\n`;
          md += `**Explanation:** ${question.explanation}\n\n`;
        }
      }
    }

    return md;
  }

  /**
   * Generates PDF output for the textbook (placeholder - requires additional libraries)
   */
  private async generatePdfOutput(generatedTextbook: GeneratedTextbook, outputPath: string): Promise<void> {
    // This is a placeholder - in a real implementation, you would use a library like Puppeteer or PDFKit
    // For now, we'll just create a text file with a note
    const pdfPlaceholderPath = path.join(outputPath, 'textbook-pdf-generation.txt');
    fs.writeFileSync(pdfPlaceholderPath,
      'PDF generation would be implemented using a library like Puppeteer or PDFKit.\n\n' +
      'This requires additional dependencies:\n' +
      '- Puppeteer: npm install puppeteer\n' +
      '- PDFKit: npm install pdfkit\n\n' +
      'The implementation would convert HTML or Markdown to PDF format.'
    );
  }

  /**
   * Generates chapters for a module
   */
  private async generateChaptersForModule(module: any, config: TextbookConfig): Promise<Chapter[]> {
    // In a real implementation, this would use AI or templates to generate actual content
    // For now, we'll create placeholder content based on the module
    const chapters: Chapter[] = [];

    // If the module has predefined chapters, use them
    if (module.chapters && module.chapters.length > 0) {
      for (let i = 0; i < module.chapters.length; i++) {
        const moduleChapter = module.chapters[i];
        const chapter: Chapter = {
          id: moduleChapter.id || generateId('ch'),
          name: moduleChapter.name || `Chapter ${i + 1}`,
          content: moduleChapter.content || this.generatePlaceholderContent(module.name, i + 1),
          order: moduleChapter.order !== undefined ? moduleChapter.order : i + 1,
          learningObjectives: moduleChapter.learningObjectives || this.generatePlaceholderLearningObjectives(module.name),
          exercises: moduleChapter.exercises || this.generatePlaceholderExercises(module.name),
          quizzes: moduleChapter.quizzes || this.generatePlaceholderQuizzes(module.name),
          metadata: {
            difficultyLevel: config.difficultyLevel,
            estimatedReadingTime: calculateReadingTime(moduleChapter.content || ''),
            keywords: moduleChapter.metadata?.keywords || [module.name],
            prerequisites: moduleChapter.metadata?.prerequisites || []
          }
        };
        chapters.push(chapter);
      }
    } else {
      // If no chapters defined, create a default chapter for the module
      const chapter: Chapter = {
        id: generateId('ch'),
        name: module.name || 'Default Chapter',
        content: this.generatePlaceholderContent(module.name, 1),
        order: module.order || 1,
        learningObjectives: this.generatePlaceholderLearningObjectives(module.name),
        exercises: this.generatePlaceholderExercises(module.name),
        quizzes: this.generatePlaceholderQuizzes(module.name),
        metadata: {
          difficultyLevel: config.difficultyLevel,
          estimatedReadingTime: calculateReadingTime(this.generatePlaceholderContent(module.name, 1)),
          keywords: [module.name],
          prerequisites: module.prerequisites || []
        }
      };
      chapters.push(chapter);
    }

    return chapters;
  }

  /**
   * Generates placeholder content for a chapter
   */
  private generatePlaceholderContent(moduleName: string, chapterNumber: number): string {
    return `# ${toTitleCase(moduleName)} - Chapter ${chapterNumber}\n\n` +
      `This chapter covers the fundamentals of ${moduleName}. ` +
      `Students will learn about key concepts, principles, and applications.\n\n` +
      `## Introduction\n` +
      `The field of ${moduleName} has seen significant developments in recent years. ` +
      `This chapter provides an overview of the core principles and establishes a foundation for more advanced topics.\n\n` +
      `## Main Content\n` +
      `In this section, we explore the primary concepts related to ${moduleName}. ` +
      `We'll examine various approaches, methodologies, and best practices that are essential for understanding this domain.\n\n` +
      `## Conclusion\n` +
      `This chapter has introduced the fundamental concepts of ${moduleName}. ` +
      `The next chapter will build upon these foundations to explore more advanced topics.`;
  }

  /**
   * Generates placeholder learning objectives
   */
  private generatePlaceholderLearningObjectives(moduleName: string): LearningObjective[] {
    return [
      {
        id: generateId('lo'),
        chapterId: 'placeholder',
        description: `Define key concepts related to ${moduleName}`,
        measurable: true,
        tags: [moduleName, 'fundamentals']
      },
      {
        id: generateId('lo'),
        chapterId: 'placeholder',
        description: `Explain the principles underlying ${moduleName}`,
        measurable: true,
        tags: [moduleName, 'principles']
      },
      {
        id: generateId('lo'),
        chapterId: 'placeholder',
        description: `Apply ${moduleName} concepts to practical scenarios`,
        measurable: true,
        tags: [moduleName, 'application']
      }
    ];
  }

  /**
   * Generates placeholder exercises
   */
  private generatePlaceholderExercises(moduleName: string): Exercise[] {
    return [
      {
        id: generateId('ex'),
        chapterId: 'placeholder',
        title: `Conceptual Questions on ${moduleName}`,
        description: `Answer the following conceptual questions about ${moduleName} to test your understanding of the fundamental concepts.`,
        difficulty: 'beginner',
        solution: `Solutions would include explanations of core concepts related to ${moduleName}.`,
        type: 'theoretical'
      },
      {
        id: generateId('ex'),
        chapterId: 'placeholder',
        title: `Application Exercise`,
        description: `Apply the concepts learned in this chapter to solve a practical problem related to ${moduleName}.`,
        difficulty: 'intermediate',
        solution: `Detailed solution showing how to apply ${moduleName} concepts to solve the problem.`,
        type: 'practical'
      }
    ];
  }

  /**
   * Generates placeholder quizzes
   */
  private generatePlaceholderQuizzes(moduleName: string): Quiz[] {
    const questions: QuizQuestion[] = [
      {
        id: generateId('qq'),
        quizId: 'placeholder',
        question: `What is the fundamental principle of ${moduleName}?`,
        options: [
          `Principle A`,
          `Principle B`,
          `Principle C`,
          `Principle D`
        ],
        correctAnswer: `Principle B`,
        explanation: `The fundamental principle of ${moduleName} is Principle B, which underlies all other concepts in this field.`,
        type: 'multiple-choice'
      },
      {
        id: generateId('qq'),
        quizId: 'placeholder',
        question: `True or False: ${moduleName} concepts are only applicable in theoretical scenarios.`,
        options: [`True`, `False`],
        correctAnswer: `False`,
        explanation: `${moduleName} concepts have practical applications in various real-world scenarios.`,
        type: 'true-false'
      }
    ];

    return [
      {
        id: generateId('quiz'),
        chapterId: 'placeholder',
        title: `Chapter Quiz: ${moduleName}`,
        questions,
        timeLimit: 15
      }
    ];
  }

  /**
   * Generates default modules if none are provided
   */
  private generateDefaultModules(): any[] {
    return [
      {
        id: generateId('mod'),
        name: 'Introduction to Physical AI',
        description: 'Basic concepts of Physical AI and its applications',
        chapters: [],
        order: 1,
        prerequisites: []
      },
      {
        id: generateId('mod'),
        name: 'Humanoid Robotics Fundamentals',
        description: 'Core principles of humanoid robotics',
        chapters: [],
        order: 2,
        prerequisites: ['Introduction to Physical AI']
      },
      {
        id: generateId('mod'),
        name: 'Sensing and Perception',
        description: 'How humanoid robots perceive their environment',
        chapters: [],
        order: 3,
        prerequisites: ['Humanoid Robotics Fundamentals']
      }
    ];
  }
}