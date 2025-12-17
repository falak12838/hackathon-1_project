/**
 * FormatConverter service for the Physical AI & Humanoid Robotics Textbook Generator
 * Converts textbook content to different output formats (HTML, Markdown, PDF)
 */

import { TextbookConfig } from '../models/textbook-config';
import { Chapter, LearningObjective, Exercise, Quiz, QuizQuestion } from '../models/chapter';
import { ProcessedTextbook, ProcessedChapter } from './content-processor';
import { escapeHtml, sanitizeText, generateSlug, calculateReadingTime } from '../lib/utils';
import fs from 'fs';
import path from 'path';

export type OutputFormat = 'html' | 'markdown' | 'md' | 'pdf' | 'json' | 'text';

export interface FormatConversionOptions {
  includeLearningObjectives?: boolean;
  includeExercises?: boolean;
  includeQuizzes?: boolean;
  includeSolutions?: boolean;
  includeMetadata?: boolean;
  title?: string;
  author?: string;
  stylesheet?: string; // For HTML output
  template?: string; // Custom template path
}

export interface ConversionResult {
  success: boolean;
  content?: string;
  filePath?: string;
  format: OutputFormat;
  errors: string[];
  warnings: string[];
}

export class FormatConverter {
  /**
   * Converts a processed textbook to the specified format
   */
  public async convert(
    processedTextbook: ProcessedTextbook,
    format: OutputFormat,
    options: FormatConversionOptions = {}
  ): Promise<ConversionResult> {
    const errors: string[] = [];
    const warnings: string[] = [];

    try {
      let content: string;

      switch (format.toLowerCase()) {
        case 'html':
          content = this.convertToHtml(processedTextbook, options);
          break;
        case 'markdown':
        case 'md':
          content = this.convertToMarkdown(processedTextbook, options);
          break;
        case 'pdf':
          // PDF generation would typically require additional libraries like Puppeteer or PDFKit
          content = this.convertToHtml(processedTextbook, options); // Generate HTML as intermediate step
          warnings.push('PDF generation requires additional libraries like Puppeteer or PDFKit');
          break;
        case 'json':
          content = JSON.stringify(processedTextbook, null, 2);
          break;
        case 'text':
          content = this.convertToText(processedTextbook, options);
          break;
        default:
          errors.push(`Unsupported output format: ${format}`);
          return {
            success: false,
            format,
            errors,
            warnings
          };
      }

      return {
        success: true,
        content,
        format,
        errors,
        warnings
      };
    } catch (error) {
      errors.push(`Error converting to ${format}: ${error instanceof Error ? error.message : String(error)}`);
      return {
        success: false,
        format,
        errors,
        warnings
      };
    }
  }

  /**
   * Converts textbook to HTML format
   */
  private convertToHtml(processedTextbook: ProcessedTextbook, options: FormatConversionOptions): string {
    const {
      includeLearningObjectives = true,
      includeExercises = true,
      includeQuizzes = true,
      includeSolutions = true,
      includeMetadata = true,
      title = processedTextbook.config.title,
      author = processedTextbook.config.author,
      stylesheet
    } = options;

    let html = `<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>${escapeHtml(title)}</title>`;

    if (stylesheet) {
      html += `\n    <link rel="stylesheet" href="${escapeHtml(stylesheet)}">`;
    } else {
      html += `
    <style>
        body { font-family: Arial, sans-serif; margin: 40px; line-height: 1.6; }
        h1, h2, h3, h4 { color: #333; }
        .chapter { margin-bottom: 40px; padding: 20px; border: 1px solid #ddd; border-radius: 5px; }
        .learning-objectives { background-color: #f0f8ff; padding: 15px; margin: 10px 0; border-left: 4px solid #007acc; }
        .exercise { background-color: #fff8dc; padding: 15px; margin: 10px 0; border-left: 4px solid #ffa500; }
        .quiz { background-color: #f0fff0; padding: 15px; margin: 10px 0; border-left: 4px solid #32cd32; }
        .solution { display: none; color: #666; font-style: italic; }
        .show-solutions .solution { display: block; }
        .metadata { font-size: 0.9em; color: #666; margin: 10px 0; }
        .toc { background-color: #f9f9f9; padding: 15px; margin: 20px 0; border-radius: 5px; }
        .toc ul { margin: 0; padding-left: 20px; }
    </style>`;
    }

    html += `
</head>
<body class="${includeSolutions ? 'show-solutions' : ''}">
    <header>
        <h1>${escapeHtml(title)}</h1>
        <p>By: ${escapeHtml(author)}</p>
        <p>Target Audience: ${escapeHtml(processedTextbook.config.targetAudience)}</p>
        <p>Difficulty Level: ${escapeHtml(processedTextbook.config.difficultyLevel)}</p>`;

    if (includeMetadata && processedTextbook.summary) {
      html += `
        <div class="metadata">
            <p>Total Chapters: ${processedTextbook.summary.totalChapters}</p>
            <p>Total Reading Time: ~${processedTextbook.summary.totalReadingTime} minutes</p>
        </div>`;
    }

    html += `
    </header>`;

    // Table of Contents
    html += `
    <div class="toc">
        <h2>Table of Contents</h2>
        <ul>
            ${processedTextbook.chapters.map(ch =>
              `<li><a href="#chapter-${escapeHtml(ch.id)}">${escapeHtml(ch.title)}</a></li>`
            ).join('\n            ')}
        </ul>
    </div>`;

    // Chapters
    html += `
    <main>`;

    for (const chapter of processedTextbook.chapters) {
      html += this.generateHtmlChapter(chapter, {
        ...options,
        includeLearningObjectives,
        includeExercises,
        includeQuizzes,
        includeSolutions
      });
    }

    html += `
    </main>`;

    html += `
    <footer>
        <p>&copy; ${new Date().getFullYear()} ${escapeHtml(author)}. All rights reserved.</p>
    </footer>
</body>
</html>`;

    return html;
  }

  /**
   * Generates HTML for a single chapter
   */
  private generateHtmlChapter(chapter: ProcessedChapter, options: FormatConversionOptions): string {
    const {
      includeLearningObjectives = true,
      includeExercises = true,
      includeQuizzes = true,
      includeSolutions = true
    } = options;

    let html = `<section id="chapter-${escapeHtml(chapter.id)}" class="chapter">\n`;
    html += `    <h2>${escapeHtml(chapter.title)}</h2>\n`;

    // Add metadata
    if (options.includeMetadata && chapter.metadata) {
      html += `    <div class="metadata">`;
      html += `Reading time: ~${chapter.metadata.readingTime} minutes`;
      if (chapter.metadata.difficultyLevel) {
        html += `, Difficulty: ${escapeHtml(chapter.metadata.difficultyLevel)}`;
      }
      html += `</div>\n`;
    }

    html += `    <div class="content">${chapter.htmlContent}</div>\n`;

    // Add learning objectives
    if (includeLearningObjectives && chapter.learningObjectives.length > 0) {
      html += `    <div class="learning-objectives">\n`;
      html += `        <h3>Learning Objectives</h3>\n`;
      html += `        <ul>\n`;
      for (const objective of chapter.learningObjectives) {
        html += `            <li>${escapeHtml(objective.description)}</li>\n`;
      }
      html += `        </ul>\n`;
      html += `    </div>\n`;
    }

    // Add exercises
    if (includeExercises && chapter.exercises.length > 0) {
      html += `    <div class="exercise">\n`;
      html += `        <h3>Exercises</h3>\n`;
      for (const exercise of chapter.exercises) {
        html += `        <div class="exercise-item">\n`;
        html += `            <h4>${escapeHtml(exercise.title)} (${escapeHtml(exercise.difficulty)})</h4>\n`;
        html += `            <p>${escapeHtml(exercise.description)}</p>\n`;
        if (includeSolutions && exercise.solution) {
          html += `            <div class="solution">`;
          html += `<strong>Solution:</strong> ${escapeHtml(exercise.solution)}`;
          html += `</div>\n`;
        }
        html += `        </div>\n`;
      }
      html += `    </div>\n`;
    }

    // Add quizzes
    if (includeQuizzes && chapter.quizzes.length > 0) {
      html += `    <div class="quiz">\n`;
      html += `        <h3>Quizzes</h3>\n`;
      for (const quiz of chapter.quizzes) {
        html += `        <div class="quiz-item">\n`;
        html += `            <h4>${escapeHtml(quiz.title)}</h4>\n`;
        if (quiz.timeLimit) {
          html += `            <p><em>Time limit: ${quiz.timeLimit} minutes</em></p>\n`;
        }

        for (const question of quiz.questions) {
          html += `            <div class="question">\n`;
          html += `                <p><strong>Q:</strong> ${escapeHtml(question.question)}</p>\n`;
          if (question.options && question.options.length > 0) {
            html += `                <ul>\n`;
            for (const option of question.options) {
              html += `                    <li>${escapeHtml(option)}</li>\n`;
            }
            html += `                </ul>\n`;
          }
          if (includeSolutions) {
            html += `                <p><strong>Answer:</strong> ${escapeHtml(question.correctAnswer)}</p>\n`;
            html += `                <p><strong>Explanation:</strong> ${escapeHtml(question.explanation)}</p>\n`;
          }
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
   * Converts textbook to Markdown format
   */
  private convertToMarkdown(processedTextbook: ProcessedTextbook, options: FormatConversionOptions): string {
    const {
      includeLearningObjectives = true,
      includeExercises = true,
      includeQuizzes = true,
      includeSolutions = true,
      includeMetadata = true,
      title = processedTextbook.config.title,
      author = processedTextbook.config.author
    } = options;

    let md = `# ${title}\n\n`;
    md += `**Author:** ${author}\n\n`;
    md += `**Target Audience:** ${processedTextbook.config.targetAudience}\n\n`;
    md += `**Difficulty Level:** ${processedTextbook.config.difficultyLevel}\n\n`;

    if (includeMetadata && processedTextbook.summary) {
      md += `**Total Chapters:** ${processedTextbook.summary.totalChapters}\n\n`;
      md += `**Estimated Reading Time:** ~${processedTextbook.summary.totalReadingTime} minutes\n\n`;
    }

    // Table of Contents
    md += `## Table of Contents\n\n`;
    for (const chapter of processedTextbook.chapters) {
      md += `- [${chapter.title}](#${generateSlug(chapter.title)})\n`;
    }
    md += '\n';

    // Chapters
    for (const chapter of processedTextbook.chapters) {
      md += this.generateMarkdownChapter(chapter, {
        ...options,
        includeLearningObjectives,
        includeExercises,
        includeQuizzes,
        includeSolutions
      });
    }

    return md;
  }

  /**
   * Generates Markdown for a single chapter
   */
  private generateMarkdownChapter(chapter: ProcessedChapter, options: FormatConversionOptions): string {
    const {
      includeLearningObjectives = true,
      includeExercises = true,
      includeQuizzes = true,
      includeSolutions = true
    } = options;

    let md = `# ${chapter.title}\n\n`;

    // Add metadata
    if (options.includeMetadata && chapter.metadata) {
      md += `*Reading time: ~${chapter.metadata.readingTime} minutes`;
      if (chapter.metadata.difficultyLevel) {
        md += `, Difficulty: ${chapter.metadata.difficultyLevel}`;
      }
      md += '*\n\n';
    }

    md += `${chapter.content}\n\n`;

    // Add learning objectives
    if (includeLearningObjectives && chapter.learningObjectives.length > 0) {
      md += `## Learning Objectives\n\n`;
      for (const objective of chapter.learningObjectives) {
        md += `- ${objective.description}\n`;
      }
      md += '\n';
    }

    // Add exercises
    if (includeExercises && chapter.exercises.length > 0) {
      md += `## Exercises\n\n`;
      for (const exercise of chapter.exercises) {
        md += `### ${exercise.title} (${exercise.difficulty})\n\n`;
        md += `${exercise.description}\n\n`;
        if (includeSolutions && exercise.solution) {
          md += `**Solution:** ${exercise.solution}\n\n`;
        }
      }
    }

    // Add quizzes
    if (includeQuizzes && chapter.quizzes.length > 0) {
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
          if (includeSolutions) {
            md += `**Answer:** ${question.correctAnswer}\n\n`;
            md += `**Explanation:** ${question.explanation}\n\n`;
          }
        }
      }
    }

    return md;
  }

  /**
   * Converts textbook to plain text format
   */
  private convertToText(processedTextbook: ProcessedTextbook, options: FormatConversionOptions): string {
    const {
      includeLearningObjectives = true,
      includeExercises = true,
      includeQuizzes = true,
      includeSolutions = true,
      title = processedTextbook.config.title,
      author = processedTextbook.config.author
    } = options;

    let text = `${title}\n`;
    text += `By: ${author}\n`;
    text += `Target Audience: ${processedTextbook.config.targetAudience}\n`;
    text += `Difficulty Level: ${processedTextbook.config.difficultyLevel}\n\n`;

    // Chapters
    for (const chapter of processedTextbook.chapters) {
      text += this.generateTextChapter(chapter, {
        ...options,
        includeLearningObjectives,
        includeExercises,
        includeQuizzes,
        includeSolutions
      });
      text += '\n\n';
    }

    return text;
  }

  /**
   * Generates plain text for a single chapter
   */
  private generateTextChapter(chapter: ProcessedChapter, options: FormatConversionOptions): string {
    const {
      includeLearningObjectives = true,
      includeExercises = true,
      includeQuizzes = true,
      includeSolutions = true
    } = options;

    let text = `=== ${chapter.title} ===\n\n`;

    text += `${chapter.content}\n\n`;

    // Add learning objectives
    if (includeLearningObjectives && chapter.learningObjectives.length > 0) {
      text += `Learning Objectives:\n`;
      for (const objective of chapter.learningObjectives) {
        text += `- ${objective.description}\n`;
      }
      text += '\n';
    }

    // Add exercises
    if (includeExercises && chapter.exercises.length > 0) {
      text += `Exercises:\n`;
      for (const exercise of chapter.exercises) {
        text += `- ${exercise.title} (${exercise.difficulty}): ${exercise.description}\n`;
        if (includeSolutions && exercise.solution) {
          text += `  Solution: ${exercise.solution}\n`;
        }
      }
      text += '\n';
    }

    // Add quizzes
    if (includeQuizzes && chapter.quizzes.length > 0) {
      text += `Quizzes:\n`;
      for (const quiz of chapter.quizzes) {
        text += `- ${quiz.title}\n`;
        if (quiz.timeLimit) {
          text += `  Time limit: ${quiz.timeLimit} minutes\n`;
        }

        for (const question of quiz.questions) {
          text += `  Q: ${question.question}\n`;
          if (question.options && question.options.length > 0) {
            text += `  Options: ${question.options.join(', ')}\n`;
          }
          if (includeSolutions) {
            text += `  Answer: ${question.correctAnswer}\n`;
            text += `  Explanation: ${question.explanation}\n`;
          }
        }
      }
      text += '\n';
    }

    return text;
  }

  /**
   * Saves converted content to a file
   */
  public async saveToFile(content: string, outputPath: string, format: OutputFormat): Promise<ConversionResult> {
    try {
      // Ensure output directory exists
      const directory = path.dirname(outputPath);
      if (!fs.existsSync(directory)) {
        fs.mkdirSync(directory, { recursive: true });
      }

      // Write content to file
      fs.writeFileSync(outputPath, content);

      return {
        success: true,
        filePath: outputPath,
        format,
        errors: [],
        warnings: []
      };
    } catch (error) {
      return {
        success: false,
        format,
        errors: [`Error saving file to ${outputPath}: ${error instanceof Error ? error.message : String(error)}`],
        warnings: []
      };
    }
  }

  /**
   * Converts and saves textbook to multiple formats
   */
  public async convertMultiple(
    processedTextbook: ProcessedTextbook,
    formats: OutputFormat[],
    outputDir: string,
    options: FormatConversionOptions = {}
  ): Promise<Record<OutputFormat, ConversionResult>> {
    const results: Record<OutputFormat, ConversionResult> = {} as Record<OutputFormat, ConversionResult>;

    for (const format of formats) {
      const conversionResult = await this.convert(processedTextbook, format, options);

      if (conversionResult.success && conversionResult.content) {
        const extension = this.getExtensionForFormat(format);
        const filename = `${sanitizeText(processedTextbook.config.title).replace(/\s+/g, '_')}.${extension}`;
        const filePath = path.join(outputDir, filename);

        const saveResult = await this.saveToFile(conversionResult.content, filePath, format);
        results[format] = saveResult;
      } else {
        results[format] = conversionResult;
      }
    }

    return results;
  }

  /**
   * Gets the file extension for a given format
   */
  private getExtensionForFormat(format: OutputFormat): string {
    switch (format.toLowerCase()) {
      case 'html':
        return 'html';
      case 'markdown':
      case 'md':
        return 'md';
      case 'pdf':
        return 'pdf';
      case 'json':
        return 'json';
      case 'text':
        return 'txt';
      default:
        return format;
    }
  }
}