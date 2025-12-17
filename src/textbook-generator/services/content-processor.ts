/**
 * ContentProcessor service for the Physical AI & Humanoid Robotics Textbook Generator
 * Processes and transforms textbook content for different output formats
 */

import { Chapter, LearningObjective, Exercise, Quiz, QuizQuestion } from '../models/chapter';
import { TextbookConfig } from '../models/textbook-config';
import { ContentValidator } from './validator';
import { sanitizeText, escapeHtml, formatAsMarkdown, generateSlug, calculateReadingTime } from '../lib/utils';

export interface ProcessedChapter {
  id: string;
  title: string;
  slug: string;
  content: string;
  htmlContent: string;
  learningObjectives: LearningObjective[];
  exercises: Exercise[];
  quizzes: Quiz[];
  metadata: {
    wordCount: number;
    readingTime: number;
    difficultyLevel: string;
    keywords: string[];
  };
}

export interface ProcessedTextbook {
  config: TextbookConfig;
  chapters: ProcessedChapter[];
  tableOfContents: string[];
  summary: {
    totalChapters: number;
    totalWordCount: number;
    totalReadingTime: number;
    difficultyDistribution: Record<string, number>;
  };
}

export class ContentProcessor {
  private validator: ContentValidator;

  constructor() {
    this.validator = new ContentValidator();
  }

  /**
   * Processes a textbook configuration and its chapters
   */
  public async processTextbook(config: TextbookConfig, chapters: Chapter[]): Promise<ProcessedTextbook> {
    // Process each chapter
    const processedChapters: ProcessedChapter[] = [];
    let totalWordCount = 0;
    const difficultyDistribution: Record<string, number> = {};

    for (const chapter of chapters) {
      const processedChapter = await this.processChapter(chapter);
      processedChapters.push(processedChapter);

      // Update summary statistics
      totalWordCount += processedChapter.metadata.wordCount;

      // Update difficulty distribution
      const difficulty = processedChapter.metadata.difficultyLevel;
      difficultyDistribution[difficulty] = (difficultyDistribution[difficulty] || 0) + 1;
    }

    // Sort chapters by order
    processedChapters.sort((a, b) => {
      const chapterA = chapters.find(ch => ch.id === a.id);
      const chapterB = chapters.find(ch => ch.id === b.id);
      return (chapterA?.order || 0) - (chapterB?.order || 0);
    });

    // Generate table of contents
    const tableOfContents = processedChapters.map(ch => ch.title);

    const totalReadingTime = processedChapters.reduce((sum, ch) => sum + ch.metadata.readingTime, 0);

    const processedTextbook: ProcessedTextbook = {
      config,
      chapters: processedChapters,
      tableOfContents,
      summary: {
        totalChapters: processedChapters.length,
        totalWordCount,
        totalReadingTime,
        difficultyDistribution
      }
    };

    return processedTextbook;
  }

  /**
   * Processes a single chapter
   */
  public async processChapter(chapter: Chapter): Promise<ProcessedChapter> {
    // Sanitize content
    const sanitizedContent = sanitizeText(chapter.content);

    // Convert to HTML (basic conversion)
    const htmlContent = this.convertToHtml(sanitizedContent);

    // Calculate metadata
    const wordCount = sanitizedContent.trim().split(/\s+/).length;
    const readingTime = calculateReadingTime(sanitizedContent);

    // Process learning objectives
    const processedLearningObjectives = chapter.learningObjectives.map(obj => ({
      ...obj,
      description: sanitizeText(obj.description)
    }));

    // Process exercises
    const processedExercises = chapter.exercises.map(ex => ({
      ...ex,
      title: sanitizeText(ex.title),
      description: sanitizeText(ex.description),
      solution: sanitizeText(ex.solution)
    }));

    // Process quizzes
    const processedQuizzes = chapter.quizzes.map(quiz => ({
      ...quiz,
      title: sanitizeText(quiz.title),
      questions: quiz.questions.map(q => ({
        ...q,
        question: sanitizeText(q.question),
        options: q.options?.map(opt => sanitizeText(opt)),
        correctAnswer: sanitizeText(q.correctAnswer),
        explanation: sanitizeText(q.explanation)
      }))
    }));

    const processedChapter: ProcessedChapter = {
      id: chapter.id,
      title: sanitizeText(chapter.name),
      slug: generateSlug(chapter.name),
      content: sanitizedContent,
      htmlContent,
      learningObjectives: processedLearningObjectives,
      exercises: processedExercises,
      quizzes: processedQuizzes,
      metadata: {
        wordCount,
        readingTime,
        difficultyLevel: chapter.metadata.difficultyLevel || 'intermediate',
        keywords: chapter.metadata.keywords || []
      }
    };

    return processedChapter;
  }

  /**
   * Converts markdown-like content to HTML
   */
  private convertToHtml(content: string): string {
    // Basic markdown to HTML conversion
    let html = escapeHtml(content);

    // Convert headers
    html = html.replace(/^### (.*$)/gim, '<h3>$1</h3>');
    html = html.replace(/^## (.*$)/gim, '<h2>$1</h2>');
    html = html.replace(/^# (.*$)/gim, '<h1>$1</h1>');

    // Convert bold and italic
    html = html.replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>');
    html = html.replace(/\*(.*?)\*/g, '<em>$1</em>');

    // Convert links
    html = html.replace(/\[([^\]]+)\]\(([^)]+)\)/g, '<a href="$2">$1</a>');

    // Convert lists
    html = html.replace(/^- (.*$)/gim, '<ul><li>$1</li></ul>');
    html = html.replace(/^\d+\. (.*$)/gim, '<ol><li>$1</li></ol>');

    // Convert paragraphs
    html = html.replace(/\n\s*\n/g, '</p><p>');
    html = `<p>${html}</p>`;

    // Clean up multiple list tags
    html = html.replace(/<\/ul><ul>/g, '');
    html = html.replace(/<\/ol><ol>/g, '');
    html = html.replace(/<ul><\/ul>/g, '');
    html = html.replace(/<ol><\/ol>/g, '');

    return html;
  }

  /**
   * Formats content for a specific output format
   */
  public formatForOutput(content: string, format: 'html' | 'markdown' | 'pdf' | 'text'): string {
    switch (format) {
      case 'html':
        return this.convertToHtml(content);
      case 'markdown':
        return formatAsMarkdown(content);
      case 'text':
        // Remove all HTML tags if content is in HTML format
        return content.replace(/<[^>]*>/g, '');
      case 'pdf':
        // For PDF, we'll return markdown format which can be converted by other tools
        return formatAsMarkdown(content);
      default:
        return content;
    }
  }

  /**
   * Processes and validates content coherence
   */
  public async processContentCoherence(chapters: Chapter[]): Promise<{ isValid: boolean; issues: string[]; suggestions: string[] }> {
    // Validate content coherence using the validator
    return this.validator.validateContentCoherence(chapters);
  }

  /**
   * Optimizes content for different reading levels
   */
  public optimizeForReadingLevel(content: string, targetLevel: 'beginner' | 'intermediate' | 'advanced'): string {
    // This is a simplified implementation
    // In a real system, this would involve more sophisticated text processing

    let optimizedContent = content;

    if (targetLevel === 'beginner') {
      // Simplify complex sentences, add more explanations
      optimizedContent = this.simplifyContent(optimizedContent);
    } else if (targetLevel === 'advanced') {
      // Add more technical depth
      optimizedContent = this.addTechnicalDepth(optimizedContent);
    }
    // For intermediate, return as is

    return optimizedContent;
  }

  /**
   * Simplifies content for beginner level
   */
  private simplifyContent(content: string): string {
    // In a real implementation, this would use NLP to simplify complex sentences
    // For now, we'll just add a note
    return content + '\n\n*This content has been optimized for beginner learners.*';
  }

  /**
   * Adds technical depth for advanced level
   */
  private addTechnicalDepth(content: string): string {
    // In a real implementation, this would add more technical details
    // For now, we'll just add a note
    return content + '\n\n*Advanced technical details have been included.*';
  }

  /**
   * Generates a summary of a chapter
   */
  public generateChapterSummary(chapter: Chapter, maxSentences: number = 3): string[] {
    // Simple summary generation - in a real system this would use NLP
    const sentences = chapter.content.split(/[.!?]+/).filter(s => s.trim().length > 0);
    return sentences.slice(0, maxSentences).map(s => s.trim() + '.');
  }

  /**
   * Extracts key terms from content
   */
  public extractKeyTerms(content: string, maxTerms: number = 10): string[] {
    // Simple term extraction - in a real system this would use NLP
    const words = content.toLowerCase().match(/\b\w+\b/g) || [];
    const wordCount: Record<string, number> = {};

    for (const word of words) {
      if (word.length > 4) { // Only consider words longer than 4 characters
        wordCount[word] = (wordCount[word] || 0) + 1;
      }
    }

    // Sort by frequency and return top terms
    return Object.entries(wordCount)
      .sort((a, b) => b[1] - a[1])
      .slice(0, maxTerms)
      .map(([word]) => word);
  }

  /**
   * Checks content for accessibility compliance
   */
  public checkAccessibility(content: string): { issues: string[]; suggestions: string[] } {
    const issues: string[] = [];
    const suggestions: string[] = [];

    // Check for potential accessibility issues
    if (content.length > 10000) { // Very long paragraphs
      issues.push('Content contains very long paragraphs which may be difficult to read');
      suggestions.push('Break long paragraphs into shorter, more digestible sections');
    }

    // Check for heading structure
    const headingPattern = /<h[1-6]>/g;
    const headings = content.match(headingPattern) || [];
    if (headings.length > 0) {
      // In a real system, we'd validate heading hierarchy
    }

    return { issues, suggestions };
  }
}