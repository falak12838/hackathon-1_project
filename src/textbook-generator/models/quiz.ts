/**
 * Quiz model for the Physical AI & Humanoid Robotics Textbook Generator
 * Represents a set of questions that assess student understanding of the chapter content
 */

import { QuizQuestion } from './chapter';

export interface Quiz {
  id: string;
  chapterId: string;
  title: string;
  questions: QuizQuestion[];
  timeLimit?: number; // Time limit in minutes
}

export class QuizModel {
  public id: string;
  public chapterId: string;
  public title: string;
  public questions: QuizQuestion[];
  public timeLimit?: number;

  constructor(data: Partial<Quiz> = {}) {
    this.id = data.id || this.generateId();
    this.chapterId = data.chapterId || '';
    this.title = data.title || '';
    this.questions = data.questions || [];
    this.timeLimit = data.timeLimit;
  }

  private generateId(): string {
    return `quiz_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  public validate(): { isValid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (!this.title.trim()) {
      errors.push('Quiz title is required');
    }

    if (!this.chapterId) {
      errors.push('Chapter ID is required');
    }

    if (this.questions.length === 0) {
      errors.push('Quiz must have at least one question');
    } else {
      for (let i = 0; i < this.questions.length; i++) {
        const question = this.questions[i];
        if (!question.question.trim()) {
          errors.push(`Question ${i + 1} must have content`);
        }

        if (!question.correctAnswer.trim()) {
          errors.push(`Question ${i + 1} must have a correct answer`);
        }
      }
    }

    if (this.timeLimit !== undefined && this.timeLimit <= 0) {
      errors.push('Time limit must be a positive number');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }
}