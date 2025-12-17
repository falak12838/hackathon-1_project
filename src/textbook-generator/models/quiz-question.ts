/**
 * QuizQuestion model for the Physical AI & Humanoid Robotics Textbook Generator
 * Represents a single question within a quiz
 */

export interface QuizQuestion {
  id: string;
  quizId: string;
  question: string;
  options?: string[];
  correctAnswer: string;
  explanation: string;
  type: 'multiple-choice' | 'true-false' | 'short-answer';
}

export class QuizQuestionModel {
  public id: string;
  public quizId: string;
  public question: string;
  public options?: string[];
  public correctAnswer: string;
  public explanation: string;
  public type: 'multiple-choice' | 'true-false' | 'short-answer';

  constructor(data: Partial<QuizQuestion> = {}) {
    this.id = data.id || this.generateId();
    this.quizId = data.quizId || '';
    this.question = data.question || '';
    this.options = data.options;
    this.correctAnswer = data.correctAnswer || '';
    this.explanation = data.explanation || '';
    this.type = data.type || 'multiple-choice';
  }

  private generateId(): string {
    return `qq_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  public validate(): { isValid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (!this.question.trim()) {
      errors.push('Question content is required');
    }

    if (!this.correctAnswer.trim()) {
      errors.push('Correct answer is required');
    }

    if (!this.explanation.trim()) {
      errors.push('Explanation is required');
    }

    if (!['multiple-choice', 'true-false', 'short-answer'].includes(this.type)) {
      errors.push('Type must be one of: multiple-choice, true-false, short-answer');
    }

    if (this.type === 'multiple-choice' && (!this.options || this.options.length === 0)) {
      errors.push('Multiple choice questions must have options');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }
}