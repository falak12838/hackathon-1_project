/**
 * Exercise model for the Physical AI & Humanoid Robotics Textbook Generator
 * Represents a practical activity designed to reinforce concepts from the chapter
 */

export interface Exercise {
  id: string;
  chapterId: string;
  title: string;
  description: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  solution: string;
  type: 'coding' | 'theoretical' | 'practical';
}

export class ExerciseModel {
  public id: string;
  public chapterId: string;
  public title: string;
  public description: string;
  public difficulty: 'beginner' | 'intermediate' | 'advanced';
  public solution: string;
  public type: 'coding' | 'theoretical' | 'practical';

  constructor(data: Partial<Exercise> = {}) {
    this.id = data.id || this.generateId();
    this.chapterId = data.chapterId || '';
    this.title = data.title || '';
    this.description = data.description || '';
    this.difficulty = data.difficulty || 'intermediate';
    this.solution = data.solution || '';
    this.type = data.type || 'theoretical';
  }

  private generateId(): string {
    return `ex_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  public validate(): { isValid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (!this.title.trim()) {
      errors.push('Exercise title is required');
    }

    if (!this.description.trim()) {
      errors.push('Exercise description is required');
    }

    if (!this.solution.trim()) {
      errors.push('Exercise solution is required');
    }

    if (!this.chapterId) {
      errors.push('Chapter ID is required');
    }

    if (!['beginner', 'intermediate', 'advanced'].includes(this.difficulty)) {
      errors.push('Difficulty must be one of: beginner, intermediate, advanced');
    }

    if (!['coding', 'theoretical', 'practical'].includes(this.type)) {
      errors.push('Type must be one of: coding, theoretical, practical');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }
}