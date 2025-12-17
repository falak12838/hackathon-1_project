/**
 * LearningObjective model for the Physical AI & Humanoid Robotics Textbook Generator
 * Represents a specific, measurable outcome that students should achieve after studying a chapter
 */

export interface LearningObjective {
  id: string;
  chapterId: string;
  description: string;
  measurable: boolean;
  tags: string[];
}

export class LearningObjectiveModel {
  public id: string;
  public chapterId: string;
  public description: string;
  public measurable: boolean;
  public tags: string[];

  constructor(data: Partial<LearningObjective> = {}) {
    this.id = data.id || this.generateId();
    this.chapterId = data.chapterId || '';
    this.description = data.description || '';
    this.measurable = data.measurable ?? true; // Default to true
    this.tags = data.tags || [];
  }

  private generateId(): string {
    return `lo_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  public validate(): { isValid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (!this.description.trim()) {
      errors.push('Learning objective description is required');
    }

    if (!this.chapterId) {
      errors.push('Chapter ID is required');
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }
}