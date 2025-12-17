/**
 * TextbookConfig model for the Physical AI & Humanoid Robotics Textbook Generator
 * Represents parameters that control textbook generation (audience level, depth, output format, etc.)
 */

import { ModuleModel, Module } from './module';

export interface TextbookConfig {
  id: string;
  title: string;
  author: string;
  targetAudience: 'undergraduate' | 'graduate' | 'professional' | 'high-school';
  difficultyLevel: 'beginner' | 'intermediate' | 'advanced';
  modules: Module[];
  outputFormats: string[]; // e.g., ['html', 'markdown', 'pdf']
  customizationOptions?: {
    includeSolutions?: boolean;
    generateQuizzes?: boolean;
    addExercises?: boolean;
    includeLearningObjectives?: boolean;
    estimatedReadingTime?: boolean;
  };
}

export class TextbookConfigModel {
  public id: string;
  public title: string;
  public author: string;
  public targetAudience: 'undergraduate' | 'graduate' | 'professional' | 'high-school';
  public difficultyLevel: 'beginner' | 'intermediate' | 'advanced';
  public modules: Module[];
  public outputFormats: string[];
  public customizationOptions: {
    includeSolutions?: boolean;
    generateQuizzes?: boolean;
    addExercises?: boolean;
    includeLearningObjectives?: boolean;
    estimatedReadingTime?: boolean;
  };

  constructor(data: Partial<TextbookConfig> = {}) {
    this.id = data.id || this.generateId();
    this.title = data.title || 'Physical AI & Humanoid Robotics Textbook';
    this.author = data.author || 'Generated';
    this.targetAudience = data.targetAudience || 'undergraduate';
    this.difficultyLevel = data.difficultyLevel || 'intermediate';
    this.modules = data.modules || [];
    this.outputFormats = data.outputFormats || ['html', 'markdown'];
    this.customizationOptions = data.customizationOptions || {};
  }

  private generateId(): string {
    return `config_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  public validate(): { isValid: boolean; errors: string[] } {
    const errors: string[] = [];

    if (!this.title.trim()) {
      errors.push('Textbook title is required');
    }

    if (!this.author.trim()) {
      errors.push('Author is required');
    }

    if (this.modules.length === 0) {
      errors.push('Textbook configuration must specify at least one module');
    }

    if (!['undergraduate', 'graduate', 'professional', 'high-school'].includes(this.targetAudience)) {
      errors.push('Target audience must be one of: undergraduate, graduate, professional, high-school');
    }

    if (!['beginner', 'intermediate', 'advanced'].includes(this.difficultyLevel)) {
      errors.push('Difficulty level must be one of: beginner, intermediate, advanced');
    }

    if (this.outputFormats.length === 0) {
      errors.push('At least one output format must be specified');
    }

    // Validate each module
    for (let i = 0; i < this.modules.length; i++) {
      const module = this.modules[i];
      if (module) {
        const moduleValidation = new ModuleModel(module).validate();
        if (!moduleValidation.isValid) {
          errors.push(`Module ${i + 1} validation failed: ${moduleValidation.errors.join(', ')}`);
        }
      }
    }

    return {
      isValid: errors.length === 0,
      errors
    };
  }
}