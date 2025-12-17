/**
 * Content validation service for the Physical AI & Humanoid Robotics Textbook Generator
 * Validates content coherence and flags potential inconsistencies
 */

import { Chapter, LearningObjective, Exercise, Quiz, QuizQuestion } from '../models/chapter';
import { LearningObjectiveModel } from '../models/learning-objective';
import { ExerciseModel } from '../models/exercise';
import { QuizModel } from '../models/quiz';
import { QuizQuestionModel } from '../models/quiz-question';
import { ModuleModel } from '../models/module';
import { TextbookConfigModel } from '../models/textbook-config';

export interface ValidationResult {
  isValid: boolean;
  issues: string[];
  suggestions: string[];
}

export class ContentValidator {
  /**
   * Validates a chapter for content coherence and consistency
   */
  public validateChapter(chapter: Chapter): ValidationResult {
    const issues: string[] = [];
    const suggestions: string[] = [];

    // Check if learning objectives align with chapter content
    if (chapter.learningObjectives.length > 0 && !chapter.content.includes(chapter.learningObjectives[0].description.substring(0, 20))) {
      issues.push(`Learning objectives may not align with chapter content in chapter "${chapter.name}"`);
      suggestions.push(`Ensure chapter content covers the stated learning objectives`);
    }

    // Check if exercises reinforce chapter concepts
    for (const exercise of chapter.exercises) {
      if (!chapter.content.toLowerCase().includes(exercise.title.toLowerCase()) &&
          !chapter.content.toLowerCase().includes(exercise.description.substring(0, 30).toLowerCase())) {
        issues.push(`Exercise "${exercise.title}" may not reinforce chapter concepts`);
        suggestions.push(`Ensure exercises are directly related to chapter content`);
      }
    }

    // Check if quiz questions are based on chapter content
    for (const quiz of chapter.quizzes) {
      for (const question of quiz.questions) {
        if (!chapter.content.toLowerCase().includes(question.question.substring(0, 30).toLowerCase())) {
          issues.push(`Quiz question may not be based on chapter content`);
          suggestions.push(`Ensure quiz questions are directly related to chapter content`);
        }
      }
    }

    // Check for content coherence
    if (chapter.content.length < 100) {
      issues.push(`Chapter "${chapter.name}" content appears to be too brief`);
      suggestions.push(`Expand chapter content to provide comprehensive coverage of the topic`);
    }

    return {
      isValid: issues.length === 0,
      issues,
      suggestions
    };
  }

  /**
   * Validates a learning objective
   */
  public validateLearningObjective(learningObjective: LearningObjective): ValidationResult {
    const model = new LearningObjectiveModel(learningObjective);
    const validation = model.validate();

    if (!validation.isValid) {
      return {
        isValid: false,
        issues: validation.errors,
        suggestions: ['Ensure all required fields are properly filled']
      };
    }

    // Check if the objective is specific and measurable
    const nonSpecificWords = ['understand', 'know', 'learn about'];
    const lowerDesc = learningObjective.description.toLowerCase();

    for (const word of nonSpecificWords) {
      if (lowerDesc.includes(word)) {
        return {
          isValid: false,
          issues: [`Learning objective contains non-specific language: "${word}"`],
          suggestions: [`Use specific, measurable verbs like "identify", "explain", "demonstrate", "apply", etc.`]
        };
      }
    }

    return {
      isValid: true,
      issues: [],
      suggestions: []
    };
  }

  /**
   * Validates an exercise
   */
  public validateExercise(exercise: Exercise): ValidationResult {
    const model = new ExerciseModel(exercise);
    const validation = model.validate();

    if (!validation.isValid) {
      return {
        isValid: false,
        issues: validation.errors,
        suggestions: ['Ensure all required fields are properly filled']
      };
    }

    // Check if the solution matches the exercise
    if (exercise.solution && exercise.description) {
      if (exercise.solution.toLowerCase().includes('solution') &&
          exercise.description.toLowerCase().includes('exercise')) {
        return {
          isValid: false,
          issues: ['Exercise solution appears to be placeholder text'],
          suggestions: ['Provide a complete and accurate solution for the exercise']
        };
      }
    }

    return {
      isValid: true,
      issues: [],
      suggestions: []
    };
  }

  /**
   * Validates a quiz
   */
  public validateQuiz(quiz: Quiz): ValidationResult {
    const model = new QuizModel(quiz);
    const validation = model.validate();

    if (!validation.isValid) {
      return {
        isValid: false,
        issues: validation.errors,
        suggestions: ['Ensure all required fields are properly filled']
      };
    }

    // Check each question in the quiz
    for (const question of quiz.questions) {
      const questionValidation = this.validateQuizQuestion(question);
      if (!questionValidation.isValid) {
        return {
          isValid: false,
          issues: questionValidation.issues,
          suggestions: questionValidation.suggestions
        };
      }
    }

    return {
      isValid: true,
      issues: [],
      suggestions: []
    };
  }

  /**
   * Validates a quiz question
   */
  public validateQuizQuestion(question: QuizQuestion): ValidationResult {
    const model = new QuizQuestionModel(question);
    const validation = model.validate();

    if (!validation.isValid) {
      return {
        isValid: false,
        issues: validation.errors,
        suggestions: ['Ensure all required fields are properly filled']
      };
    }

    // Check if it's a multiple choice question and has valid options
    if (question.type === 'multiple-choice' && question.options) {
      if (question.options.length < 2) {
        return {
          isValid: false,
          issues: ['Multiple choice questions must have at least 2 options'],
          suggestions: ['Add more answer options to the question']
        };
      }

      if (!question.options.includes(question.correctAnswer)) {
        return {
          isValid: false,
          issues: ['Correct answer is not among the provided options'],
          suggestions: ['Ensure the correct answer is included in the options list']
        };
      }
    }

    return {
      isValid: true,
      issues: [],
      suggestions: []
    };
  }

  /**
   * Validates a module for content consistency
   */
  public validateModule(module: any): ValidationResult {
    const model = new ModuleModel(module);
    const validation = model.validate();

    if (!validation.isValid) {
      return {
        isValid: false,
        issues: validation.errors,
        suggestions: ['Ensure all required fields are properly filled']
      };
    }

    // Check for potential conflicts with prerequisites
    if (module.prerequisites && module.prerequisites.length > 0) {
      // This would require checking against other modules which would need to be passed in
      // For now, just validate the structure
    }

    return {
      isValid: true,
      issues: [],
      suggestions: []
    };
  }

  /**
   * Validates a textbook configuration
   */
  public validateTextbookConfig(config: any): ValidationResult {
    const model = new TextbookConfigModel(config);
    const validation = model.validate();

    if (!validation.isValid) {
      return {
        isValid: false,
        issues: validation.errors,
        suggestions: ['Ensure all required fields are properly filled']
      };
    }

    // Validate all modules in the config
    for (const module of config.modules) {
      const moduleValidation = this.validateModule(module);
      if (!moduleValidation.isValid) {
        return {
          isValid: false,
          issues: moduleValidation.issues,
          suggestions: moduleValidation.suggestions
        };
      }
    }

    return {
      isValid: true,
      issues: [],
      suggestions: []
    };
  }

  /**
   * Validates content coherence across multiple chapters
   */
  public validateContentCoherence(chapters: Chapter[]): ValidationResult {
    const issues: string[] = [];
    const suggestions: string[] = [];

    // Check for potential contradictions between chapters
    for (let i = 0; i < chapters.length; i++) {
      for (let j = i + 1; j < chapters.length; j++) {
        const chapter1 = chapters[i];
        const chapter2 = chapters[j];

        // Look for contradictory terms (this is a simplified check)
        if (chapter1.name.toLowerCase().includes('introduction') &&
            chapter2.name.toLowerCase().includes('advanced') &&
            chapter1.order > chapter2.order) {
          issues.push(`Chapter ordering may be inconsistent: "${chapter1.name}" comes after "${chapter2.name}"`);
          suggestions.push(`Ensure chapters are ordered from basic to advanced concepts`);
        }
      }
    }

    // Check for missing prerequisites
    for (const chapter of chapters) {
      if (chapter.metadata?.prerequisites) {
        for (const prereq of chapter.metadata.prerequisites) {
          const prereqExists = chapters.some(c => c.name.toLowerCase().includes(prereq.toLowerCase()));
          if (!prereqExists) {
            issues.push(`Chapter "${chapter.name}" has prerequisite "${prereq}" which is not included`);
            suggestions.push(`Add the prerequisite chapter or remove it from requirements`);
          }
        }
      }
    }

    return {
      isValid: issues.length === 0,
      issues,
      suggestions
    };
  }
}