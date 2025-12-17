/**
 * Chapter model for the Physical AI & Humanoid Robotics Textbook Generator
 * Represents a major division of the textbook containing related content, learning objectives, exercises, and quizzes
 */

export interface LearningObjective {
  id: string;
  chapterId: string;
  description: string;
  measurable: boolean;
  tags: string[];
}

export interface Exercise {
  id: string;
  chapterId: string;
  title: string;
  description: string;
  difficulty: 'beginner' | 'intermediate' | 'advanced';
  solution: string;
  type: 'coding' | 'theoretical' | 'practical';
}

export interface QuizQuestion {
  id: string;
  quizId: string;
  question: string;
  options?: string[];
  correctAnswer: string;
  explanation: string;
  type: 'multiple-choice' | 'true-false' | 'short-answer';
}

export interface Quiz {
  id: string;
  chapterId: string;
  title: string;
  questions: QuizQuestion[];
  timeLimit?: number; // Time limit in minutes
}

export interface Chapter {
  id: string;
  name: string;
  content: string;
  order: number;
  learningObjectives: LearningObjective[];
  exercises: Exercise[];
  quizzes: Quiz[];
  metadata: {
    difficultyLevel?: string;
    estimatedReadingTime?: number; // in minutes
    keywords?: string[];
    prerequisites?: string[];
  };
}

// Export all types for use in other modules
export type { QuizQuestion as QuizQuestionType, Quiz as QuizType, Chapter as ChapterType, LearningObjective as LearningObjectiveType, Exercise as ExerciseType };