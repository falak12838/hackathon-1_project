/**
 * Simple test to debug module validation
 */

import { ModuleManager } from '../services/module-manager';
import { ModuleModel } from '../models/module';
import { Chapter } from '../models/chapter';

describe('Simple Module Test', () => {
  test('should create a basic module successfully', async () => {
    const moduleManager = new ModuleManager();

    const simpleModule = new ModuleModel({
      id: 'simple-module',
      name: 'Simple Module',
      description: 'A simple test module',
      order: 1,
      chapters: [
        {
          id: 'ch-1',
          name: 'Introduction',
          content: 'This is an introduction chapter',
          order: 1,
          learningObjectives: [],
          exercises: [],
          quizzes: [],
          metadata: {
            difficultyLevel: 'beginner',
            estimatedReadingTime: 10,
            keywords: ['introduction'],
            prerequisites: []
          }
        }
      ],
      prerequisites: [],
      metadata: {
        version: '1.0.0',
        author: 'Test Author'
      }
    });

    console.log('Module created:', simpleModule);
    console.log('Module validation:', simpleModule.validate());

    const result = await moduleManager.registerModule(simpleModule);
    console.log('Registration result:', result);

    expect(result.success).toBe(true);
  });
});