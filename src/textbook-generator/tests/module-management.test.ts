/**
 * Tests for module management functionality
 * Task T032: Test addition of new modules without breaking existing functionality
 * Task T033: Validate content coherence when modules are added or removed
 */

import { ModuleManager } from '../services/module-manager';
import { ModuleModel } from '../models/module';
import { Chapter, LearningObjective, Exercise, Quiz, QuizQuestion } from '../models/chapter';
import { TextbookConfigModel } from '../models/textbook-config';

describe('Module Management Tests', () => {
  let moduleManager: ModuleManager;

  beforeEach(() => {
    moduleManager = new ModuleManager();
  });

  describe('T032: Add new modules without breaking existing functionality', () => {
    test('should add new modules while preserving existing modules', async () => {
      // Create and register an initial module
      const initialModule = new ModuleModel({
        id: 'module-1',
        name: 'Initial Module',
        description: 'The first module',
        order: 1,
        chapters: [
          {
            id: 'ch-1',
            name: 'Introduction',
            content: 'This is the introduction chapter',
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

      const result1 = await moduleManager.registerModule(initialModule);
      expect(result1.success).toBe(true);

      // Verify the initial module is registered
      const initialModuleRetrieved = moduleManager.getModule('module-1');
      expect(initialModuleRetrieved).toBeDefined();
      expect(initialModuleRetrieved?.name).toBe('Initial Module');

      // Add a second module
      const secondModule = new ModuleModel({
        id: 'module-2',
        name: 'Second Module',
        description: 'The second module',
        order: 2,
        chapters: [
          {
            id: 'ch-2',
            name: 'Advanced Topics',
            content: 'This covers advanced topics',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'intermediate',
              estimatedReadingTime: 15,
              keywords: ['advanced'],
              prerequisites: []
            }
          }
        ],
        prerequisites: ['module-1'], // Has dependency on first module
        metadata: {
          version: '1.0.0',
          author: 'Test Author'
        }
      });

      const result2 = await moduleManager.registerModule(secondModule);
      expect(result2.success).toBe(true);

      // Verify both modules still exist
      const module1 = moduleManager.getModule('module-1');
      const module2 = moduleManager.getModule('module-2');

      expect(module1).toBeDefined();
      expect(module2).toBeDefined();
      expect(module1?.name).toBe('Initial Module');
      expect(module2?.name).toBe('Second Module');

      // Verify dependencies are correctly tracked
      const unsatisfiedPrereqs = moduleManager.getUnsatisfiedPrerequisites(secondModule);
      expect(unsatisfiedPrereqs).toHaveLength(0); // Should be satisfied since module-1 exists

      // Verify all modules can be retrieved
      const allModules = moduleManager.getAllModules();
      expect(allModules).toHaveLength(2);
      expect(allModules.map(m => m.id)).toContain('module-1');
      expect(allModules.map(m => m.id)).toContain('module-2');
    });

    test('should handle module registration with validation', async () => {
      const validModule = new ModuleModel({
        id: 'valid-module',
        name: 'Valid Module',
        description: 'A properly configured module',
        order: 1,
        chapters: [
          {
            id: 'ch-1',
            name: 'Chapter 1',
            content: 'Content for chapter 1',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'beginner',
              estimatedReadingTime: 10,
              keywords: ['basic'],
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

      const result = await moduleManager.registerModule(validModule);
      expect(result.success).toBe(true);
      expect(result.errors).toHaveLength(0);

      const retrievedModule = moduleManager.getModule('valid-module');
      expect(retrievedModule).toBeDefined();
      expect(retrievedModule?.name).toBe('Valid Module');
    });

    test('should not break when adding modules with circular dependencies', async () => {
      // Add first module that depends on the second
      const moduleA = new ModuleModel({
        id: 'module-a',
        name: 'Module A',
        description: 'Module A',
        order: 1,
        chapters: [
          {
            id: 'ch-a',
            name: 'Module A Chapter',
            content: 'Content for Module A',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'beginner',
              estimatedReadingTime: 10,
              keywords: ['module-a'],
              prerequisites: []
            }
          }
        ],
        prerequisites: ['module-b'] // Depends on module-b (not yet registered)
      });

      // Add second module that depends on the first
      const moduleB = new ModuleModel({
        id: 'module-b',
        name: 'Module B',
        description: 'Module B',
        order: 2,
        chapters: [
          {
            id: 'ch-b',
            name: 'Module B Chapter',
            content: 'Content for Module B',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'beginner',
              estimatedReadingTime: 10,
              keywords: ['module-b'],
              prerequisites: []
            }
          }
        ],
        prerequisites: ['module-a'] // Depends on module-a (will create circular dependency)
      });

      // Register both modules
      const resultA = await moduleManager.registerModule(moduleA);
      const resultB = await moduleManager.registerModule(moduleB);

      expect(resultA.success).toBe(true); // First module can be registered
      expect(resultB.success).toBe(true); // Second module can be registered

      // Check for circular dependencies
      const hasCircularDeps = moduleManager.hasCircularDependencies();
      expect(hasCircularDeps).toBe(true); // Should detect the circular dependency

      // Verify both modules still exist
      expect(moduleManager.getModule('module-a')).toBeDefined();
      expect(moduleManager.getModule('module-b')).toBeDefined();
    });

    test('should maintain dependency order when retrieving modules', async () => {
      // Create modules with dependencies
      const moduleC = new ModuleModel({
        id: 'module-c',
        name: 'Module C',
        description: 'Module C - depends on A and B',
        order: 3,
        chapters: [
          {
            id: 'ch-c',
            name: 'Module C Chapter',
            content: 'Content for Module C',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'advanced',
              estimatedReadingTime: 15,
              keywords: ['module-c'],
              prerequisites: []
            }
          }
        ],
        prerequisites: ['module-a', 'module-b']
      });

      const moduleB = new ModuleModel({
        id: 'module-b',
        name: 'Module B',
        description: 'Module B - depends on A',
        order: 2,
        chapters: [
          {
            id: 'ch-b',
            name: 'Module B Chapter',
            content: 'Content for Module B',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'intermediate',
              estimatedReadingTime: 12,
              keywords: ['module-b'],
              prerequisites: []
            }
          }
        ],
        prerequisites: ['module-a']
      });

      const moduleA = new ModuleModel({
        id: 'module-a',
        name: 'Module A',
        description: 'Module A - no dependencies',
        order: 1,
        chapters: [
          {
            id: 'ch-a',
            name: 'Module A Chapter',
            content: 'Content for Module A',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'beginner',
              estimatedReadingTime: 10,
              keywords: ['module-a'],
              prerequisites: []
            }
          }
        ],
        prerequisites: []
      });

      // Register in reverse order to test dependency resolution
      const resultC = await moduleManager.registerModule(moduleC);
      const resultB = await moduleManager.registerModule(moduleB);
      const resultA = await moduleManager.registerModule(moduleA);

      expect(resultA.success).toBe(true);
      expect(resultB.success).toBe(true);
      expect(resultC.success).toBe(true);

      // Get modules in dependency order (A should come before B, which should come before C)
      const orderedModules = moduleManager.getModulesInDependencyOrder();

      const moduleIds = orderedModules.map(m => m.id);
      const indexOfA = moduleIds.indexOf('module-a');
      const indexOfB = moduleIds.indexOf('module-b');
      const indexOfC = moduleIds.indexOf('module-c');

      // Module A should come before Module B
      expect(indexOfA).toBeLessThan(indexOfB);
      // Module B should come before Module C
      expect(indexOfB).toBeLessThan(indexOfC);
    });
  });

  describe('T033: Validate content coherence when modules are added or removed', () => {
    test('should validate module compatibility when adding new modules', async () => {
      // Add first module
      const module1 = new ModuleModel({
        id: 'module-1',
        name: 'Physics Fundamentals',
        description: 'Basic physics concepts',
        order: 1,
        chapters: [
          {
            id: 'ch-physics-1',
            name: 'Newton Laws',
            content: 'Newton\'s laws of motion',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'beginner',
              estimatedReadingTime: 15,
              keywords: ['physics', 'newton'],
              prerequisites: []
            }
          }
        ],
        prerequisites: []
      });

      const result1 = await moduleManager.registerModule(module1);
      expect(result1.success).toBe(true);

      // Try to add a compatible module
      const compatibleModule = new ModuleModel({
        id: 'module-2',
        name: 'Advanced Physics',
        description: 'Advanced physics concepts',
        order: 2,
        chapters: [
          {
            id: 'ch-adv-physics-1',
            name: 'Quantum Mechanics',
            content: 'Introduction to quantum mechanics',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'advanced',
              estimatedReadingTime: 20,
              keywords: ['physics', 'quantum'],
              prerequisites: []
            }
          }
        ],
        prerequisites: ['module-1']
      });

      const result2 = await moduleManager.registerModule(compatibleModule);
      expect(result2.success).toBe(true);

      // Validate compatibility
      const compatibilityCheck = moduleManager.validateModuleCompatibility(compatibleModule);
      expect(compatibilityCheck.isValid).toBe(true);
      expect(compatibilityCheck.conflicts).toHaveLength(0);
    });

    test('should detect conflicts when adding modules with overlapping content', async () => {
      // Add first module
      const module1 = new ModuleModel({
        id: 'module-1',
        name: 'Physics Fundamentals',
        description: 'Basic physics concepts',
        order: 1,
        chapters: [
          {
            id: 'ch-1',
            name: 'Newton Laws',
            content: 'Newton\'s laws of motion',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'beginner',
              estimatedReadingTime: 15,
              keywords: ['physics', 'newton'],
              prerequisites: []
            }
          }
        ],
        prerequisites: []
      });

      const result1 = await moduleManager.registerModule(module1);
      expect(result1.success).toBe(true);

      // Try to add a module with conflicting chapter name
      const conflictingModule = new ModuleModel({
        id: 'module-2',
        name: 'Mechanics Deep Dive',
        description: 'Detailed mechanics concepts',
        order: 2,
        chapters: [
          {
            id: 'ch-2',
            name: 'Newton Laws', // Same name as in module-1 (conflict!)
            content: 'Detailed analysis of Newton\'s laws',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'intermediate',
              estimatedReadingTime: 18,
              keywords: ['mechanics', 'newton'],
              prerequisites: []
            }
          }
        ],
        prerequisites: ['module-1']
      });

      const result2 = await moduleManager.registerModule(conflictingModule);
      expect(result2.success).toBe(true); // Module can still be registered

      // But validation should detect the conflict
      const compatibilityCheck = moduleManager.validateModuleCompatibility(conflictingModule);
      expect(compatibilityCheck.isValid).toBe(false);
      expect(compatibilityCheck.conflicts).toContainEqual(
        expect.stringContaining('Chapter name conflict: Newton Laws exists in both modules')
      );
    });

    test('should handle module removal and update dependencies', async () => {
      // Create modules with dependencies
      const moduleA = new ModuleModel({
        id: 'module-a',
        name: 'Foundation Module',
        description: 'Basic concepts',
        order: 1,
        chapters: [
          {
            id: 'ch-a',
            name: 'Foundation Concepts',
            content: 'Basic concepts',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'beginner',
              estimatedReadingTime: 10,
              keywords: ['foundation'],
              prerequisites: []
            }
          }
        ],
        prerequisites: []
      });

      const moduleB = new ModuleModel({
        id: 'module-b',
        name: 'Advanced Module',
        description: 'Advanced concepts',
        order: 2,
        chapters: [
          {
            id: 'ch-b',
            name: 'Advanced Concepts',
            content: 'Advanced concepts',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'advanced',
              estimatedReadingTime: 15,
              keywords: ['advanced'],
              prerequisites: []
            }
          }
        ],
        prerequisites: ['module-a'] // Depends on module-a
      });

      await moduleManager.registerModule(moduleA);
      await moduleManager.registerModule(moduleB);

      // Verify both modules exist
      expect(moduleManager.getModule('module-a')).toBeDefined();
      expect(moduleManager.getModule('module-b')).toBeDefined();

      // Check dependencies - module-b should depend on module-a
      const dependentModules = moduleManager.getDependentModules('module-a');
      expect(dependentModules).toHaveLength(1);
      expect(dependentModules[0].id).toBe('module-b');

      // Remove the foundation module
      const wasRemoved = moduleManager.unregisterModule('module-a');
      expect(wasRemoved).toBe(true);

      // Verify module-a is removed but module-b still exists
      expect(moduleManager.getModule('module-a')).toBeUndefined();
      expect(moduleManager.getModule('module-b')).toBeDefined();

      // Check if there are any modules that depend on the removed module
      const dependentModulesAfterRemoval = moduleManager.getDependentModules('module-a');
      expect(dependentModulesAfterRemoval).toHaveLength(0);
    });

    test('should integrate modules into textbook configuration properly', async () => {
      // Create a base textbook configuration
      const baseConfig = new TextbookConfigModel({
        id: 'textbook-1',
        title: 'Physical AI & Humanoid Robotics Textbook',
        author: 'AI Assistant',
        targetAudience: 'undergraduate',
        difficultyLevel: 'intermediate',
        modules: [],
        outputFormats: ['html', 'markdown']
      });

      // Create a module to add
      const newModule = new ModuleModel({
        id: 'module-robotics',
        name: 'Robotics Fundamentals',
        description: 'Basic robotics concepts',
        order: 1,
        chapters: [
          {
            id: 'ch-robotics-intro',
            name: 'Introduction to Robotics',
            content: 'Basic robotics concepts and principles',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'beginner',
              estimatedReadingTime: 12,
              keywords: ['robotics', 'introduction'],
              prerequisites: []
            }
          }
        ],
        prerequisites: []
      });

      // Integrate the module into the textbook
      const updatedConfig = moduleManager.integrateModuleIntoTextbook(newModule, baseConfig);

      // Verify the module was added
      expect(updatedConfig.modules).toHaveLength(1);
      expect(updatedConfig.modules[0].id).toBe('module-robotics');
      expect(updatedConfig.modules[0].name).toBe('Robotics Fundamentals');

      // Create another module and integrate it
      const secondModule = new ModuleModel({
        id: 'module-ai',
        name: 'AI Fundamentals',
        description: 'Basic AI concepts',
        order: 2,
        chapters: [
          {
            id: 'ch-ai-intro',
            name: 'Introduction to AI',
            content: 'Basic AI concepts and principles',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'beginner',
              estimatedReadingTime: 10,
              keywords: ['ai', 'introduction'],
              prerequisites: []
            }
          }
        ],
        prerequisites: ['module-robotics']
      });

      const configWithBoth = moduleManager.integrateModuleIntoTextbook(secondModule, updatedConfig);

      // Verify both modules are in the config
      expect(configWithBoth.modules).toHaveLength(2);
      expect(configWithBoth.modules.map(m => m.id)).toContain('module-robotics');
      expect(configWithBoth.modules.map(m => m.id)).toContain('module-ai');

      // Verify they are sorted by order
      expect(configWithBoth.modules[0].order).toBe(1);
      expect(configWithBoth.modules[1].order).toBe(2);
    });

    test('should remove modules from textbook configuration properly', async () => {
      // Create a textbook configuration with modules
      const module1 = new ModuleModel({
        id: 'module-1',
        name: 'Module 1',
        description: 'First module',
        order: 1,
        chapters: [
          {
            id: 'ch-1',
            name: 'Chapter 1',
            content: 'Content for chapter 1',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'beginner',
              estimatedReadingTime: 10,
              keywords: ['module1'],
              prerequisites: []
            }
          }
        ],
        prerequisites: []
      });

      const module2 = new ModuleModel({
        id: 'module-2',
        name: 'Module 2',
        description: 'Second module',
        order: 2,
        chapters: [
          {
            id: 'ch-2',
            name: 'Chapter 2',
            content: 'Content for chapter 2',
            order: 1,
            learningObjectives: [],
            exercises: [],
            quizzes: [],
            metadata: {
              difficultyLevel: 'intermediate',
              estimatedReadingTime: 12,
              keywords: ['module2'],
              prerequisites: []
            }
          }
        ],
        prerequisites: ['module-1']
      });

      const config = new TextbookConfigModel({
        id: 'textbook-test',
        title: 'Test Textbook',
        author: 'Test Author',
        targetAudience: 'undergraduate',
        difficultyLevel: 'beginner',
        modules: [module1, module2],
        outputFormats: ['html']
      });

      // Verify initial state
      expect(config.modules).toHaveLength(2);

      // Remove the second module
      const configAfterRemoval = moduleManager.removeFromTextbook('module-2', config);

      // Verify the module was removed
      expect(configAfterRemoval.modules).toHaveLength(1);
      expect(configAfterRemoval.modules[0].id).toBe('module-1');
      expect(configAfterRemoval.modules.find(m => m.id === 'module-2')).toBeUndefined();
    });
  });
});