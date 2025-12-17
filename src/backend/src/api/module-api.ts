/**
 * API endpoints for module management in the Physical AI & Humanoid Robotics Textbook Generator
 */

import express, { Request, Response, Router } from 'express';
import { ModuleManager } from '../../../textbook-generator/services/module-manager';
import { ModuleModel } from '../../../textbook-generator/models/module';
import { validationResult } from 'express-validator';
import { body, param, query } from 'express-validator';

// Create a global instance of the module manager
const moduleManager = new ModuleManager();

const router: Router = express.Router();

// Validation middleware
const validateModule = [
  body('id').isString().withMessage('Module ID must be a string'),
  body('name').isString().withMessage('Module name must be a string'),
  body('description').isString().withMessage('Module description must be a string'),
  body('chapters').isArray().withMessage('Chapters must be an array'),
  body('order').isNumeric().withMessage('Order must be a number'),
  body('prerequisites').isArray().withMessage('Prerequisites must be an array')
];

// GET /api/modules - Get all registered modules
router.get('/', async (req: Request, res: Response) => {
  try {
    const modules = moduleManager.getAllModules();
    res.json({
      success: true,
      data: modules,
      count: modules.length
    });
  } catch (error) {
    res.status(500).json({
      success: false,
      error: error instanceof Error ? error.message : 'Unknown error occurred'
    });
  }
});

// GET /api/modules/:id - Get a specific module by ID
router.get('/:id',
  param('id').isString().withMessage('Module ID must be a string'),
  async (req: Request, res: Response) => {
    try {
      const errors = validationResult(req);
      if (!errors.isEmpty()) {
        return res.status(400).json({
          success: false,
          errors: errors.array()
        });
      }

      const moduleId = req.params.id;
      const module = moduleManager.getModule(moduleId);

      if (!module) {
        return res.status(404).json({
          success: false,
          error: `Module with ID ${moduleId} not found`
        });
      }

      return res.json({
        success: true,
        data: module
      });
    } catch (error) {
      return res.status(500).json({
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error occurred'
      });
    }
  });

// POST /api/modules - Register a new module
router.post('/', validateModule, async (req: Request, res: Response) => {
  try {
    const errors = validationResult(req);
    if (!errors.isEmpty()) {
      return res.status(400).json({
        success: false,
        errors: errors.array()
      });
    }

    const moduleData = req.body;
    const module = new ModuleModel(moduleData);

    const result = await moduleManager.registerModule(module);

    if (result.success) {
      return res.status(201).json({
        success: true,
        moduleId: result.moduleId,
        message: `Module ${module.name} registered successfully`
      });
    } else {
      return res.status(400).json({
        success: false,
        errors: result.errors,
        warnings: result.warnings
      });
    }
  } catch (error) {
    return res.status(500).json({
      success: false,
      error: error instanceof Error ? error.message : 'Unknown error occurred'
    });
  }
});

// PUT /api/modules/:id - Update an existing module
router.put('/:id',
  param('id').isString().withMessage('Module ID must be a string'),
  validateModule,
  async (req: Request, res: Response) => {
    try {
      const errors = validationResult(req);
      if (!errors.isEmpty()) {
        return res.status(400).json({
          success: false,
          errors: errors.array()
        });
      }

      const moduleId = req.params.id;
      const module = moduleManager.getModule(moduleId);

      if (!module) {
        return res.status(404).json({
          success: false,
          error: `Module with ID ${moduleId} not found`
        });
      }

      // Unregister the old module
      moduleManager.unregisterModule(moduleId);

      // Create and register the updated module
      const moduleData = { ...req.body, id: moduleId }; // Preserve the original ID
      const updatedModule = new ModuleModel(moduleData);

      const result = await moduleManager.registerModule(updatedModule);

      if (result.success) {
        return res.json({
          success: true,
          moduleId: result.moduleId,
          message: `Module ${updatedModule.name} updated successfully`
        });
      } else {
        // Restore the original module if update failed
        moduleManager.registerModule(module);
        return res.status(400).json({
          success: false,
          errors: result.errors,
          warnings: result.warnings
        });
      }
    } catch (error) {
      return res.status(500).json({
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error occurred'
      });
    }
  });

// DELETE /api/modules/:id - Unregister a module
router.delete('/:id',
  param('id').isString().withMessage('Module ID must be a string'),
  async (req: Request, res: Response) => {
    try {
      const errors = validationResult(req);
      if (!errors.isEmpty()) {
        return res.status(400).json({
          success: false,
          errors: errors.array()
        });
      }

      const moduleId = req.params.id;
      const module = moduleManager.getModule(moduleId);

      if (!module) {
        return res.status(404).json({
          success: false,
          error: `Module with ID ${moduleId} not found`
        });
      }

      const removed = moduleManager.unregisterModule(moduleId);

      if (removed) {
        return res.json({
          success: true,
          message: `Module ${module.name} unregistered successfully`
        });
      } else {
        return res.status(500).json({
          success: false,
          error: 'Failed to unregister module'
        });
      }
    } catch (error) {
      return res.status(500).json({
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error occurred'
      });
    }
  });

// POST /api/modules/load - Load a module from a file or URL
router.post('/load',
  body('source').isString().withMessage('Source must be a string'),
  body('moduleId').optional().isString().withMessage('Module ID must be a string'),
  async (req: Request, res: Response) => {
    try {
      const errors = validationResult(req);
      if (!errors.isEmpty()) {
        return res.status(400).json({
          success: false,
          errors: errors.array()
        });
      }

      const { source, moduleId } = req.body;
      const result = await moduleManager.loadModule(source, moduleId);

      if (result.success && result.module) {
        // Optionally register the loaded module
        const registerResult = await moduleManager.registerModule(result.module);

        return res.json({
          success: true,
          module: result.module,
          registered: registerResult.success,
          moduleId: registerResult.moduleId,
          warnings: result.warnings
        });
      } else {
        return res.status(400).json({
          success: false,
          errors: result.errors,
          warnings: result.warnings
        });
      }
    } catch (error) {
      return res.status(500).json({
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error occurred'
      });
    }
  });

// GET /api/modules/:id/prerequisites - Check prerequisites for a module
router.get('/:id/prerequisites',
  param('id').isString().withMessage('Module ID must be a string'),
  async (req: Request, res: Response) => {
    try {
      const errors = validationResult(req);
      if (!errors.isEmpty()) {
        return res.status(400).json({
          success: false,
          errors: errors.array()
        });
      }

      const moduleId = req.params.id;
      const module = moduleManager.getModule(moduleId);

      if (!module) {
        return res.status(404).json({
          success: false,
          error: `Module with ID ${moduleId} not found`
        });
      }

      const unsatisfied = moduleManager.getUnsatisfiedPrerequisites(module);
      const satisfied = module.prerequisites.filter(prereq => !unsatisfied.includes(prereq));

      return res.json({
        success: true,
        data: {
          moduleId,
          satisfied,
          unsatisfied,
          all: module.prerequisites
        }
      });
    } catch (error) {
      return res.status(500).json({
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error occurred'
      });
    }
  });

// GET /api/modules/dependency-graph - Get the dependency graph of all modules
router.get('/dependency-graph', async (req: Request, res: Response) => {
  try {
    const graph = moduleManager.buildDependencyGraph();
    const hasCycles = moduleManager.hasCircularDependencies();

    return res.json({
      success: true,
      data: {
        graph: Object.fromEntries(graph),
        hasCycles,
        orderedModules: moduleManager.getModulesInDependencyOrder().map(m => m.id)
      }
    });
  } catch (error) {
    return res.status(500).json({
      success: false,
      error: error instanceof Error ? error.message : 'Unknown error occurred'
    });
  }
});

// GET /api/modules/:id/dependents - Get modules that depend on the specified module
router.get('/:id/dependents',
  param('id').isString().withMessage('Module ID must be a string'),
  async (req: Request, res: Response) => {
    try {
      const errors = validationResult(req);
      if (!errors.isEmpty()) {
        return res.status(400).json({
          success: false,
          errors: errors.array()
        });
      }

      const moduleId = req.params.id;
      const dependents = moduleManager.getDependentModules(moduleId);

      return res.json({
        success: true,
        data: dependents,
        count: dependents.length
      });
    } catch (error) {
      return res.status(500).json({
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error occurred'
      });
    }
  });

export default router;