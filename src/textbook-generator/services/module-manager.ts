/**
 * ModuleManager service for the Physical AI & Humanoid Robotics Textbook Generator
 * Manages dynamic loading, registration, and validation of textbook modules
 */

import { ModuleModel } from '../models/module';
import { ContentValidator } from './validator';
import { TextbookConfig } from '../models/textbook-config';
import { Chapter } from '../models/chapter';
import fs from 'fs';
import path from 'path';

export interface ModuleRegistrationResult {
  success: boolean;
  moduleId?: string;
  errors: string[];
  warnings: string[];
}

export interface ModuleLoadResult {
  success: boolean;
  module?: ModuleModel;
  errors: string[];
  warnings: string[];
}

export class ModuleManager {
  private modules: Map<string, ModuleModel>;
  private validator: ContentValidator;
  private registeredSources: Map<string, string>; // Maps module ID to source path/URL

  constructor() {
    this.modules = new Map();
    this.validator = new ContentValidator();
    this.registeredSources = new Map();
  }

  /**
   * Registers a module with the manager
   */
  public async registerModule(module: ModuleModel): Promise<ModuleRegistrationResult> {
    const errors: string[] = [];
    const warnings: string[] = [];

    // Validate the module
    const validation = module.validate();
    if (!validation.isValid) {
      errors.push(...validation.errors);
    }
    warnings.push(...validation.warnings);

    // Check for duplicate module ID
    if (this.modules.has(module.id)) {
      errors.push(`Module with ID ${module.id} already exists`);
      return {
        success: false,
        errors,
        warnings
      };
    }

    // Check for conflicts with prerequisites
    const unsatisfiedPrerequisites = this.getUnsatisfiedPrerequisites(module);
    if (unsatisfiedPrerequisites.length > 0) {
      warnings.push(`Module has unsatisfied prerequisites: ${unsatisfiedPrerequisites.join(', ')}`);
    }

    if (errors.length > 0) {
      return {
        success: false,
        errors,
        warnings
      };
    }

    // Add the module to the registry
    this.modules.set(module.id, module);

    // Register source if dynamic loading is enabled
    if (module.dynamicLoading?.enabled && module.dynamicLoading.source) {
      this.registeredSources.set(module.id, module.dynamicLoading.source);
    }

    return {
      success: true,
      moduleId: module.id,
      errors: [],
      warnings
    };
  }

  /**
   * Unregisters a module from the manager
   */
  public unregisterModule(moduleId: string): boolean {
    const removed = this.modules.delete(moduleId);
    this.registeredSources.delete(moduleId);
    return removed;
  }

  /**
   * Loads a module from a file or URL
   */
  public async loadModule(source: string, moduleId?: string): Promise<ModuleLoadResult> {
    const errors: string[] = [];
    const warnings: string[] = [];

    try {
      let moduleData: any;

      // Determine if source is a file path or URL
      if (source.startsWith('http://') || source.startsWith('https://')) {
        // This is a simplified implementation - in a real system, you'd use fetch or similar
        throw new Error('URL loading not implemented in this version');
      } else {
        // Load from file
        if (!fs.existsSync(source)) {
          errors.push(`Module file does not exist: ${source}`);
          return {
            success: false,
            errors,
            warnings
          };
        }

        const content = fs.readFileSync(source, 'utf8');
        moduleData = JSON.parse(content);
      }

      // Create module instance
      const module = new ModuleModel(moduleData);

      // If moduleId is provided, ensure it matches
      if (moduleId && module.id !== moduleId) {
        errors.push(`Module ID mismatch: expected ${moduleId}, got ${module.id}`);
        return {
          success: false,
          errors,
          warnings
        };
      }

      // Validate the loaded module
      const validation = module.validate();
      if (!validation.isValid) {
        errors.push(...validation.errors);
        warnings.push(...validation.warnings);
      }

      return {
        success: errors.length === 0,
        module,
        errors,
        warnings
      };
    } catch (error) {
      errors.push(`Failed to load module from ${source}: ${error instanceof Error ? error.message : String(error)}`);
      return {
        success: false,
        errors,
        warnings
      };
    }
  }

  /**
   * Dynamically loads a registered module
   */
  public async loadRegisteredModule(moduleId: string): Promise<ModuleLoadResult> {
    const source = this.registeredSources.get(moduleId);
    if (!source) {
      return {
        success: false,
        errors: [`Module ${moduleId} is not registered with a source`],
        warnings: []
      };
    }

    return this.loadModule(source, moduleId);
  }

  /**
   * Gets a module by ID
   */
  public getModule(moduleId: string): ModuleModel | undefined {
    return this.modules.get(moduleId);
  }

  /**
   * Gets all registered modules
   */
  public getAllModules(): ModuleModel[] {
    return Array.from(this.modules.values());
  }

  /**
   * Checks if all prerequisites for a module are satisfied
   */
  public getUnsatisfiedPrerequisites(module: ModuleModel): string[] {
    const availableModuleIds = Array.from(this.modules.keys());
    return module.prerequisites.filter(prereqId => !availableModuleIds.includes(prereqId));
  }

  /**
   * Checks for circular dependencies among registered modules
   */
  public hasCircularDependencies(): boolean {
    const modulesArray = Array.from(this.modules.values());

    // Build dependency graph
    const graph: Map<string, string[]> = new Map();
    for (const module of modulesArray) {
      graph.set(module.id, module.prerequisites);
    }

    // Check for cycles using DFS
    const visited: Set<string> = new Set();
    const recStack: Set<string> = new Set();

    for (const moduleId of graph.keys()) {
      if (this.isCyclicUtil(moduleId, graph, visited, recStack)) {
        return true;
      }
    }

    return false;
  }

  /**
   * Helper method for cycle detection
   */
  private isCyclicUtil(
    moduleId: string,
    graph: Map<string, string[]>,
    visited: Set<string>,
    recStack: Set<string>
  ): boolean {
    if (!visited.has(moduleId)) {
      visited.add(moduleId);
      recStack.add(moduleId);

      const dependencies = graph.get(moduleId) || [];
      for (const dependency of dependencies) {
        if (!visited.has(dependency) && this.isCyclicUtil(dependency, graph, visited, recStack)) {
          return true;
        } else if (recStack.has(dependency)) {
          return true;
        }
      }
    }

    recStack.delete(moduleId);
    return false;
  }

  /**
   * Validates module compatibility with existing modules
   */
  public validateModuleCompatibility(newModule: ModuleModel): { isValid: boolean; conflicts: string[] } {
    const conflicts: string[] = [];
    const existingModules = this.getAllModules();

    for (const existingModule of existingModules) {
      // Check for name conflicts
      if (existingModule.name === newModule.name && existingModule.id !== newModule.id) {
        conflicts.push(`Module name conflict: ${newModule.name} already exists with different ID`);
      }

      // Check for content conflicts (e.g., overlapping chapter names)
      for (const newChapter of newModule.chapters) {
        for (const existingChapter of existingModule.chapters) {
          if (newChapter.name === existingChapter.name) {
            conflicts.push(`Chapter name conflict: ${newChapter.name} exists in both modules ${newModule.id} and ${existingModule.id}`);
          }
        }
      }

      // Check compatibility using module's own method
      if (!newModule.isCompatibleWith(existingModule)) {
        conflicts.push(`Module ${newModule.id} is not compatible with ${existingModule.id}`);
      }
    }

    return {
      isValid: conflicts.length === 0,
      conflicts
    };
  }

  /**
   * Builds a dependency graph for modules
   */
  public buildDependencyGraph(): Map<string, string[]> {
    const graph: Map<string, string[]> = new Map();

    for (const [moduleId, module] of this.modules) {
      graph.set(moduleId, [...module.prerequisites]);
    }

    return graph;
  }

  /**
   * Gets modules in topological order based on dependencies
   */
  public getModulesInDependencyOrder(): ModuleModel[] {
    const graph = this.buildDependencyGraph();
    const visited: Set<string> = new Set();
    const stack: string[] = [];

    // Perform topological sort
    for (const moduleId of graph.keys()) {
      if (!visited.has(moduleId)) {
        this.topologicalSortUtil(moduleId, graph, visited, stack);
      }
    }

    // Return modules in dependency order
    return stack.reverse().map(id => this.modules.get(id)).filter(mod => mod !== undefined) as ModuleModel[];
  }

  /**
   * Helper for topological sort
   */
  private topologicalSortUtil(
    moduleId: string,
    graph: Map<string, string[]>,
    visited: Set<string>,
    stack: string[]
  ): void {
    visited.add(moduleId);
    const dependencies = graph.get(moduleId) || [];

    for (const dependency of dependencies) {
      if (!visited.has(dependency)) {
        this.topologicalSortUtil(dependency, graph, visited, stack);
      }
    }

    stack.push(moduleId);
  }

  /**
   * Integrates a new module into an existing textbook configuration
   */
  public integrateModuleIntoTextbook(module: ModuleModel, config: TextbookConfig): TextbookConfig {
    // Create a deep copy of the config
    const newConfig = JSON.parse(JSON.stringify(config));

    // Add the module to the configuration
    const moduleExists = newConfig.modules.some((m: any) => m.id === module.id);
    if (!moduleExists) {
      newConfig.modules.push(module);
    }

    // Sort modules by order
    newConfig.modules.sort((a: any, b: any) => a.order - b.order);

    return newConfig;
  }

  /**
   * Removes a module from a textbook configuration
   */
  public removeFromTextbook(moduleId: string, config: TextbookConfig): TextbookConfig {
    // Create a deep copy of the config
    const newConfig = JSON.parse(JSON.stringify(config));

    // Remove the module
    newConfig.modules = newConfig.modules.filter((m: any) => m.id !== moduleId);

    return newConfig;
  }

  /**
   * Gets modules that depend on a specific module
   */
  public getDependentModules(targetModuleId: string): ModuleModel[] {
    return this.getAllModules().filter(module =>
      module.prerequisites.includes(targetModuleId)
    );
  }
}