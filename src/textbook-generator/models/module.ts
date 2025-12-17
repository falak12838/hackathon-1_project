/**
 * Module model for the Physical AI & Humanoid Robotics Textbook Generator
 * Represents a self-contained unit of content that can be added to expand the textbook
 */

import { Chapter } from './chapter';

export interface Module {
  id: string;
  name: string;
  description: string;
  chapters: Chapter[];
  order: number;
  prerequisites: string[]; // List of module IDs that should be completed before this one
  dependencies?: string[]; // Additional module dependencies
  metadata?: {
    version?: string;
    author?: string;
    license?: string;
    tags?: string[];
    lastUpdated?: string;
    compatibility?: string; // Version compatibility information
  };
  dynamicLoading?: {
    enabled: boolean;
    source?: string; // URL or file path for dynamic loading
    autoUpdate?: boolean;
  };
}

export class ModuleModel {
  public id: string;
  public name: string;
  public description: string;
  public chapters: Chapter[];
  public order: number;
  public prerequisites: string[];
  public dependencies: string[];
  public metadata?: {
    version?: string;
    author?: string;
    license?: string;
    tags?: string[];
    lastUpdated?: string;
    compatibility?: string;
  };
  public dynamicLoading?: {
    enabled: boolean;
    source?: string;
    autoUpdate?: boolean;
  };

  constructor(data: Partial<Module> = {}) {
    this.id = data.id || this.generateId();
    this.name = data.name || '';
    this.description = data.description || '';
    this.chapters = data.chapters || [];
    this.order = data.order || 0;
    this.prerequisites = data.prerequisites || [];
    this.dependencies = data.dependencies || [];
    this.metadata = data.metadata;
    this.dynamicLoading = data.dynamicLoading || { enabled: false };
  }

  private generateId(): string {
    return `mod_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`;
  }

  public validate(): { isValid: boolean; errors: string[]; warnings: string[] } {
    const errors: string[] = [];
    const warnings: string[] = [];

    if (!this.name.trim()) {
      errors.push('Module name is required');
    }

    if (!this.description.trim()) {
      errors.push('Module description is required');
    }

    if (this.chapters.length === 0) {
      errors.push('Module must have at least one chapter');
    }

    // Check for duplicate chapter names within the module
    const chapterNames = this.chapters.map(ch => ch.name);
    const uniqueNames = new Set(chapterNames);
    if (chapterNames.length !== uniqueNames.size) {
      errors.push('Chapter names within a module must be unique');
    }

    // Check for circular dependencies
    if (this.prerequisites.includes(this.id)) {
      errors.push('Module cannot have itself as a prerequisite');
    }

    // Validate dynamic loading configuration if enabled
    if (this.dynamicLoading?.enabled && !this.dynamicLoading.source) {
      errors.push('Dynamic loading is enabled but no source is specified');
    }

    // Check for metadata
    if (!this.metadata?.version) {
      warnings.push('Module version is not specified');
    }

    if (!this.metadata?.author) {
      warnings.push('Module author is not specified');
    }

    return {
      isValid: errors.length === 0,
      errors,
      warnings
    };
  }

  /**
   * Checks if this module is compatible with another module
   */
  public isCompatibleWith(otherModule: ModuleModel): boolean {
    // Check version compatibility if specified
    if (this.metadata?.compatibility && otherModule.metadata?.version) {
      // This is a simplified check - in a real implementation, you'd use semver
      return this.metadata.compatibility.includes(otherModule.metadata.version);
    }
    return true;
  }

  /**
   * Checks if all prerequisites are satisfied
   */
  public hasPrerequisitesSatisfied(availableModules: ModuleModel[]): boolean {
    return this.prerequisites.every(prereqId =>
      availableModules.some(mod => mod.id === prereqId)
    );
  }

  /**
   * Loads module content dynamically from a source
   */
  public async loadDynamically(): Promise<boolean> {
    if (!this.dynamicLoading?.enabled || !this.dynamicLoading.source) {
      return false;
    }

    try {
      // This is a placeholder implementation
      // In a real implementation, this would fetch content from the source
      console.log(`Loading module ${this.name} from ${this.dynamicLoading.source}`);

      // For now, return true to indicate success
      return true;
    } catch (error) {
      console.error(`Failed to load module ${this.name} dynamically:`, error);
      return false;
    }
  }

  /**
   * Updates module content if auto-update is enabled
   */
  public async updateIfNecessary(): Promise<boolean> {
    if (!this.dynamicLoading?.autoUpdate) {
      return false;
    }

    return this.loadDynamically();
  }
}