/**
 * Textbook Generation Workflow Orchestrator for the Physical AI & Humanoid Robotics Textbook Generator
 * Coordinates the entire textbook generation process from configuration to output
 */

import { TextbookConfig } from '../models/textbook-config';
import { Chapter } from '../models/chapter';
import { ContentGenerator, GenerationResult } from './content-generator';
import { ContentProcessor, ProcessedTextbook } from './content-processor';
import { ContentValidator, ValidationResult } from './validator';
import { saveToFile, generateId } from '../lib/utils';
import fs from 'fs';
import path from 'path';

export interface WorkflowOptions {
  validateContent?: boolean;
  processContent?: boolean;
  generateOutput?: boolean;
  outputFormats?: string[];
  outputPath?: string;
}

export interface WorkflowResult {
  success: boolean;
  taskId: string;
  config: TextbookConfig;
  generatedTextbook?: {
    chapters: Chapter[];
    outputPath: string;
  };
  processedTextbook?: ProcessedTextbook;
  validationResults?: ValidationResult[];
  outputPaths?: string[];
  errors: string[];
  warnings: string[];
  executionTime: number;
}

export class TextbookWorkflowOrchestrator {
  private generator: ContentGenerator;
  private processor: ContentProcessor;
  private validator: ContentValidator;

  constructor() {
    this.generator = new ContentGenerator();
    this.processor = new ContentProcessor();
    this.validator = new ContentValidator();
  }

  /**
   * Executes the complete textbook generation workflow
   */
  public async executeWorkflow(config: TextbookConfig, options: WorkflowOptions = {}): Promise<WorkflowResult> {
    const startTime = Date.now();
    const taskId = generateId('workflow');
    const errors: string[] = [];
    const warnings: string[] = [];

    // Set default options
    const workflowOptions: Required<WorkflowOptions> = {
      validateContent: options.validateContent ?? true,
      processContent: options.processContent ?? true,
      generateOutput: options.generateOutput ?? true,
      outputFormats: options.outputFormats ?? ['html', 'markdown'],
      outputPath: options.outputPath ?? './output'
    };

    let result: Partial<WorkflowResult> = {
      taskId,
      config,
      errors: [],
      warnings: [],
      executionTime: 0
    };

    try {
      // Step 1: Validate configuration
      if (workflowOptions.validateContent) {
        console.log(`[${taskId}] Validating textbook configuration...`);
        const configValidation = this.validator.validateTextbookConfig(config);
        if (!configValidation.isValid) {
          errors.push(...configValidation.issues);
          warnings.push(...configValidation.suggestions);
          return {
            success: false,
            ...result,
            errors,
            warnings,
            executionTime: Date.now() - startTime
          } as WorkflowResult;
        }
      }

      // Step 2: Generate textbook content
      console.log(`[${taskId}] Generating textbook content...`);
      const generationResult: GenerationResult = await this.generator.generateTextbook(config);

      if (!generationResult.success) {
        errors.push(...generationResult.errors);
        warnings.push(...generationResult.warnings);
        return {
          success: false,
          ...result,
          errors,
          warnings,
          executionTime: Date.now() - startTime
        } as WorkflowResult;
      }

      result.generatedTextbook = generationResult.textbook;

      // Step 3: Process content (if enabled)
      let processedTextbook: ProcessedTextbook | undefined;
      if (workflowOptions.processContent && generationResult.textbook) {
        console.log(`[${taskId}] Processing textbook content...`);
        processedTextbook = await this.processor.processTextbook(
          generationResult.textbook.config,
          generationResult.textbook.chapters
        );
        result.processedTextbook = processedTextbook;
      }

      // Step 4: Generate output in specified formats (if enabled)
      const outputPaths: string[] = [];
      if (workflowOptions.generateOutput && generationResult.textbook) {
        console.log(`[${taskId}] Generating output in formats: ${workflowOptions.outputFormats.join(', ')}...`);

        for (const format of workflowOptions.outputFormats) {
          const formatOutputPath = path.join(workflowOptions.outputPath, format);
          if (!fs.existsSync(formatOutputPath)) {
            fs.mkdirSync(formatOutputPath, { recursive: true });
          }

          await this.generator.generateOutput(generationResult.textbook, format, formatOutputPath);
          outputPaths.push(formatOutputPath);
        }

        result.outputPaths = outputPaths;
      }

      // Step 5: Perform final validation on generated content
      if (workflowOptions.validateContent && generationResult.textbook?.chapters) {
        console.log(`[${taskId}] Performing final content validation...`);
        const finalValidation = this.validator.validateContentCoherence(generationResult.textbook.chapters);
        if (!finalValidation.isValid) {
          warnings.push(...finalValidation.issues);
        }
      }

      console.log(`[${taskId}] Textbook generation workflow completed successfully!`);

      return {
        success: true,
        ...result,
        errors,
        warnings,
        executionTime: Date.now() - startTime
      } as WorkflowResult;
    } catch (error) {
      errors.push(error instanceof Error ? error.message : String(error));
      return {
        success: false,
        ...result,
        errors,
        warnings,
        executionTime: Date.now() - startTime
      } as WorkflowResult;
    }
  }

  /**
   * Executes a simplified workflow for quick generation
   */
  public async executeQuickWorkflow(config: TextbookConfig): Promise<WorkflowResult> {
    return this.executeWorkflow(config, {
      validateContent: true,
      processContent: true,
      generateOutput: true,
      outputFormats: ['html', 'markdown'],
      outputPath: './output'
    });
  }

  /**
   * Validates the entire workflow before execution
   */
  public validateWorkflow(config: TextbookConfig): ValidationResult {
    // Validate the configuration
    const configValidation = this.validator.validateTextbookConfig(config);
    if (!configValidation.isValid) {
      return configValidation;
    }

    // Additional workflow-specific validations could go here
    // For example, checking if required output directories are writable, etc.

    return {
      isValid: true,
      issues: [],
      suggestions: []
    };
  }

  /**
   * Estimates the time required for the workflow
   */
  public estimateExecutionTime(config: TextbookConfig): number {
    // Rough estimation based on number of modules and chapters
    const baseTime = 1000; // 1 second base time
    const modulesTime = (config.modules?.length || 0) * 2000; // 2 seconds per module
    const chaptersPerModule = config.modules?.reduce((sum, mod) => sum + (mod.chapters?.length || 1), 0) || 0;
    const chaptersTime = chaptersPerModule * 1000; // 1 second per chapter

    return baseTime + modulesTime + chaptersTime;
  }

  /**
   * Creates a progress tracker for long-running workflows
   */
  public createProgressTracker(taskId: string): (progress: number, message: string) => void {
    return (progress: number, message: string) => {
      console.log(`[${taskId}] Progress: ${Math.round(progress * 100)}% - ${message}`);
    };
  }
}