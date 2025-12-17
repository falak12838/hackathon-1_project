#!/usr/bin/env node

/**
 * CLI for the Physical AI & Humanoid Robotics Textbook Generator
 * Provides commands for generating textbooks with chapters, learning objectives, exercises, and quizzes
 */

import { Command } from 'commander';
import { TextbookConfigModel } from '../models/textbook-config';
import { TextbookWorkflowOrchestrator } from '../services/textbook-workflow-orchestrator';
import { ContentValidator } from '../services/validator';
import fs from 'fs';
import path from 'path';

const program = new Command();

program
  .name('textbook-generator')
  .description('CLI for generating Physical AI & Humanoid Robotics textbooks')
  .version('1.0.0');

program
  .command('generate')
  .description('Generate a textbook from a configuration file')
  .option('-c, --config <path>', 'Path to the textbook configuration file')
  .option('-o, --output <path>', 'Output directory for the generated textbook')
  .option('--format <formats...>', 'Output formats (html, markdown, pdf)')
  .option('--no-validate', 'Skip content validation')
  .option('--no-process', 'Skip content processing')
  .action(async (options) => {
    try {
      console.log('Starting textbook generation...');

      // Load configuration
      let config: any;
      if (options.config) {
        const configPath = path.resolve(options.config);
        if (!fs.existsSync(configPath)) {
          throw new Error(`Configuration file not found: ${configPath}`);
        }

        const configContent = fs.readFileSync(configPath, 'utf8');
        config = JSON.parse(configContent);
      } else {
        // Use a default configuration
        console.log('No configuration file specified, using default configuration...');
        config = {
          title: 'Physical AI & Humanoid Robotics - Default Textbook',
          author: 'AI Generated',
          targetAudience: 'undergraduate',
          difficultyLevel: 'intermediate',
          modules: [],
          outputFormats: ['html', 'markdown']
        };
      }

      // Create config model
      const configModel = new TextbookConfigModel(config);

      // Create workflow orchestrator
      const orchestrator = new TextbookWorkflowOrchestrator();

      // Execute the workflow
      const result = await orchestrator.executeWorkflow(configModel, {
        validateContent: options.validate,
        processContent: options.process,
        generateOutput: true,
        outputFormats: options.format || config.outputFormats || ['html', 'markdown'],
        outputPath: options.output || './output'
      });

      if (result.success) {
        console.log(`✓ Textbook generation completed successfully!`);
        console.log(`Task ID: ${result.taskId}`);
        console.log(`Execution time: ${result.executionTime}ms`);
        if (result.outputPaths) {
          console.log(`Output generated at: ${result.outputPaths.join(', ')}`);
        }
      } else {
        console.error('✗ Textbook generation failed:');
        result.errors.forEach(error => console.error(`- ${error}`));
        result.warnings.forEach(warning => console.error(`- Warning: ${warning}`));
        process.exit(1);
      }
    } catch (error) {
      console.error('Error during textbook generation:', error instanceof Error ? error.message : String(error));
      process.exit(1);
    }
  });

program
  .command('validate')
  .description('Validate a textbook configuration file')
  .option('-c, --config <path>', 'Path to the textbook configuration file')
  .action((options) => {
    try {
      if (!options.config) {
        console.error('Please specify a configuration file with -c or --config');
        process.exit(1);
      }

      const configPath = path.resolve(options.config);
      if (!fs.existsSync(configPath)) {
        throw new Error(`Configuration file not found: ${configPath}`);
      }

      const configContent = fs.readFileSync(configPath, 'utf8');
      const config = JSON.parse(configContent);

      const validator = new ContentValidator();
      const validation = validator.validateTextbookConfig(config);

      if (validation.isValid) {
        console.log('✓ Configuration is valid');
      } else {
        console.log('✗ Configuration validation failed:');
        validation.issues.forEach(issue => console.log(`- ${issue}`));
        validation.suggestions.forEach(suggestion => console.log(`- Suggestion: ${suggestion}`));
      }
    } catch (error) {
      console.error('Error during validation:', error instanceof Error ? error.message : String(error));
      process.exit(1);
    }
  });

program
  .command('init')
  .description('Initialize a new textbook configuration')
  .option('-o, --output <path>', 'Output path for the configuration file')
  .action((options) => {
    try {
      const defaultConfig = {
        title: 'Physical AI & Humanoid Robotics Textbook',
        author: 'Your Name',
        targetAudience: 'undergraduate',
        difficultyLevel: 'intermediate',
        modules: [
          {
            id: 'introduction',
            name: 'Introduction to Physical AI',
            description: 'Basic concepts of Physical AI and its applications',
            chapters: [],
            order: 1,
            prerequisites: []
          }
        ],
        outputFormats: ['html', 'markdown'],
        customizationOptions: {
          includeSolutions: true,
          generateQuizzes: true,
          addExercises: true,
          includeLearningObjectives: true
        }
      };

      const outputPath = options.output || './textbook-config.json';
      fs.writeFileSync(outputPath, JSON.stringify(defaultConfig, null, 2));
      console.log(`Default textbook configuration created at: ${outputPath}`);
    } catch (error) {
      console.error('Error during initialization:', error instanceof Error ? error.message : String(error));
      process.exit(1);
    }
  });

// If no command is provided, show help
if (!process.argv.slice(2).length) {
  program.outputHelp();
}

program.parse();