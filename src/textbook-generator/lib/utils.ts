/**
 * Utility functions for the Physical AI & Humanoid Robotics Textbook Generator
 */

import fs from 'fs';
import path from 'path';
import { Chapter, LearningObjective, Exercise, Quiz, QuizQuestion } from '../models/chapter';

/**
 * Generates a unique ID
 */
export function generateId(prefix: string = ''): string {
  const timestamp = Date.now();
  const random = Math.random().toString(36).substr(2, 9);
  return `${prefix ? prefix + '_' : ''}${timestamp}_${random}`;
}

/**
 * Deep clone an object
 */
export function deepClone<T>(obj: T): T {
  return JSON.parse(JSON.stringify(obj));
}

/**
 * Sanitizes text content by removing potentially harmful characters
 */
export function sanitizeText(text: string): string {
  if (!text) return '';

  // Remove potentially dangerous characters/sequences
  return text
    .replace(/<script\b[^<]*(?:(?!<\/script>)<[^<]*)*<\/script>/gi, '') // Remove script tags
    .replace(/javascript:/gi, '') // Remove javascript: protocol
    .replace(/vbscript:/gi, '') // Remove vbscript: protocol
    .replace(/on\w+="[^"]*"/gi, '') // Remove event handlers
    .trim();
}

/**
 * Formats text as markdown
 */
export function formatAsMarkdown(content: string): string {
  // Basic markdown formatting
  return content
    .replace(/\*\*(.*?)\*\*/g, '**$1**') // Bold
    .replace(/\*(.*?)\*/g, '*$1*') // Italic
    .replace(/^(\d+)\. /gm, '$1. ') // Numbered lists
    .replace(/^- /gm, '- '); // Bullet points
}

/**
 * Checks if a string is a valid URL
 */
export function isValidUrl(string: string): boolean {
  try {
    const url = new URL(string);
    return url.protocol === 'http:' || url.protocol === 'https:';
  } catch (_) {
    return false;
  }
}

/**
 * Saves content to a file
 */
export function saveToFile(content: string, filePath: string): void {
  const directory = path.dirname(filePath);
  if (!fs.existsSync(directory)) {
    fs.mkdirSync(directory, { recursive: true });
  }
  fs.writeFileSync(filePath, content);
}

/**
 * Reads content from a file
 */
export function readFromFile(filePath: string): string {
  if (!fs.existsSync(filePath)) {
    throw new Error(`File does not exist: ${filePath}`);
  }
  return fs.readFileSync(filePath, 'utf8');
}

/**
 * Formats text to title case
 */
export function toTitleCase(text: string): string {
  return text.toLowerCase()
    .split(' ')
    .map((word, index) => {
      if (word.length === 0) return word;
      // Don't capitalize small words unless they're the first word
      if (index === 0 || !['a', 'an', 'and', 'as', 'at', 'but', 'by', 'for', 'if', 'in', 'nor', 'of', 'on', 'or', 'so', 'the', 'to', 'up', 'yet'].includes(word)) {
        return word.charAt(0).toUpperCase() + word.slice(1);
      }
      return word;
    })
    .join(' ');
}

/**
 * Calculates reading time in minutes based on word count
 * @param text The text to analyze
 * @param wpm Average words per minute (default 200)
 * @returns Estimated reading time in minutes
 */
export function calculateReadingTime(text: string, wpm: number = 200): number {
  if (!text) return 0;
  const words = text.trim().split(/\s+/).length;
  return Math.ceil(words / wpm);
}

/**
 * Escapes HTML characters in a string
 */
export function escapeHtml(text: string): string {
  if (!text) return '';
  return text
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;')
    .replace(/'/g, '&#039;');
}

/**
 * Generates a slug from a string
 */
export function generateSlug(text: string): string {
  return text
    .toLowerCase()
    .replace(/[^\w\s-]/g, '') // Remove special characters
    .replace(/[\s_-]+/g, '-') // Replace spaces and underscores with hyphens
    .replace(/^-+|-+$/g, ''); // Remove leading/trailing hyphens
}

/**
 * Validates email format
 */
export function isValidEmail(email: string): boolean {
  const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/;
  return emailRegex.test(email);
}

/**
 * Checks if a value is empty
 */
export function isEmpty(value: any): boolean {
  if (value === null || value === undefined) return true;
  if (typeof value === 'string') return value.trim().length === 0;
  if (Array.isArray(value)) return value.length === 0;
  if (typeof value === 'object') return Object.keys(value).length === 0;
  return false;
}

/**
 * Formats a number with commas
 */
export function formatNumber(num: number): string {
  return num.toString().replace(/\B(?=(\d{3})+(?!\d))/g, ',');
}

/**
 * Waits for a specified amount of time
 */
export function sleep(ms: number): Promise<void> {
  return new Promise(resolve => setTimeout(resolve, ms));
}

/**
 * Generates a random number between min and max (inclusive)
 */
export function randomInt(min: number, max: number): number {
  return Math.floor(Math.random() * (max - min + 1)) + min;
}

/**
 * Removes duplicate values from an array
 */
export function uniqueArray<T>(array: T[]): T[] {
  return Array.from(new Set(array));
}

/**
 * Capitalizes the first letter of a string
 */
export function capitalizeFirstLetter(str: string): string {
  if (!str) return str;
  return str.charAt(0).toUpperCase() + str.slice(1);
}