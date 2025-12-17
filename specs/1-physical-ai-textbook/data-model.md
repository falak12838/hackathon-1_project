# Data Model: Physical AI & Humanoid Robotics Textbook Generator

## Entities

### Chapter
- **name**: string - The title of the chapter
- **id**: string - Unique identifier for the chapter
- **content**: string - The main content of the chapter
- **order**: number - The position of the chapter in the textbook
- **learningObjectives**: LearningObjective[] - List of learning objectives for this chapter
- **exercises**: Exercise[] - List of exercises for this chapter
- **quizzes**: Quiz[] - List of quizzes for this chapter
- **metadata**: object - Additional metadata like difficulty level, estimated reading time

### LearningObjective
- **id**: string - Unique identifier for the learning objective
- **chapterId**: string - Reference to the parent chapter
- **description**: string - Detailed description of what the student should learn
- **measurable**: boolean - Whether the objective can be measured/tested
- **tags**: string[] - Keywords associated with this objective

### Exercise
- **id**: string - Unique identifier for the exercise
- **chapterId**: string - Reference to the parent chapter
- **title**: string - Title of the exercise
- **description**: string - Detailed description of the exercise
- **difficulty**: string - Level of difficulty (beginner, intermediate, advanced)
- **solution**: string - Solution or answer to the exercise
- **type**: string - Type of exercise (coding, theoretical, practical)

### Quiz
- **id**: string - Unique identifier for the quiz
- **chapterId**: string - Reference to the parent chapter
- **title**: string - Title of the quiz
- **questions**: QuizQuestion[] - List of questions in the quiz
- **timeLimit**: number - Time limit for completing the quiz (in minutes)

### QuizQuestion
- **id**: string - Unique identifier for the question
- **quizId**: string - Reference to the parent quiz
- **question**: string - The actual question text
- **options**: string[] - Multiple choice options (if applicable)
- **correctAnswer**: string - The correct answer
- **explanation**: string - Explanation of the correct answer
- **type**: string - Type of question (multiple-choice, true-false, short-answer)

### Module
- **id**: string - Unique identifier for the module
- **name**: string - Name of the module
- **description**: string - Description of what the module covers
- **chapters**: Chapter[] - List of chapters in this module
- **order**: number - The position of the module in the textbook
- **prerequisites**: string[] - List of module IDs that should be completed before this one

### TextbookConfig
- **id**: string - Unique identifier for the configuration
- **title**: string - Title of the textbook
- **author**: string - Author of the textbook
- **targetAudience**: string - Intended audience (undergraduate, graduate, professional)
- **difficultyLevel**: string - Overall difficulty level
- **modules**: Module[] - List of modules to include in the textbook
- **outputFormats**: string[] - List of desired output formats
- **customizationOptions**: object - Additional customization settings

## Relationships

- A **TextbookConfig** contains multiple **Module** entities
- A **Module** contains multiple **Chapter** entities
- A **Chapter** contains multiple **LearningObjective**, **Exercise**, and **Quiz** entities
- A **Quiz** contains multiple **QuizQuestion** entities

## Validation Rules

- Chapter titles must be unique within a textbook
- Learning objectives must be specific and measurable
- Exercises must have corresponding solutions
- Quiz questions must have exactly one correct answer
- Modules must have unique names
- Textbook configurations must specify at least one module