# Data Model: Module 1 - The Robotic Nervous System (ROS 2)

## Overview
This document defines the conceptual data model for Module 1 of the AI Robotics Technical Book, covering ROS 2 fundamentals for humanoid robot control.

## Core Entities

### Module
- **Description**: A self-contained section of the technical book focusing on a specific aspect of AI robotics
- **Fields**:
  - moduleId: unique identifier for the module
  - title: human-readable title of the module
  - description: brief overview of module content and objectives
  - learningOutcomes: list of measurable outcomes students should achieve
  - chapters: collection of chapters contained in the module
  - prerequisites: knowledge requirements before starting the module
- **Relationships**: Contains multiple chapters; connected to supporting documentation

### Chapter
- **Description**: A section within a module that covers related topics and concepts
- **Fields**:
  - chapterId: unique identifier for the chapter
  - title: human-readable title of the chapter
  - description: brief overview of chapter content
  - topics: list of key topics covered in the chapter
  - objectives: learning objectives specific to this chapter
  - sections: collection of sections within the chapter
- **Relationships**: Belongs to one module; contains multiple sections

### Section
- **Description**: A subsection within a chapter covering a specific topic in detail
- **Fields**:
  - sectionId: unique identifier for the section
  - title: human-readable title of the section
  - content: detailed explanation of the topic
  - examples: code snippets or practical examples
  - exercises: practice problems or lab exercises
- **Relationships**: Belongs to one chapter; contains multiple exercises/examples

### Topic
- **Description**: A specific subject or concept within the ROS 2 learning curriculum
- **Fields**:
  - topicId: unique identifier for the topic
  - name: name of the topic (e.g., "Nodes", "Topics", "Services")
  - description: detailed explanation of the topic
  - resources: related documentation, tutorials, or references
  - prerequisites: other topics that should be understood first
- **Relationships**: Related to multiple sections; may depend on other topics

### LearningOutcome
- **Description**: A measurable competency that students should achieve
- **Fields**:
  - outcomeId: unique identifier for the outcome
  - description: specific skill or knowledge to be acquired
  - assessmentMethod: how the outcome will be validated
  - relatedTopics: topics that contribute to this outcome
- **Relationships**: Associated with one module; connected to multiple topics

### Exercise
- **Description**: A hands-on activity for students to practice concepts
- **Fields**:
  - exerciseId: unique identifier for the exercise
  - title: brief title of the exercise
  - description: detailed instructions for the exercise
  - difficulty: level of complexity (beginner, intermediate, advanced)
  - requiredComponents: ROS 2 components needed for the exercise
  - validationSteps: steps to verify completion
  - estimatedTime: time needed to complete the exercise
- **Relationships**: Belongs to one section; uses specific ROS 2 components

### Reference
- **Description**: An authoritative source for information in the module
- **Fields**:
  - referenceId: unique identifier for the reference
  - title: title of the referenced material
  - authors: creators of the referenced material
  - publicationDate: when the reference was published
  - type: category (academic paper, official documentation, tutorial, etc.)
  - url: link to the reference
  - citation: properly formatted citation in APA/IEEE style
  - relevance: why this reference is relevant to the module
- **Relationships**: Used by multiple sections or topics

## ROS 2-Specific Entities

### ROS2Node
- **Description**: A fundamental unit of computation in ROS 2
- **Fields**:
  - nodeId: unique identifier for the node
  - name: the name of the node
  - namespace: optional namespace for the node
  - publishers: list of publishers hosted by the node
  - subscribers: list of subscribers hosted by the node
  - services: list of services offered by the node
  - clients: list of service clients in the node
  - actions: list of action servers/clients in the node
- **Relationships**: May be implemented in multiple examples/exercises

### ROS2Topic
- **Description**: Communication channel for asynchronous data transfer
- **Fields**:
  - topicId: unique identifier for the topic
  - name: the name of the topic
  - messageType: type of messages sent over the topic
  - publishers: nodes publishing to this topic
  - subscribers: nodes subscribed to this topic
  - qosProfile: Quality of Service settings for the topic
- **Relationships**: Connected to multiple publisher/subscriber nodes

### ROS2Service
- **Description**: Synchronous request-response communication pattern
- **Fields**:
  - serviceId: unique identifier for the service
  - name: the name of the service
  - serviceType: type defining the request/response interface
  - serverNode: node hosting the service server
  - clientNodes: list of nodes calling this service
- **Relationships**: Connected to service server and clients

### ROS2Action
- **Description**: Extended services for long-running tasks with feedback
- **Fields**:
  - actionId: unique identifier for the action
  - name: the name of the action
  - actionType: type defining the goal/feedback/result interface
  - serverNode: node hosting the action server
  - clientNodes: list of nodes using this action
  - feedbackType: type for feedback messages
  - resultType: type for result messages
- **Relationships**: Connected to action server and clients

### ROS2Parameter
- **Description**: Configuration value accessible to ROS 2 nodes
- **Fields**:
  - parameterId: unique identifier for the parameter
  - name: the name of the parameter
  - type: data type of the parameter value
  - defaultValue: value used if not explicitly set
  - currentValue: current value of the parameter
  - description: explanation of what the parameter controls
- **Relationships**: Associated with specific nodes

### URDFModel
- **Description**: Unified Robot Description Format model for humanoid robots
- **Fields**:
  - modelId: unique identifier for the model
  - name: the name of the robot model
  - joints: collection of joint definitions
  - links: collection of link definitions
  - materials: collection of material definitions
  - gazeboExtensions: Gazebo-specific extensions to the model
  - validationRules: rules for checking model correctness
- **Relationships**: Used in simulation exercises and examples

## Relationships

- Module (1) → Chapter (Many): One module contains multiple chapters
- Chapter (1) → Section (Many): One chapter contains multiple sections
- Topic (Many) → Section (Many): Topics span across multiple sections (many-to-many)
- Section (1) → Exercise (Many): One section may contain multiple exercises
- Module (1) → LearningOutcome (Many): One module has multiple learning outcomes
- Topic (Many) → LearningOutcome (Many): Topics contribute to multiple learning outcomes
- Exercise (1) → ROS2Node/Topic/Service/Action/Parameter (Many): Exercises involve multiple ROS 2 components

## Validation Rules

1. Each Module must have 4-6 Chapters as specified in the requirements (FR-003)
2. Each Chapter must have a title, description, and list of key topics (FR-004)
3. Each Module must define 5-8 measurable learning outcomes (FR-002)
4. All exercises must be reproducible in a standard ROS 2 environment (SC-003)
5. At least 40% of technical claims must be supported by official documentation or academic references (SC-004)
6. Content must strictly adhere to ROS 2 middleware topics as defined in the goal (FR-008)

## State Transitions

For Exercise entity:
- Draft → Review → Approved → Published → Deprecated
- Each transition requires validation of content accuracy and reproducibility

## Docusaurus Compatibility

The data model is structured to map directly to Docusaurus documentation hierarchy:
- Module → Docusaurus Category
- Chapter → Docusaurus Doc
- Section → Content sections within each doc
- Exercise → Interactive examples or tutorials