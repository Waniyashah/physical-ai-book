# Data Model

### Book

- **title**: String
- **subtitle**: String
- **modules**: has-many Module

### Module

- **title**: String
- **chapters**: has-many Chapter
- **learning_objectives**: Array of String

### Chapter

- **title**: String
- **content**: Markdown/MDX
- **code_examples**: has-many CodeExample
- **diagrams**: has-many Diagram
- **l1_l7_spec**: has-one Specification

### CodeExample

- **language**: String (e.g., "python", "xml", "launch")
- **code**: String

### Diagram

- **title**: String
- **format**: SVG
- **source**: String (path to SVG file)

### Specification

- **title**: String
- **levels**: Array of String (L1 to L7)
