# ROS State Machine Integration

This ROS package implements a state machine in Python, designed to manage and execute tasks with ROS integration.

## Overview

The state machine manages tasks defined as instances of the `Task_Item` class. Each task consists of sequential milestones to be executed, facilitating robust task management in ROS environments.

## Components

### `Task_Item` Class

The `Task_Item` class encapsulates individual tasks with the following attributes:

- **Attributes**:
  - `name`: Name of the task.
  - `milestones`: Tuple of milestone callback functions.
  - `_current_milestone`: Index of the current milestone being executed.
  - `status`: Completion status of the task.

- **Methods**:
  - `Execute()`: Executes the next milestone in the task sequence.
  - `__updateStatus()`: Updates the task completion status.

### `State_Machine` Class

The `State_Machine` class manages a collection of tasks and provides methods to interact with them:

- **Attributes**:
  - `tasklist`: Tuple containing all tasks to be managed.

- **Methods**:
  - `Status()`: Returns the overall completion percentage of all tasks.
  - `Call(name)`: Executes a specific task by name using its next callback function.
  - `checkName(name)`: Checks if a task with the given name exists in `tasklist`.

### ROS Integration

This package interacts with ROS through the `state_machine` node:

- **Publishes**: [List topics for data publication, if applicable]
- **Subscribes**: [List topics for data subscription, if applicable]

## Example Task Setup

To add a new task:

1. Define milestone functions for the task.
2. Create a tuple of milestone functions in the order they should be executed.
3. Add the task to `task_list` in `tasks.py`:

```python
# Example task setup
def task_milestone_1() -> None:
    """Instructions for milestone 1."""
    pass

def task_milestone_2() -> None:
    """Instructions for milestone 2."""
    pass

# Tuple of milestone functions
example_task_milestones = (task_milestone_1, task_milestone_2)

# Add to task_list
task_list = {"example_task": example_task_milestones}
```

**Maintainer:** Luke Deffenbaugh
