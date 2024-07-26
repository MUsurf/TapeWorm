
"""
State_Machine class
Handles the tasks and task list

does not use ros directly

Maintainer: Luke Deffenbaugh
"""

from state_task import Task_Item


class State_Machine():
    def __init__(self, tasks : tuple):
        """initialize State Machine to make handling the state machine easier

        Args:
            tasks (tuple): tuple containing all tasks to be completed Type: Task_Item
        """
        self.tasklist : tuple = tasks

        # Create each of the targets for the PID controllers
        # X, Y, Z, Forward/Backward, Left/Right, Up/Down
        self.x_target = 0
        self.y_target = 0
        self.z_target = 0
        self.fb_target = 0
        self.lr_target = 0
        self.depth_target = 0

    def pidTargetTopic(self) -> str:
        """Create the string to be sent to the topic
           This string is intended to be read with regex

           contains pid targets

        Returns:
            str: topic string
        """
        output = f"x:{self.x_target} y:{self.y_target} z:{self.z_target} fb:{self.fb_target} lr:{self.lr_target} ud:{self.ud_target}"
        return output
        
    def Status(self):
        """Return a decimal percentage representing the completion of all tasks collectively

        Returns:
            float: percentage (0-1)
        """
        return sum([task.status for task in self.tasklist]) / len(self.tasklist)

    def Call(self, name : str) -> bool:
        """ Execute a specific task with the given name using its next callback funciton.

        Args:
            name (str): name of the task to be executed

        Returns:
            bool: True if task was successfully executed | False if task is already complete or does not exist
        """
        return False if (task := self.checkName(name)) is None else task.Execute()
        return True
    
    def checkName(self, name : str):
        """Check if a task with the given name exists.
        Returns the task if this condition is met.

        Args:
            name (str): Name to be checked against the list of tasks

        Returns:
            Task: Task with the given name | None if no task is found by that name
        """
        return next((task for task in self.tasklist if name is task.name), None)