'''
ROS 
node: state_machine
Publishes:
Subscribes:


Maintainer: Luke Deffenbaugh
'''

from tasks import task_list
from state_task import Task_Item as Task
import rospy

class State_Machine():
    def __init__(self, tasks : tuple(Task)):
        """initialize State Machine to make handling the state machine easier

        Args:
            tasks (tuple): tuple containing all tasks to be completed
        """
        self.tasklist = tasks
        
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
        
        
    
    def checkName(self, name : str):
        """Check if a task with the given name exists.
        Returns the task if this condition is met.

        Args:
            name (str): Name to be checked against the list of tasks

        Returns:
            Task: Task with the given name | None if no task is found by that name
        """
        return next((task for task in self.tasklist if name is task.name), None)

        


if __name__ == "__main__":
    # create a statemachine object using a tuple of task objects from the dictionary `task_list`
    state = StateMachine((Task(key, value) for key, value in task_list.items()))
    
    # Ros setup
    rospy.init_node('state_machine', anonymous = True)
    # TODO: When a topic to subscribe to exists... add it here
    # TODO: publish the information about tasks
    # TODO: Write out the steps to follow
