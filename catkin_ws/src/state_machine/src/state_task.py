'''
Maintainer: Luke Deffenbaugh

File does NOT use ROS directly
'''

class Task_Item():
    def __init__(self, name: str, milestones: Tuple[Callable[[], None], ...]):
        """Initialize Task object with milestone callback functions

        Args:
            name (str): name to give the overall task
            milestones (Tuple[Callable[[], None], ...]): tuple of callable functions to completed in sequence. (Should have no attributes and no returns)
        """

        self.name = name
        self.milestones = milestones
        self._current_milelstone = 0
        self.status = _current_milelstone / len(self.milestones)

    def Execute(self) -> bool:
        """ Execute the next milestone in the task

        Returns:
            bool: True if a milestone was completed | False if task is already complete
        """
        if self.status < 1:
            self.milestones[self._current_milelstone]() # Execute milestone callback
            self._current_milelstone += 1
            self.__updateStatus()
            return True
        else:
            return False 

    def __updateStatus(self):
        """Update the percentage status currently completed
        """
        self.status = self._current_milelstone / len(self.milestones)



