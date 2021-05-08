from abc import ABC, abstractmethod

class Subject(ABC):
    """
    The Subject interface declares the update method, used by subjects.
    """

    @abstractmethod
    def update(self, observer, key=None):
        """
        Receive update from subject.
        """
        pass