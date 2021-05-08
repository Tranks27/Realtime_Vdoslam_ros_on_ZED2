from abc import ABC, abstractmethod
from random import randrange


class BaseObserver(ABC):
    """
    The Observer interface declares a set of methods for managing subscribers.
    """

    @abstractmethod
    def attach(self, subject):
        """
        Attach a subject to the observer.
        """
        pass

    @abstractmethod
    def detach(self, observer):
        """
        Detach a subject from the observer.
        """
        pass

    @abstractmethod
    def notify(self):
        """
        Notify all subjects about an event.
        """
        pass


class Observer(BaseObserver):
    """
    The Observer owns some important state and notifies subjects when the state
    changes.
    """

    _state = None
    """
    For the sake of simplicity, the Observers's state, essential to all
    subscribers, is stored in this variable.
    """

    _subjects = []

    """
    List of subscribers. In real life, the list of subscribers can be stored
    more comprehensively (categorized by event type, etc.).
    """

    def attach(self, subject):
        print("Subject: Attached an observer.")
        self._subjects.append(subject)

    def detach(self, subject):
        self._subjects.remove(subject)

    """
    The subscription management methods.
    """

    def notify(self, key=None):
        """
        Trigger an update in each subscriber.
        """

        print("Subject: Notifying observers...")
        for subject in self._subjects:
            subject.update(self, key)
