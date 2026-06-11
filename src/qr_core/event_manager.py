import threading
from qr_core.event import QREvent
from collections import defaultdict, deque


class EventQueue(object):
    def __init__(self, max_size: int = 10):
        self._dq = deque(maxlen=max_size)
        self._condition = threading.Condition()

    def put(self, event: QREvent):
        with self._condition:
            self._dq.append(event)
            self._condition.notify()

    def pop(self):
        with self._condition:
            while not self._dq:
                self._condition.wait()
            return self._dq.popleft()


class EventManager(object):
    def __init__(self):
        self.event_queues = defaultdict(list)

    event_queues: dict[type, list[EventQueue]]

    def emit_event(self, event):
        for queue in self.event_queues[event.__class__]:
            queue.put(event)

    def subscribe_event(self, event, max_queue_size=0) -> EventQueue:
        dq = EventQueue(max_queue_size)
        self.event_queues[event].append(dq)
        return dq

_default_event_manager = None


def get_default_event_manager():
    global _default_event_manager
    if _default_event_manager is None:
        _default_event_manager = EventManager()
    return _default_event_manager
