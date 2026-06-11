import threading
from qr_core.event import QREvent
from qr_core.event_manager import EventQueue, get_default_event_manager
from typing import Optional, Callable


class QRModule(object):
    EVENTS_RECEIVES = {}
    EVENTS_GENERATES = []
    QUERIES_RECEIVES = {}
    QUERIES_GENERATES = []

    def __init__(self):
        self._event_manager = get_default_event_manager()
        self._event_subscribers = list()

    @property
    def event_manager(self):
        return self._event_manager

    def emit_event(self, event: QREvent):
        """Emit an event."""
        self.event_manager.emit_event(event)

    def subscribe_to_event(self, event_type, callback, max_queue_size: int = 10):
        """Subscribe to an event."""
        dq = self.event_manager.subscribe_event(event_type, max_queue_size)
        subscriber = ModuleEventSubscriber(self, dq, callback)

        thread = threading.Thread(target=subscriber, daemon=True)
        thread.start()
        subscriber.set_thread(thread)
        self._event_subscribers.append(subscriber)
        return subscriber


class ModuleEventSubscriber(object):
    def __init__(self, module, dq, callback):
        self.module = module
        self.dq = dq
        self.callback = callback
        self.thread = None

    module: QRModule
    """The module that the subscriber is attached to."""

    dq: EventQueue
    """The event queue that the subscriber is using to receive events."""

    callback: Callable[[QREvent], None]

    thread: Optional[threading.Thread] = None

    def set_thread(self, thread):
        self.thread = thread

    def __call__(self):
        while True:
            event = self.dq.pop()
            self.callback(event)
