"""Yield preparation authority when FCU state changes outside our commands."""
from contextlib import contextmanager
import threading


OVERRIDE = '차량 상태가 외부에서 변경되어 웹 준비를 중단했습니다.'


class VehicleAuthority:
    def __init__(self):
        self.lock = threading.RLock()
        self.revision = 0
        self.previous = None
        self.expected = {}

    def observe(self, state):
        with self.lock:
            if not state.get('connected'):
                # A reconnect must not revive an outstanding preparation.
                if self.previous is not None:
                    self.revision += 1
                self.previous = None
                self.expected.clear()
                return
            current = {key: state[key] for key in ('armed', 'mode')}
            if self.previous is not None:
                for key, value in current.items():
                    if value != self.previous[key]:
                        if key in self.expected and value == self.expected[key]:
                            del self.expected[key]
                        else:
                            self.revision += 1
                            self.expected.clear()
            self.previous = current

    def check(self, revision):
        with self.lock:
            if revision != self.revision:
                raise RuntimeError(OVERRIDE)

    @contextmanager
    def expect(self, revision, **changes):
        with self.lock:
            self.check(revision)
            self.expected.update(changes)
        try:
            yield
        finally:
            with self.lock:
                for key in changes:
                    self.expected.pop(key, None)
