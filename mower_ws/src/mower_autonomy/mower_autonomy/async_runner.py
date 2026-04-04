#!/usr/bin/env python3
import asyncio
import threading
import time
from typing import Coroutine, Any, Optional


class AsyncRunner:
    """
    Runs an asyncio loop in a background thread so ROS timers/callbacks
    can schedule async websocket calls reliably.

    This version shuts down cleanly:
      - cancels pending tasks
      - closes the loop
      - joins the thread
    """

    def __init__(self):
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None

    def start(self):
        if self._thread is not None:
            return

        def _run():
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._loop = loop
            try:
                loop.run_forever()
            finally:
                # Cancel all pending tasks cleanly
                try:
                    pending = asyncio.all_tasks(loop)
                    for t in pending:
                        t.cancel()
                    if pending:
                        loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
                except Exception:
                    pass
                try:
                    loop.close()
                except Exception:
                    pass

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()

        # Wait for loop creation
        while self._loop is None:
            time.sleep(0.01)

    def submit(self, coro: Coroutine[Any, Any, Any]):
        if self._loop is None:
            raise RuntimeError("AsyncRunner not started")
        return asyncio.run_coroutine_threadsafe(coro, self._loop)

    def stop(self):
        if self._loop is None:
            return
        try:
            self._loop.call_soon_threadsafe(self._loop.stop)
        except Exception:
            pass

        # join thread so loop is closed after tasks are cancelled
        try:
            if self._thread is not None:
                self._thread.join(timeout=1.0)
        except Exception:
            pass

        self._loop = None
        self._thread = None

