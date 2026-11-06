"""Project-local synchronization for Foxy's ActionClient request bookkeeping."""
import threading

from rclpy.action import ActionClient


class SynchronizedActionClient(ActionClient):
    """Register each sequence before another executor thread consumes its reply.

    Foxy's response handlers contain no suspension points except user feedback.
    Dispatch feedback outside the lock so an async user callback can await work.
    No system ROS package is patched.
    """
    def __init__(self, *args, **kwargs):
        self._request_lock = threading.RLock()
        super().__init__(*args, **kwargs)

    def send_goal_async(self, *args, **kwargs):
        with self._request_lock:
            return super().send_goal_async(*args, **kwargs)

    def _cancel_goal_async(self, *args, **kwargs):
        with self._request_lock:
            return super()._cancel_goal_async(*args, **kwargs)

    def _get_result_async(self, *args, **kwargs):
        with self._request_lock:
            return super()._get_result_async(*args, **kwargs)

    def _remove_pending_goal_request(self, future):
        with self._request_lock:
            return super()._remove_pending_goal_request(future)

    def _remove_pending_cancel_request(self, future):
        with self._request_lock:
            return super()._remove_pending_cancel_request(future)

    def _remove_pending_result_request(self, future):
        with self._request_lock:
            return super()._remove_pending_result_request(future)

    def retire_future(self, future):
        """Forget a lost reply only after the caller proved server termination."""
        with self._request_lock:
            if future.done():
                return
            self._remove_pending_goal_request(future)
            self._remove_pending_cancel_request(future)
            self._remove_pending_result_request(future)
            future.cancel()

    async def execute(self, taken_data):
        responses = {key: value for key, value in taken_data.items() if key != 'feedback'}
        with self._request_lock:
            await super().execute(responses)
        if 'feedback' in taken_data:
            await super().execute({'feedback': taken_data['feedback']})
