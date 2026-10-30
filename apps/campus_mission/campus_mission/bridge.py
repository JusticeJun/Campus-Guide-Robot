"""Bounded RPC to a separate ROS process; HTTP/AI never spin the ROS executor."""
import asyncio
import concurrent.futures
import multiprocessing as mp
import threading
import uuid


class RosBridge:
    mode = 'live'

    def __init__(self, config):
        from .ros_worker import run_worker
        context = mp.get_context('spawn')
        self.pipe, child = context.Pipe()
        self.pending = {}
        self.lock = threading.Lock()
        self.state = {'connected': False, 'ready': False, 'map_match': False}
        self.process = context.Process(target=run_worker, args=(child, config), daemon=True)
        self.process.start()
        child.close()
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        try:
            while True:
                msg = self.pipe.recv()
                if 'state' in msg:
                    self.state = msg['state']
                    continue
                with self.lock:
                    future = self.pending.pop(msg['id'], None)
                if future and not future.done():
                    try:
                        if 'error' in msg:
                            future.set_exception(RuntimeError(msg['error']))
                        else:
                            future.set_result(msg.get('result'))
                    except concurrent.futures.InvalidStateError:
                        # HTTP cancellation/timeout may win after done() above.
                        pass
        except (EOFError, OSError):
            self.state = {'connected': False, 'ready': False, 'map_match': False,
                          'reason': 'ROS 연결 프로세스가 종료됐습니다.'}
            with self.lock:
                for f in self.pending.values():
                    if not f.done():
                        f.set_exception(RuntimeError(self.state['reason']))
                self.pending.clear()

    async def call(self, op, timeout=150, **kwargs):
        key = uuid.uuid4().hex
        f = concurrent.futures.Future()
        with self.lock:
            if not self.process.is_alive():
                raise RuntimeError('ROS 프로세스가 실행 중이 아닙니다.')
            self.pending[key] = f
            self.pipe.send({'id': key, 'op': op, 'args': kwargs})
        try:
            return await asyncio.wait_for(asyncio.wrap_future(f), timeout)
        finally:
            with self.lock:
                self.pending.pop(key, None)

    async def close(self):
        try:
            await self.call('shutdown', timeout=8)
        except Exception:
            pass
        await asyncio.to_thread(self.process.join, 3)
        if self.process.is_alive():
            self.process.terminate()
        self.pipe.close()


class PreviewBridge:
    """Explicit offline preview: no ROS imports, fabricated motion, or fake success."""
    mode = 'preview'
    state = {'connected': False, 'ready': False, 'map_match': False,
             'reason': '화면 미리보기 · 실제 차량 연결 없음', 'path': []}

    async def call(self, op, **kwargs):
        raise RuntimeError('미리보기 모드에서는 실제 이동·촬영을 실행하지 않습니다.')

    async def close(self):
        pass
