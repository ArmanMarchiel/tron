"""Offscreen MuJoCo render service.

One background thread owns the OpenGL context.  It continuously renders the live simulator
state into the latest JPEG (served as an MJPEG stream) and services on-demand requests to
render an arbitrary joint configuration (used by the timeline to show the twin at a past
instant).
"""
from __future__ import annotations

import io
import queue
import threading
import time
from collections.abc import Callable
from concurrent.futures import Future

import mujoco
import numpy as np
from PIL import Image


class RenderService:
    def __init__(self, model: mujoco.MjModel, live_state: Callable[[], tuple[np.ndarray, np.ndarray]],
                 camera: str = "twin_cam", width: int = 960, height: int = 640, fps: float = 15.0, quality: int = 80):
        self.m = model
        self.live_state = live_state
        self.camera, self.width, self.height, self.fps, self.quality = camera, width, height, fps, quality
        self.latest: dict[str, bytes] = {}       # camera name -> latest JPEG
        self.frame_id: dict[str, int] = {}
        self._viewers: dict[str, int] = {}        # camera name -> number of open streams
        self._vlock = threading.Lock()
        self._requests: queue.Queue = queue.Queue()
        self._stop = False
        # orbit camera for the environment view (MjvCamera, free): lookat / azimuth / elevation / distance
        self.orbit = mujoco.MjvCamera()
        self.orbit.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.orbit.lookat[:] = [0.45, 0.1, 0.3]
        self.orbit.azimuth, self.orbit.elevation, self.orbit.distance = 51.0, -24.0, 2.7
        self._orbit_default = (51.0, -24.0, 2.7, [0.45, 0.1, 0.3])
        self._thread = threading.Thread(target=self._loop, name="tron-render", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop = True

    def render_pose(self, qpos: np.ndarray, mocap, camera: str | None = None) -> Future:
        fut: Future = Future()
        self._requests.put((np.asarray(qpos, dtype=float), mocap, camera or self.camera, fut))
        return fut

    def orbit_update(self, d_az: float = 0.0, d_el: float = 0.0, zoom: float = 1.0, reset: bool = False) -> dict:
        if reset:
            az, el, dist, look = self._orbit_default
            self.orbit.azimuth, self.orbit.elevation, self.orbit.distance = az, el, dist
            self.orbit.lookat[:] = look
        else:
            self.orbit.azimuth = (self.orbit.azimuth + d_az) % 360.0
            self.orbit.elevation = max(-89.0, min(-2.0, self.orbit.elevation + d_el))
            self.orbit.distance = max(0.6, min(6.0, self.orbit.distance * zoom))
        return {"azimuth": round(self.orbit.azimuth, 1), "elevation": round(self.orbit.elevation, 1), "distance": round(self.orbit.distance, 2)}

    def acquire(self, camera: str) -> None:
        with self._vlock:
            self._viewers[camera] = self._viewers.get(camera, 0) + 1

    def release(self, camera: str) -> None:
        with self._vlock:
            n = self._viewers.get(camera, 0) - 1
            if n <= 0:
                self._viewers.pop(camera, None)
            else:
                self._viewers[camera] = n

    def cameras_to_render(self) -> list[str]:
        with self._vlock:
            return list(self._viewers) or [self.camera]

    # ------------------------------------------------------------------ thread
    def _loop(self) -> None:
        rd = mujoco.MjData(self.m)
        renderer = mujoco.Renderer(self.m, height=self.height, width=self.width)
        period = 1.0 / self.fps
        next_t = time.perf_counter()
        while not self._stop:
            # on-demand requests first
            try:
                while True:
                    qpos, mocap, cam, fut = self._requests.get_nowait()
                    try:
                        fut.set_result(self._render(renderer, rd, qpos, mocap, cam))
                    except Exception as e:  # pragma: no cover
                        fut.set_exception(e)
            except queue.Empty:
                pass
            qpos, mocap = self.live_state()
            for cam in self.cameras_to_render():
                self.latest[cam] = self._render(renderer, rd, qpos, mocap, cam)
                self.frame_id[cam] = self.frame_id.get(cam, 0) + 1
            next_t += period
            delay = next_t - time.perf_counter()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.perf_counter()
        renderer.close()

    def _render(self, renderer, rd, qpos, mocap, camera: str) -> bytes:
        n = min(len(qpos), self.m.nq)
        rd.qpos[:n] = qpos[:n]
        if mocap is not None and self.m.nmocap > 0:
            pos, quat = mocap if isinstance(mocap, tuple) else (mocap, None)
            if pos is not None:
                rd.mocap_pos[:] = np.asarray(pos).reshape(rd.mocap_pos.shape)
            if quat is not None:
                rd.mocap_quat[:] = np.asarray(quat).reshape(rd.mocap_quat.shape)
        mujoco.mj_forward(self.m, rd)
        renderer.update_scene(rd, camera=(self.orbit if camera == self.camera else camera))
        img = renderer.render()
        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format="JPEG", quality=self.quality)
        return buf.getvalue()
