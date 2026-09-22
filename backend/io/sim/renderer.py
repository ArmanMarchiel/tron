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
        self._resized = False
        # orbit camera for the environment view (MjvCamera, free): lookat / azimuth / elevation / distance
        self.orbit = mujoco.MjvCamera()
        self.orbit.type = mujoco.mjtCamera.mjCAMERA_FREE
        self.orbit.lookat[:] = [0.45, 0.1, 0.3]
        self.orbit.azimuth, self.orbit.elevation, self.orbit.distance = 51.0, -24.0, 2.7
        self._orbit_default = (51.0, -24.0, 2.7, [0.45, 0.1, 0.3])
        # overlay visibility: each group is a set of geoms that can be hidden without touching physics
        # (they are all contype/conaffinity 0 annotations), by zeroing their alpha for the render.
        self._overlay_geoms = self._group_overlays()
        self._overlay_alpha = {g: float(self.m.geom_rgba[g][3]) for ids in self._overlay_geoms.values() for g in ids}
        self.overlays = {k: True for k in self._overlay_geoms}
        self.shadows = fps <= 15.0        # see set_fps: the shadow pass is about half the frame time
        self._thread = threading.Thread(target=self._loop, name="tron-render", daemon=True)

    def _group_overlays(self) -> dict[str, list[int]]:
        """One switchable group per annotation, so each zone is its own toggle.

        Floor markings are painted on the slab, not an annotation, so they are never switchable.
        """
        groups: dict[str, list[int]] = {}
        for g in range(self.m.ngeom):
            name = self.m.geom(g).name or ""
            if name.startswith("zone_"):
                groups.setdefault(name[len("zone_"):], []).append(g)
            elif name.startswith("scanner_"):
                groups.setdefault(name[len("scanner_"):], []).append(g)
        return groups

    def set_overlay(self, name: str, visible: bool) -> dict:
        """Show or hide one overlay group. Visual only -- these geoms never collide."""
        if name not in self._overlay_geoms:
            raise KeyError(name)
        for g in self._overlay_geoms[name]:
            self.m.geom_rgba[g][3] = self._overlay_alpha[g] if visible else 0.0
        self.overlays[name] = visible
        return dict(self.overlays)

    def overlay_state(self) -> dict:
        """Visibility plus each group's own colour, so the viewer's swatches match the render."""
        swatches = {}
        for k, ids in self._overlay_geoms.items():
            # these geoms take their colour from a material, so geom_rgba is the placeholder default
            mat = self.m.geom_matid[ids[0]]
            r, g, b, _a = (float(v) for v in (self.m.mat_rgba[mat] if mat >= 0 else self.m.geom_rgba[ids[0]]))
            swatches[k] = f"#{int(r * 255):02x}{int(g * 255):02x}{int(b * 255):02x}"
        return {"overlays": dict(self.overlays), "colors": swatches,
                "counts": {k: len(v) for k, v in self._overlay_geoms.items()}}

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

    def set_fps(self, fps: float) -> dict:
        """Change the render rate while streaming, trading shadows for speed above 15 fps.

        The scene is geometry-bound rather than pixel-bound: a CAD machine shell runs to seven figures
        of triangles, so a frame costs about the same at any window size.  Measured on this scene, the
        shadow pass is roughly half the frame time, because it rasterises all that geometry a second
        time from the light's point of view.  Asking for a higher rate without giving anything back
        therefore changes nothing -- the renderer is already flat out.  So the rate doubles as a
        quality dial: 15 keeps shadows, above that drops them, which is what actually buys the frames.
        """
        self.fps = max(1.0, min(float(fps), 60.0))
        self.shadows = self.fps <= 15.0
        return {"fps": self.fps, "shadows": self.shadows}

    def resize(self, width: int, height: int) -> dict:
        """Ask for a different render size, within the model's offscreen framebuffer.

        The viewport is sized by the browser window, so rendering at a fixed shape leaves bars on one
        axis. The client reports its panel size and the render loop picks the change up on its next
        pass."""
        max_w = int(self.m.vis.global_.offwidth)
        max_h = int(self.m.vis.global_.offheight)
        w = max(320, min(int(width), max_w))
        h = max(240, min(int(height), max_h))
        if (w, h) != (self.width, self.height):
            self.width, self.height = w, h
            self._resized = True
        return {"width": self.width, "height": self.height, "max": [max_w, max_h]}

    # ------------------------------------------------------------------ thread
    def _loop(self) -> None:
        rd = mujoco.MjData(self.m)
        renderer = mujoco.Renderer(self.m, height=self.height, width=self.width)
        next_t = time.perf_counter()
        while not self._stop:
            period = 1.0 / self.fps          # read each pass: set_fps may change it mid-stream
            if self._resized:                      # the viewport changed shape
                self._resized = False
                renderer.close()
                renderer = mujoco.Renderer(self.m, height=self.height, width=self.width)
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
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = 1 if self.shadows else 0
        img = renderer.render()
        buf = io.BytesIO()
        Image.fromarray(img).save(buf, format="JPEG", quality=self.quality)
        return buf.getvalue()
