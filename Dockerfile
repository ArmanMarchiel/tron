FROM python:3.12-slim
# MuJoCo offscreen rendering needs an EGL/OSMesa-capable libGL; OSMesa works without a GPU.
RUN apt-get update && apt-get install -y --no-install-recommends libgl1-mesa-dri libosmesa6 libglew2.2 libglfw3 && rm -rf /var/lib/apt/lists/*
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa TRON_HOST=0.0.0.0
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt
COPY backend backend
COPY frontend frontend
EXPOSE 8000
HEALTHCHECK --interval=15s --timeout=5s --start-period=40s CMD python -c "import urllib.request,sys; sys.exit(0 if urllib.request.urlopen('http://127.0.0.1:8000/healthz').status==200 else 1)"
CMD ["uvicorn", "backend.app.main:app", "--host", "0.0.0.0", "--port", "8000"]
