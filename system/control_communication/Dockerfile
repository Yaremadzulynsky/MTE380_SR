FROM python:3.11-slim

WORKDIR /app

ENV PYTHONUNBUFFERED=1

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY app.py ./
COPY robot_control_system/ ./robot_control_system/

ENV PORT=5000
EXPOSE 5000

CMD ["python", "app.py"]
