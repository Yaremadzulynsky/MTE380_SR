FROM python:3.11-slim

WORKDIR /app

ENV PYTHONUNBUFFERED=1

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY app.py ./
COPY public ./public

ENV PORT=8100
EXPOSE 8100

CMD ["python", "app.py"]
