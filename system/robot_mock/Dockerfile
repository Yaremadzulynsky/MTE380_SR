FROM python:3.11-slim

WORKDIR /app

ENV PYTHONUNBUFFERED=1

COPY requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt

COPY app.py ./
COPY smoke_test.py ./
COPY public ./public

ENV PORT=8200
EXPOSE 8200

CMD ["python", "app.py"]
