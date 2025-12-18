FROM python:3.11-slim

WORKDIR /app

# Systemabhängigkeiten falls nötig (z.B. für Postgres client libs)
RUN apt-get update && apt-get install -y build-essential gcc libpq-dev \
    && rm -rf /var/lib/apt/lists/*

# Abhängigkeiten kopieren und installieren
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# App-Code kopieren (einschließlich 3DServer.py falls dort liegt)
COPY . .

# Port für uvicorn
EXPOSE 8000

# Startbefehl (ändere falls notwendig)
CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
