from sqlalchemy import create_engine # Verbindung von Datenbanken
from sqlalchemy.orm import sessionmaker # Session für Interaktion mit Verbindung für DB
from sqlalchemy.ext.declarative import declarative_base # Klasse für Modelldefinition, Grundlage Tabellen
import os

# Pfad der Datenbank quasi in einer URL eingegeben. Damit die Verbindung leben kann. 
#URL_DATABASE = 'postgresql://postgres:securepassword@localhost:5432/rosData'
URL_DATABASE = os.getenv("DATABASE_URL", "postgresql://postgres:securepassword@db:5432/rosData")

# Verbindungsaufbau mit der Datenbank
engine = create_engine(URL_DATABASE)

# Session Definieren. Änderungen in DB nicht automatisch gespeichert. Selber mit db.commit machen
# Soll nicht die Änderungen an die DB senden. 
# Die Session soll auf dieser DB erstellt werden bind = erstellte Engine
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

Base = declarative_base() # Grundmodell für Datenbanken. Base macht Python-Classen in Tabellen