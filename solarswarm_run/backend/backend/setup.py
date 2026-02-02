#!/usr/bin/env python3
"""
Ein einfaches Setup-Skript für die Datenbank.
Führt alles aus: Tabellen, Daten und Trigger.
"""
from sqlalchemy import text
from database import engine, SessionLocal
import models

print("Setup Datenbank...")

# 1. Tabellen erstellen
#print("1. Erstelle Tabellen...")
#models.Base.metadata.drop_all(bind=engine)  # Altes löschen
#models.Base.metadata.create_all(bind=engine)
#print("Fertig")

# 2. Daten einfügen
print("2.Füge Standarddaten ein...")
db = SessionLocal()

try:
    # State-Daten
    db.execute(text('INSERT INTO "State" (state_id, state) VALUES (1, \'Idle\')'))
    db.execute(text('INSERT INTO "State" (state_id, state) VALUES (2, \'Charging\')'))
    db.execute(text('INSERT INTO "State" (state_id, state) VALUES (3, \'MoveToPosition\')'))
    db.execute(text('INSERT INTO "State" (state_id, state) VALUES (4, \'Working\')'))
    
    # Trigger erstellen
    print("3. Erstelle Trigger...")
    db.execute(text("""
        CREATE OR REPLACE FUNCTION log_state_change()
        RETURNS TRIGGER AS $$
        BEGIN
            IF OLD.state_id IS DISTINCT FROM NEW.state_id THEN
                INSERT INTO "StateChange" (state_id, robot_id, begin)
                VALUES (NEW.state_id, NEW.robot_id, EXTRACT(EPOCH FROM NOW()));
            END IF;
            RETURN NEW;
        END;
        $$ LANGUAGE plpgsql;
    """))
    
    db.execute(text("""
        DROP TRIGGER IF EXISTS robot_state_change_trigger ON "Robot";
        CREATE TRIGGER robot_state_change_trigger
        AFTER UPDATE ON "Robot"
        FOR EACH ROW
        EXECUTE FUNCTION log_state_change();
    """))
    
    db.commit()
    print("Fertig")
    print("\nSetup komplett! Datenbank ist bereit.")
    
except Exception as e:
    print(f"Fehler: {e}")
    db.rollback()
finally:
    db.close()