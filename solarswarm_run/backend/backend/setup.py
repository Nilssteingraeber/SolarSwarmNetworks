#!/usr/bin/env python3
"""
Database Setup Script.
Safe version: Only creates missing tables and inserts missing data.
Does NOT drop existing tables, preventing deadlocks with running services.
"""
import sys
import time
import logging
from sqlalchemy import text
from sqlalchemy.exc import OperationalError, SQLAlchemyError, IntegrityError
from database import engine, SessionLocal
import models

# --- Configuration & Logging ---
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] setup.py: %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger(__name__)

def wait_for_db(retries=15, delay=2):
    """Waits for the database connection to be available."""
    logger.info("Connecting to database...")
    for i in range(retries):
        try:
            with engine.connect() as conn:
                logger.info("Database connection established.")
                return True
        except OperationalError:
            logger.warning(f"Database not ready. Retrying in {delay}s... ({retries - i - 1} attempts left)")
            time.sleep(delay)
    
    logger.error("Could not connect to the database after multiple attempts.")
    sys.exit(1)

def main():
    logger.info("Starting Database Setup...")

    # 1. Wait for DB
    wait_for_db()

    # 2. Create Tables (Safe: checkfirst=True is default)
    # We REMOVED drop_all() to avoid deadlocks with data_sink
    try:
        logger.info("1. Ensuring Tables Exist...")
        models.Base.metadata.create_all(bind=engine)
        logger.info("Tables checked/created.")
    except SQLAlchemyError as e:
        logger.error(f"Failed to check tables: {e}")
        sys.exit(1)

    # 3. Insert Data
    logger.info("2. Inserting Default Data...")
    db = SessionLocal()
    
    try:
        # --- A. Insert States (Idempotent) ---
        states = [
            {"id": 1, "name": "Idle"},
            {"id": 2, "name": "Charging"},
            {"id": 3, "name": "MoveToPosition"},
            {"id": 4, "name": "Working"}
        ]

        for s in states:
            # Check if exists first to avoid Primary Key errors
            exists = db.execute(
                text('SELECT 1 FROM "State" WHERE state_id = :id'), 
                {"id": s["id"]}
            ).scalar()

            if not exists:
                db.execute(
                    text('INSERT INTO "State" (state_id, state) VALUES (:id, :name)'),
                    {"id": s["id"], "name": s["name"]}
                )
                logger.info(f"Inserted state: {s['name']}")
        
        # --- B. Create Triggers ---
        logger.info("3. Configuring Triggers...")
        
        # Function: log_state_change
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

        # Trigger: robot_state_change_trigger
        db.execute(text("""
            DROP TRIGGER IF EXISTS robot_state_change_trigger ON "Robot";
            CREATE TRIGGER robot_state_change_trigger
            AFTER UPDATE ON "Robot"
            FOR EACH ROW
            EXECUTE FUNCTION log_state_change();
        """))

        db.commit()
        logger.info("Setup actions completed successfully.")

    except Exception as e:
        logger.error(f"Error during setup: {e}")
        db.rollback()
        sys.exit(1)
    finally:
        db.close()

if __name__ == "__main__":
    main()