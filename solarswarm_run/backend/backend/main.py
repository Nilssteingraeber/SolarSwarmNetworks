from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from typing import List, Optional, Any, Dict
import models
from database import engine, SessionLocal
from sqlalchemy.orm import Session
import json
import subprocess
import os
import shlex
import time

'''
Cheatcode alles resetten

# Alle Container stoppen
docker-compose down

# Alle nicht verwendeten Images, Container und Netzwerke löschen
docker system prune -a -f

# Volumes löschen (Achtung: Daten gehen verloren!)
docker volume prune -f

# Cache leeren
docker builder prune -a -f

'''

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5170"],
    allow_methods=["*"],
    allow_headers=["*"],
)
models.Base.metadata.create_all(bind=engine)
ROS_DISTRO = os.getenv("ROS_DISTRO", "jazzy")

# Pydantic Modelle
class StatusBase(BaseModel):
    robot_id: int
    battery: Optional[float] = None
    cpu_1: Optional[float] = None
    point: Optional[Dict[str, Any]] = None
    orientation: Optional[Dict[str, Any]] = None
    last_heard: int

class ServiceCall(BaseModel):
    service_name: str
    service_type: str
    arguments: Optional[Any] = Field(default=None, description="dict or raw string")
    timeout: Optional[int] = 10

class StatusResponse(StatusBase):
    status_id: int
    
    class Config:
        from_attributes = True

class RobotBase(BaseModel):
    nid: str
    state_id: Optional[int] = None
    display_name: Optional[str] = None
    ipv4: Optional[str] = None
    ipv6: Optional[str] = None
    mac: Optional[str] = None

class RobotResponse(RobotBase):
    robot_id: int
    
    class Config:
        from_attributes = True

class NeighborBase(BaseModel):
    robot_id: int
    neighbor: int
    strength: Optional[float] = None

class NeighborResponse(NeighborBase):
    class Config:
        from_attributes = True

class StateBase(BaseModel):
    state: str

class StateResponse(StateBase):
    state_id: int
    
    class Config:
        from_attributes = True

class StateChangeBase(BaseModel):
    state_id: int
    robot_id: int
    begin: int

class StateChangeResponse(StateChangeBase):
    change_id: int
    
    class Config:
        from_attributes = True

# Datenbankverbindung
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

db_dependency = Depends(get_db)

# ROS2 Hilfsfunktionen
def ros_yaml_repr(obj) -> str:
    """Einfache Konvertierung von Python-Objekten in eine ROS/YAML Inline-Notation."""
    if obj is None:
        return "{}"
    if isinstance(obj, bool):
        return "true" if obj else "false"
    if isinstance(obj, (int, float)):
        return str(obj)
    if isinstance(obj, str):
        return f"'{obj}'"
    if isinstance(obj, list):
        inner = ", ".join(ros_yaml_repr(x) for x in obj)
        return f"[{inner}]"
    if isinstance(obj, dict):
        inner = ", ".join(f"{k}: {ros_yaml_repr(v)}" for k, v in obj.items())
        return "{" + inner + "}"
    return f"'{str(obj)}'"

def run_ros2_command(nid: str, ros_cmd: str, timeout: int = 10):
    """Führe ros2-Befehle in einer gesourcten bash-Shell aus und setze ROS_DOMAIN_ID auf nid."""
    full = f"source /opt/ros/{ROS_DISTRO}/setup.bash && ROS_DOMAIN_ID={nid} {ros_cmd}"
    try:
        completed = subprocess.run(["bash", "-lc", full], capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired as e:
        raise HTTPException(status_code=504, detail=f"Timeout: {str(e)}")
    return {"returncode": completed.returncode, "stdout": completed.stdout, "stderr": completed.stderr}

# Status Endpunkte
@app.get("/status/", response_model=List[StatusResponse])
def read_all_status(skip: int = 0, limit: int = 100, db: Session = db_dependency):
    return db.query(models.Status).offset(skip).limit(limit).all()

@app.get("/status/{status_id}", response_model=StatusResponse)
def read_status(status_id: int, db: Session = db_dependency):
    status = db.query(models.Status).filter(models.Status.status_id == status_id).first()
    if not status:
        raise HTTPException(status_code=404, detail="Status not found")
    return status

@app.get("/status/robot/{robot_id}", response_model=List[StatusResponse])
def read_status_by_robot(robot_id: int, db: Session = db_dependency):
    return db.query(models.Status).filter(models.Status.robot_id == robot_id).all()

# Robot Endpunkte
@app.get("/robot/", response_model=List[RobotResponse])
def read_all_robots(skip: int = 0, limit: int = 100, db: Session = db_dependency):
    return db.query(models.Robot).offset(skip).limit(limit).all()

@app.get("/robot/{robot_id}", response_model=RobotResponse)
def read_robot(robot_id: int, db: Session = db_dependency):
    robot = db.query(models.Robot).filter(models.Robot.robot_id == robot_id).first()
    if not robot:
        raise HTTPException(status_code=404, detail="Robot not found")
    return robot

@app.get("/robot/nid/{nid}", response_model=RobotResponse)
def read_robot_by_nid(nid: str, db: Session = db_dependency):
    robot = db.query(models.Robot).filter(models.Robot.nid == nid).first()
    if not robot:
        raise HTTPException(status_code=404, detail="Robot not found")
    return robot

# Nachbar Endpunkte
@app.get("/neighbor/", response_model=List[NeighborResponse])
def read_all_neighbors(skip: int = 0, limit: int = 100, db: Session = db_dependency):
    return db.query(models.Neighbor).offset(skip).limit(limit).all()

@app.get("/neighbor/robot/{robot_id}", response_model=List[NeighborResponse])
def read_neighbors_by_robot(robot_id: int, db: Session = db_dependency):
    return db.query(models.Neighbor).filter(models.Neighbor.robot_id == robot_id).all()

# State Endpunkte
@app.get("/state/", response_model=List[StateResponse])
def read_all_states(db: Session = db_dependency):
    return db.query(models.State).all()

# StateChange Endpunkte
@app.get("/statechange/", response_model=List[StateChangeResponse])
def read_all_state_changes(skip: int = 0, limit: int = 100, db: Session = db_dependency):
    return db.query(models.StateChange).offset(skip).limit(limit).all()


# NEU: Roboter nach Services befragen
@app.get("/robot/{robot_id}/services")
def get_robot_services(robot_id: int, db: Session = db_dependency):
    """Listet alle verfügbaren ROS2 Services eines Roboters auf"""
    robot = db.query(models.Robot).filter(models.Robot.robot_id == robot_id).first()
    if not robot:
        raise HTTPException(status_code=404, detail="Robot not found")
    
    out = run_ros2_command(robot.nid, "ros2 service list -t", timeout=8)
    if out["returncode"] != 0:
        raise HTTPException(status_code=500, detail={"stderr": out["stderr"], "code": out["returncode"]})

    lines = [l.strip() for l in out["stdout"].splitlines() if l.strip()]
    services = []
    for line in lines:
        if "[" in line and "]" in line:
            name, rest = line.split("[", 1)
            service_type = rest.strip().rstrip("]").strip()
            services.append({"name": name.strip(), "type": service_type})
        else:
            services.append({"name": line, "type": None})

    return {"robot_id": robot_id, "nid": robot.nid, "services": services}

#Service auf Roboter aufrufen
@app.post("/robot/{robot_id}/call_service")
def call_robot_service(robot_id: int, call: ServiceCall, db: Session = db_dependency):
    """Ruft einen ROS2 Service auf einem bestimmten Roboter auf"""
    robot = db.query(models.Robot).filter(models.Robot.robot_id == robot_id).first()
    if not robot:
        raise HTTPException(status_code=404, detail="Robot not found")
    
    if call.arguments is None:
        arg_str = "{}"
    elif isinstance(call.arguments, str):
        arg_str = call.arguments
    else:
        arg_str = ros_yaml_repr(call.arguments)

    svc_name_q = shlex.quote(call.service_name)
    svc_type_q = shlex.quote(call.service_type)
    arg_q = shlex.quote(arg_str)

    ros_cmd = f"ros2 service call {svc_name_q} {svc_type_q} {arg_q}"
    out = run_ros2_command(robot.nid, ros_cmd, timeout=call.timeout or 10)
    success = out["returncode"] == 0
    
    return {
        "robot_id": robot_id,
        "nid": robot.nid,
        "service": call.service_name,
        "success": success,
        "result": out
    }

# ROS Endpunkte (für direkte NID-basierte Aufrufe)
@app.get("/ros/{nid}/services")
def list_ros_services(nid: str, timeout: int = 8):
    """Fragt 'ros2 service list -t' für den gegebenen NID ab."""
    out = run_ros2_command(nid, "ros2 service list -t", timeout=timeout)
    if out["returncode"] != 0:
        raise HTTPException(status_code=500, detail={"stderr": out["stderr"], "code": out["returncode"]})

    lines = [l.strip() for l in out["stdout"].splitlines() if l.strip()]
    services = []
    for line in lines:
        if "[" in line and "]" in line:
            name, rest = line.split("[", 1)
            service_type = rest.strip().rstrip("]").strip()
            services.append({"name": name.strip(), "type": service_type})
        else:
            services.append({"name": line, "type": None})

    return {"nid": nid, "services": services, "raw": out}

@app.post("/ros/{nid}/services/call")
def call_ros_service(nid: str, call: ServiceCall):
    """Führt 'ros2 service call <service> <type> <args>' aus."""
    if call.arguments is None:
        arg_str = "{}"
    elif isinstance(call.arguments, str):
        arg_str = call.arguments
    else:
        arg_str = ros_yaml_repr(call.arguments)

    svc_name_q = shlex.quote(call.service_name)
    svc_type_q = shlex.quote(call.service_type)
    arg_q = shlex.quote(arg_str)

    ros_cmd = f"ros2 service call {svc_name_q} {svc_type_q} {arg_q}"
    out = run_ros2_command(nid, ros_cmd, timeout=call.timeout or 10)
    success = out["returncode"] == 0
    return {"nid": nid, "service": call.service_name, "success": success, "result": out}