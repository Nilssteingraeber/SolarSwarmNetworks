import os
import sys
import time
import re
import subprocess
import shlex
import logging
import yaml
from contextlib import asynccontextmanager
from typing import List, Optional, Any, Dict

import models
from database import engine, SessionLocal
from fastapi import FastAPI, HTTPException, Depends, Body
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel, Field
from sqlalchemy.orm import Session
from sqlalchemy import desc, or_, String, text
from sqlalchemy.exc import OperationalError

# ==========================================
# 1. CONFIGURATION & LOGGING
# ==========================================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[logging.StreamHandler(sys.stdout)]
)
logger = logging.getLogger(__name__)

ROS_DISTRO = os.getenv("ROS_DISTRO", "jazzy")

# ==========================================
# 2. PYDANTIC MODELS (SCHEMAS)
# ==========================================

class RobotIdsRequest(BaseModel):
    robot_ids: List[int]

class StatusBase(BaseModel):
    robot_id: int
    battery: Optional[float] = None
    cpu_1: Optional[float] = None
    point: Optional[Dict[str, Any]] = None
    orientation: Optional[Dict[str, Any]] = None
    last_heard: int

class StatusResponse(StatusBase):
    status_id: int
    class Config: from_attributes = True

class RobotBase(BaseModel):
    nid: str
    state_id: Optional[int] = None
    display_name: Optional[str] = None
    ipv4: Optional[str] = None
    mac: Optional[str] = None

class RobotResponse(RobotBase):
    robot_id: int
    class Config: from_attributes = True

class NeighborResponse(BaseModel):
    robot_id: int
    neighbor: int
    strength: Optional[float] = None
    class Config: from_attributes = True

class StateResponse(BaseModel):
    state_id: int
    state: str
    class Config: from_attributes = True

class ServiceCall(BaseModel):
    service_name: str
    service_type: str
    arguments: Optional[Any] = Field(default=None)
    timeout: Optional[int] = 10

# ==========================================
# 3. LIFECYCLE & DATABASE SETUP
# ==========================================

@asynccontextmanager
async def lifespan(app: FastAPI):
    retries = 10
    while retries > 0:
        try:
            models.Base.metadata.create_all(bind=engine)
            logger.info("Database connection established.")
            break
        except OperationalError:
            retries -= 1
            logger.warning(f"DB not ready. Retrying... ({retries} left)")
            time.sleep(2)
    
    db = SessionLocal()
    try:
        if db.query(models.State).count() == 0:
            setup_script = os.path.join(os.path.dirname(__file__), "setup.py")
            if os.path.exists(setup_script):
                subprocess.run([sys.executable, setup_script], check=True)
                logger.info("setup.py completed successfully.")
    finally:
        db.close()
    yield

app = FastAPI(lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5170"],
    allow_methods=["*"],
    allow_headers=["*"],
)

def get_db():
    db = SessionLocal()
    try: yield db
    finally: db.close()

db_dependency = Depends(get_db)

# ==========================================
# 4. ROS2 UTILITIES
# ==========================================

def ros_yaml_repr(obj) -> str:
    if obj is None: return "{}"
    if isinstance(obj, bool): return "true" if obj else "false"
    if isinstance(obj, (int, float)): return str(obj)
    if isinstance(obj, str): return f"'{obj}'"
    if isinstance(obj, list):
        return f"[{', '.join(ros_yaml_repr(x) for x in obj)}]"
    if isinstance(obj, dict):
        return "{" + ", ".join(f"{k}: {ros_yaml_repr(v)}" for k, v in obj.items()) + "}"
    return f"'{str(obj)}'"

def run_ros2_command(nid: str, ros_cmd: str, timeout: int = 10):
    full = (f"source /opt/ros/{ROS_DISTRO}/setup.bash && "
            f"source /app/solarswarm/install/local_setup.bash && "
            f"ROS_DOMAIN_ID=0 {ros_cmd}")
    try:
        return subprocess.run(["bash", "-lc", full], capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=504, detail="ROS2 Command Timeout")

def parse_ros_interface(interface_text: str):
    try:
        parts = interface_text.split('---')
        req = parts[0]
        res = parts[1] if len(parts) > 1 else ""

        def parse_block(block):
            out = []
            for line in block.splitlines():
                line = line.strip()
                if not line or line.startswith('#'): continue
                match = re.match(r'^(\S+)\s+(\S+)', line)
                if match:
                    t, name = match.groups()
                    out.append({"name": name, "type": t})
            return out

        return {
            "request": {"inputs": parse_block(req)},
            "response": {"outputs": parse_block(res)},
        }
    except Exception as e:
        logger.error(f"Interface parsing failed: {e}")
        return {"request": {"inputs": []}, "response": {"outputs": []}}

def parse_ros_output(stdout: str) -> Dict[str, Any]:
    """Parses 'Response(a=1, b='text')' into {'a': 1, 'b': 'text'}"""
    try:
        # Find the line after 'response:'
        lines = stdout.splitlines()
        payload = ""
        for i, line in enumerate(lines):
            if line.strip().lower().startswith("response:"):
                payload = " ".join(lines[i + 1:]).strip()
                break
        
        # Extract content inside parentheses
        match = re.search(r'\((.*)\)', payload, re.DOTALL)
        if not match: return {}

        # Regex for key=value pairs
        pattern = r"(\w+)=('[^']*'|\[.*?\]|[^,\s)]+)"
        pairs = re.findall(pattern, match.group(1))

        result = {}
        for key, val in pairs:
            val = val.strip()
            if val.startswith("'"): val = val[1:-1] # Strip quotes
            elif val.lower() == 'true': val = True
            elif val.lower() == 'false': val = False
            else:
                try: val = float(val) if '.' in val else int(val)
                except: pass
            result[key] = val
        return result
    except:
        return {}


# ==========================================
# 5. ENDPOINTS (NID-CENTRIC)
# ==========================================

@app.get("/robot", response_model=List[RobotResponse])
def read_robots(skip: int = 0, limit: int = 100, db: Session = db_dependency):
    return db.query(models.Robot).offset(skip).limit(limit).all()

@app.get("/robot/{nid}/services")
def get_services_with_interface(nid: str, db: Session = db_dependency):
    robot = db.query(models.Robot).filter_by(nid=nid).first()
    if not robot: raise HTTPException(status_code=404, detail="Robot not found")
    
    out = run_ros2_command(robot.nid, "ros2 service list -t", timeout=8)
    if out.returncode != 0: raise HTTPException(status_code=500, detail=out.stderr)

    enriched_services = []
    internal_keywords = ['parameter', 'type_description', 'lifecycle', 'list_parameters']
    unique_suffix = nid.split('_')[-1] if '_' in nid else nid

    for line in out.stdout.splitlines():
        line = line.strip()
        if "[" in line:
            name, stype = line.split("[", 1)
            name = name.strip()
            stype = stype.replace("]", "").strip()
            
            # Filter noise and other robots
            if any(key in name.lower() for key in internal_keywords) or unique_suffix not in name.lower():
                continue

            iface_out = run_ros2_command(robot.nid, f"ros2 interface show {stype}")
            if iface_out.returncode == 0:
                enriched_services.append({
                    "serviceName": name,
                    "serviceType": stype,
                    "parsed": parse_ros_interface(iface_out.stdout)
                })
    return enriched_services

@app.post("/robot/{nid}/call_service")
def call_service(nid: str, call: ServiceCall, db: Session = db_dependency):
    robot = db.query(models.Robot).filter_by(nid=nid).first()
    if not robot: raise HTTPException(status_code=404, detail="Robot not found")

    arg_str = call.arguments if isinstance(call.arguments, str) else ros_yaml_repr(call.arguments or {})
    cmd = f"ros2 service call {shlex.quote(call.service_name)} {shlex.quote(call.service_type)} {shlex.quote(arg_str)}"
    
    out = run_ros2_command(robot.nid, cmd, timeout=call.timeout)
    
    return {
        "success": out.returncode == 0,
        "result": {
            "stdout": out.stdout,
            "stderr": out.stderr,
            "parsed_data": parse_ros_output(out.stdout) if out.returncode == 0 else None
        }
    }

@app.post("/status/robots", response_model=List[StatusResponse])
def get_batch_statuses(req: RobotIdsRequest, db: Session = db_dependency):
    latest_map = {}
    statuses = db.query(models.Status).filter(models.Status.robot_id.in_(req.robot_ids)).all()
    for s in statuses:
        if s.robot_id not in latest_map or s.last_heard > latest_map[s.robot_id].last_heard:
            latest_map[s.robot_id] = s
    return list(latest_map.values())

@app.post("/neighbor/robots")
def get_batch_neighbors(req: RobotIdsRequest, db: Session = db_dependency):
    neighbors = db.query(models.Neighbor).filter(models.Neighbor.robot_id.in_(req.robot_ids)).all()
    result = {}
    for n in neighbors:
        result.setdefault(n.robot_id, []).append({"neighbor": n.neighbor, "strength": n.strength})
    return result


@app.get("/states", response_model=Dict[int, str])
def get_states_map(db: Session = db_dependency):
    states = db.query(models.State).all()
    return {s.state_id: s.state for s in states}