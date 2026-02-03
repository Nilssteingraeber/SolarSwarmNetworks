# models.py
from sqlalchemy import Column, Integer, String, Float, ForeignKey, BigInteger, JSON
from database import declarative_base
from sqlalchemy.orm import relationship

Base = declarative_base()

class State(Base):
    __tablename__ = "State"
    
    state_id = Column(Integer, primary_key=True, index=True)
    state = Column(String(64))
    
    robots = relationship("Robot", back_populates="state")
    state_changes = relationship("StateChange", back_populates="state")

class Robot(Base):
    __tablename__ = "Robot"
    
    robot_id = Column(Integer, primary_key=True, index=True)
    nid = Column(String(64), unique=True, index=True)
    state_id = Column(Integer, ForeignKey("State.state_id"))
    display_name = Column(String(64))
    ipv4 = Column(String(16))
    ipv6 = Column(String(40))
    mac = Column(String(24))
    
    statuses = relationship("Status", back_populates="robot")
    state_changes = relationship("StateChange", back_populates="robot")
    state = relationship("State", back_populates="robots")

class Status(Base):
    __tablename__ = "Status"
    
    status_id = Column(Integer, primary_key=True, index=True)
    robot_id = Column(Integer, ForeignKey("Robot.robot_id"))
    battery = Column(Float)
    cpu_1 = Column(Float)
    point = Column(JSON)
    orientation = Column(JSON)
    last_heard = Column(BigInteger)
    
    robot = relationship("Robot", back_populates="statuses")

class Neighbor(Base):
    __tablename__ = "Neighbor"
    
    robot_id = Column(Integer, ForeignKey("Robot.robot_id"), primary_key=True)
    neighbor = Column(Integer, ForeignKey("Robot.robot_id"), primary_key=True)
    strength = Column(Float)
    
    robot = relationship("Robot", foreign_keys=[robot_id])
    neighbor_robot = relationship("Robot", foreign_keys=[neighbor])

class StateChange(Base):
    __tablename__ = "StateChange"
    
    change_id = Column(Integer, primary_key=True, index=True)
    state_id = Column(Integer, ForeignKey("State.state_id"))
    robot_id = Column(Integer, ForeignKey("Robot.robot_id"))
    begin = Column(BigInteger)
    
    state = relationship("State", back_populates="state_changes")
    robot = relationship("Robot", back_populates="state_changes")