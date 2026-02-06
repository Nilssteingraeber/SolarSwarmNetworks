-- Exported from QuickDBD: https://www.quickdatabasediagrams.com/
-- NOTE! If you have used non-SQL datatypes in your design, you will have to change these here.


CREATE TABLE "Status" (
    "status_id" serial   NOT NULL,
    "robot_id" int   NOT NULL,
    "battery" double precision   NULL,
    "cpu_1" double precision   NULL,
    "point" jsonb   NULL,
    "orientation" jsonb   NULL,
    -- UTC-Timestamp in Sekunden als 64-bit Integer
    -- mit GROUP BY, für neustes Tupel pro Roboter
    "last_heard" bigint  DEFAULT 0 NOT NULL,
    CONSTRAINT "pk_Status" PRIMARY KEY (
        "status_id"
     )
);

CREATE TABLE "Robot" (
    "robot_id" serial   NOT NULL,
    "nid" varchar(64)   NOT NULL,
    -- BEFORE UPDATE TRIGGER -> INSERT INTO StateChange
    "state_id" int   NULL,
    "display_name" varchar(64)   NULL,
    -- 15 Zeichen für ipv4
    "ipv4" varchar(16)   NULL,
    -- 39 Zeichen für ipv6
    "ipv6" varchar(40)   NULL,
    -- 17 Zeichen für mac
    "mac" varchar(24)   NULL,
    CONSTRAINT "pk_Robot" PRIMARY KEY (
        "robot_id"
     ),
    CONSTRAINT "uc_Robot_nid" UNIQUE (
        "nid"
    )
);

CREATE TABLE "State" (
    "state_id" serial   NOT NULL,
    "state" varchar(64)   NOT NULL,
    CONSTRAINT "pk_State" PRIMARY KEY (
        "state_id"
     ),
    CONSTRAINT "uc_State_state" UNIQUE (
        "state"
    )
);

CREATE TABLE "StateChange" (
    "change_id" serial   NOT NULL,
    "state_id" int   NOT NULL,
    "robot_id" int   NOT NULL,
    "begin" bigint   NOT NULL,
    CONSTRAINT "pk_StateChange" PRIMARY KEY (
        "change_id"
     )
);

CREATE TABLE "Neighbor" (
    "robot_id" int   NOT NULL,
    "neighbor" int   NOT NULL,
    "strength" real  DEFAULT 1.0 NOT NULL,
    CONSTRAINT "pk_Neighbor" PRIMARY KEY (
        "robot_id","neighbor"
     )
);

ALTER TABLE "Status" ADD CONSTRAINT "fk_Status_robot_id" FOREIGN KEY("robot_id")
REFERENCES "Robot" ("robot_id");

ALTER TABLE "Robot" ADD CONSTRAINT "fk_Robot_state_id" FOREIGN KEY("state_id")
REFERENCES "State" ("state_id");

ALTER TABLE "StateChange" ADD CONSTRAINT "fk_StateChange_state_id" FOREIGN KEY("state_id")
REFERENCES "State" ("state_id");

ALTER TABLE "StateChange" ADD CONSTRAINT "fk_StateChange_robot_id" FOREIGN KEY("robot_id")
REFERENCES "Robot" ("robot_id");

ALTER TABLE "Neighbor" ADD CONSTRAINT "fk_Neighbor_robot_id" FOREIGN KEY("robot_id")
REFERENCES "Robot" ("robot_id");

ALTER TABLE "Neighbor" ADD CONSTRAINT "fk_Neighbor_neighbor" FOREIGN KEY("neighbor")
REFERENCES "Robot" ("robot_id");

