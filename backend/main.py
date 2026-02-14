"""
FastAPI backend for the fire demo.
- Receives alert payloads (e.g., Caltrans camera trigger).
- Receives drone telemetry / observations.
- Stores to a lightweight SQLite DB for the demo.
- Ready to deploy on a Vultr VM: `uvicorn main:app --host 0.0.0.0 --port 8000`.
"""

import os
import sqlite3
from datetime import datetime
from typing import Optional, List

from fastapi import FastAPI
from pydantic import BaseModel, Field
from fastapi.middleware.cors import CORSMiddleware

DB_PATH = os.getenv("DEMO_DB_PATH", "data.db")
BACKEND_VERSION = "1.0.0"


def init_db():
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute(
        """
        CREATE TABLE IF NOT EXISTS alerts (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            created_at TEXT,
            source TEXT,
            location TEXT,
            screenshot_url TEXT,
            note TEXT
        )
        """
    )
    cur.execute(
        """
        CREATE TABLE IF NOT EXISTS observations (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            created_at TEXT,
            lat REAL,
            lon REAL,
            alt REAL,
            summary TEXT,
            media_url TEXT,
            metadata TEXT
        )
        """
    )
    conn.commit()
    conn.close()


init_db()

app = FastAPI(title="FireBot Demo Backend", version=BACKEND_VERSION)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class AlertIn(BaseModel):
    source: str = Field(default="caltrans_cam")
    location: str
    screenshot_url: str
    note: Optional[str] = None


class ObservationIn(BaseModel):
    lat: float
    lon: float
    alt: float
    summary: str
    media_url: Optional[str] = None
    metadata: Optional[str] = None


@app.get("/")
def root():
    return {"status": "ok", "version": BACKEND_VERSION, "db": DB_PATH}


@app.post("/alert")
def post_alert(alert: AlertIn):
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    now = datetime.utcnow().isoformat()
    cur.execute(
        "INSERT INTO alerts (created_at, source, location, screenshot_url, note) VALUES (?, ?, ?, ?, ?)",
        (now, alert.source, alert.location, alert.screenshot_url, alert.note),
    )
    conn.commit()
    conn.close()
    return {"ok": True, "ts": now}


@app.post("/observation")
def post_observation(obs: ObservationIn):
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    now = datetime.utcnow().isoformat()
    cur.execute(
        "INSERT INTO observations (created_at, lat, lon, alt, summary, media_url, metadata) VALUES (?, ?, ?, ?, ?, ?, ?)",
        (now, obs.lat, obs.lon, obs.alt, obs.summary, obs.media_url, obs.metadata),
    )
    conn.commit()
    conn.close()
    return {"ok": True, "ts": now}


@app.get("/alerts")
def list_alerts():
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute("SELECT id, created_at, source, location, screenshot_url, note FROM alerts ORDER BY id DESC LIMIT 20")
    rows = cur.fetchall()
    conn.close()
    return [
        {
            "id": r[0],
            "created_at": r[1],
            "source": r[2],
            "location": r[3],
            "screenshot_url": r[4],
            "note": r[5],
        }
        for r in rows
    ]


@app.get("/observations")
def list_observations():
    conn = sqlite3.connect(DB_PATH)
    cur = conn.cursor()
    cur.execute(
        "SELECT id, created_at, lat, lon, alt, summary, media_url, metadata FROM observations ORDER BY id DESC LIMIT 20"
    )
    rows = cur.fetchall()
    conn.close()
    return [
        {
            "id": r[0],
            "created_at": r[1],
            "lat": r[2],
            "lon": r[3],
            "alt": r[4],
            "summary": r[5],
            "media_url": r[6],
            "metadata": r[7],
        }
        for r in rows
    ]

