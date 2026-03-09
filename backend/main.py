import os
import logging

from fastapi import FastAPI, Query, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from dotenv import load_dotenv

from maprad_client import fetch_broadcast_systems
from calculations import process_and_rank

load_dotenv()
logging.basicConfig(level=logging.INFO)

app = FastAPI(title="Tower Finder API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:5173", "http://localhost:3000"],
    allow_methods=["*"],
    allow_headers=["*"],
)

API_KEY = os.getenv("MAPRAD_API_KEY", "")


@app.get("/api/towers")
async def find_towers(
    lat: float = Query(..., ge=-90, le=90, description="Latitude"),
    lon: float = Query(..., ge=-180, le=180, description="Longitude"),
    altitude: float = Query(0, ge=0, description="Receiver altitude in metres"),
    limit: int = Query(20, ge=1, le=100, description="Max towers to return"),
    source: str = Query("au", description="Data source: us, au, ca"),
):
    """
    Return nearby broadcast towers ranked for passive-radar suitability.
    """
    if not API_KEY:
        raise HTTPException(status_code=500, detail="MAPRAD_API_KEY not configured")

    source = source.lower()
    if source not in ("us", "au", "ca"):
        raise HTTPException(status_code=400, detail="Invalid source. Use: us, au, ca")

    try:
        raw = await fetch_broadcast_systems(API_KEY, lat, lon, source=source)
    except Exception as exc:
        logging.exception("Maprad API call failed")
        raise HTTPException(status_code=502, detail=f"Upstream API error: {exc}")

    towers = process_and_rank(raw, lat, lon, limit=limit)

    return {
        "towers": towers,
        "query": {
            "latitude": lat,
            "longitude": lon,
            "altitude_m": altitude,
            "radius_km": 80,
            "source": source,
        },
        "count": len(towers),
    }


@app.get("/api/health")
async def health():
    return {"status": "ok"}
