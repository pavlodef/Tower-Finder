"""
Passive Radar Detection Pipeline
Reads .detection files, tracks targets, geolocates on bistatic ellipse,
and outputs tar1090-compatible aircraft.json.

Pipeline: detection data → tracker → geolocator → tar1090 JSON
"""

import json
import math
import os
import time
import glob
from pathlib import Path
from typing import Optional

# ─── Constants ───────────────────────────────────────────────────────
C = 299_792_458.0  # speed of light m/s
EARTH_RADIUS_M = 6_371_000.0
FT_TO_M = 0.3048

# ─── Node Configuration ─────────────────────────────────────────────
DEFAULT_NODE_CONFIG = {
    "node_id": "net13",
    "Fs": 2_000_000,        # Sample rate Hz
    "FC": 195_000_000,      # Center frequency Hz
    "rx_lat": 33.939182,
    "rx_lon": -84.651910,
    "rx_alt_ft": 950,
    "tx_lat": 33.75667,
    "tx_lon": -84.331844,
    "tx_alt_ft": 1600,
    "doppler_min": -300,
    "doppler_max": 300,
    "min_doppler": 15,
}


def haversine(lat1, lon1, lat2, lon2):
    """Distance in meters between two lat/lon points."""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def bearing(lat1, lon1, lat2, lon2):
    """Initial bearing from point 1 to point 2, in radians."""
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlon = lon2 - lon1
    x = math.sin(dlon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.atan2(x, y)


def dest_point(lat, lon, bearing_rad, distance_m):
    """Destination point given start, bearing, distance."""
    lat = math.radians(lat)
    lon = math.radians(lon)
    d = distance_m / EARTH_RADIUS_M
    new_lat = math.asin(
        math.sin(lat) * math.cos(d) + math.cos(lat) * math.sin(d) * math.cos(bearing_rad)
    )
    new_lon = lon + math.atan2(
        math.sin(bearing_rad) * math.sin(d) * math.cos(lat),
        math.cos(d) - math.sin(lat) * math.sin(new_lat),
    )
    return math.degrees(new_lat), math.degrees(new_lon)


# ─── Bistatic Geometry ──────────────────────────────────────────────

class BistaticGeometry:
    """Computes positions on a bistatic ellipse defined by TX and RX locations."""

    def __init__(self, config: dict):
        self.tx_lat = config["tx_lat"]
        self.tx_lon = config["tx_lon"]
        self.rx_lat = config["rx_lat"]
        self.rx_lon = config["rx_lon"]
        self.Fs = config["Fs"]
        self.FC = config["FC"]
        self.baseline_m = haversine(self.tx_lat, self.tx_lon, self.rx_lat, self.rx_lon)
        self.midpoint_lat = (self.tx_lat + self.rx_lat) / 2
        self.midpoint_lon = (self.tx_lon + self.rx_lon) / 2
        self.baseline_bearing = bearing(self.tx_lat, self.tx_lon, self.rx_lat, self.rx_lon)

    def delay_to_bistatic_range(self, delay_bins: float) -> float:
        """Convert delay bins to bistatic range excess in meters.
        delay_bins * (c / Fs) gives the extra path length beyond the baseline."""
        return delay_bins * (C / self.Fs)

    def point_on_ellipse(self, delay_bins: float, theta: float):
        """Get lat/lon for a point on the bistatic ellipse.

        Args:
            delay_bins: delay value from detection
            theta: angle parameter (radians, 0 = perpendicular bisector of baseline)

        Returns:
            (lat, lon) of the point
        """
        range_excess = self.delay_to_bistatic_range(delay_bins)
        total_path = self.baseline_m + range_excess
        a = total_path / 2  # semi-major axis
        c_half = self.baseline_m / 2  # half-focal distance

        if a <= c_half:
            return self.midpoint_lat, self.midpoint_lon

        b = math.sqrt(a * a - c_half * c_half)  # semi-minor axis

        # Ellipse in local coords (x along baseline, y perpendicular)
        x_local = a * math.cos(theta)
        y_local = b * math.sin(theta)

        # Convert to distance/bearing from midpoint
        dist = math.sqrt(x_local ** 2 + y_local ** 2)
        local_angle = math.atan2(y_local, x_local)
        world_bearing = self.baseline_bearing + local_angle

        return dest_point(self.midpoint_lat, self.midpoint_lon, world_bearing, dist)

    def estimate_theta_from_doppler(self, delay_bins: float, doppler_hz: float) -> float:
        """Estimate the ellipse angle parameter from Doppler shift.

        Uses the Doppler to bias the position. Positive Doppler → approaching
        (closer to one focus), negative → receding. Maps to theta roughly."""
        doppler_max = 300.0
        normalized = max(-1.0, min(1.0, doppler_hz / doppler_max))
        # Map to theta: 0 is perpendicular, ±π/2 toward foci
        return normalized * (math.pi / 3)

    def geolocate(self, delay_bins: float, doppler_hz: float):
        """Geolocate a detection to lat/lon.

        Returns:
            (lat, lon, speed_estimate_knots)
        """
        theta = self.estimate_theta_from_doppler(delay_bins, doppler_hz)
        lat, lon = self.point_on_ellipse(delay_bins, theta)

        # Estimate ground speed from Doppler
        wavelength = C / self.FC
        # Bistatic doppler ≈ fd = v/λ * (cos α + cos β)
        # For approximate speed, assume cos factors ≈ 1.2 average
        speed_ms = abs(doppler_hz) * wavelength / 1.2
        speed_knots = speed_ms * 1.94384

        return lat, lon, speed_knots


# ─── Tracker ─────────────────────────────────────────────────────────

class Track:
    """A tracked target in delay-doppler space."""

    _next_id = 1

    def __init__(self, delay: float, doppler: float, snr: float, timestamp_ms: int):
        self.track_id = Track._next_id
        Track._next_id += 1
        self.hex_id = f"pr{self.track_id:04x}"
        self.delay = delay
        self.doppler = doppler
        self.snr = snr
        self.last_update_ms = timestamp_ms
        self.first_seen_ms = timestamp_ms
        self.hit_count = 1
        self.miss_count = 0
        self.confirmed = False
        self.lat: Optional[float] = None
        self.lon: Optional[float] = None
        self.speed_knots: Optional[float] = None
        self.track_angle: Optional[float] = None

    def distance_to(self, delay: float, doppler: float) -> float:
        """Gate distance in normalized delay-doppler space."""
        dd = (self.delay - delay) / 5.0       # normalize delay (range ~60)
        df = (self.doppler - doppler) / 50.0   # normalize doppler (range ~600)
        return math.sqrt(dd * dd + df * df)

    def update(self, delay: float, doppler: float, snr: float, timestamp_ms: int):
        """Update track with new detection."""
        alpha = 0.3  # smoothing factor
        self.delay = self.delay * (1 - alpha) + delay * alpha
        self.doppler = self.doppler * (1 - alpha) + doppler * alpha
        self.snr = snr
        self.last_update_ms = timestamp_ms
        self.hit_count += 1
        self.miss_count = 0
        if self.hit_count >= 3:
            self.confirmed = True

    def predict(self, timestamp_ms: int):
        """Coast the track (no update this frame)."""
        self.miss_count += 1

    @property
    def is_dead(self) -> bool:
        return self.miss_count > 10

    @property
    def age_seconds(self) -> float:
        return (self.last_update_ms - self.first_seen_ms) / 1000.0


class Tracker:
    """Simple nearest-neighbor tracker in delay-doppler space."""

    GATE_THRESHOLD = 2.0  # normalized distance threshold

    def __init__(self, snr_threshold: float = 6.0, min_doppler: float = 15.0):
        self.tracks: list[Track] = []
        self.snr_threshold = snr_threshold
        self.min_doppler = min_doppler

    def update(self, detections: list[dict], timestamp_ms: int):
        """Process one frame of detections.

        Args:
            detections: list of {"delay": float, "doppler": float, "snr": float}
            timestamp_ms: frame timestamp
        """
        # Filter by SNR and min doppler (remove zero-doppler clutter)
        valid = [
            d for d in detections
            if d["snr"] >= self.snr_threshold and abs(d["doppler"]) >= self.min_doppler
        ]

        used = set()
        for det in valid:
            best_track = None
            best_dist = self.GATE_THRESHOLD

            for i, track in enumerate(self.tracks):
                if i in used:
                    continue
                dist = track.distance_to(det["delay"], det["doppler"])
                if dist < best_dist:
                    best_dist = dist
                    best_track = track

            if best_track is not None:
                best_track.update(det["delay"], det["doppler"], det["snr"], timestamp_ms)
                used.add(self.tracks.index(best_track))
            else:
                # Start new track
                self.tracks.append(
                    Track(det["delay"], det["doppler"], det["snr"], timestamp_ms)
                )

        # Coast unmatched tracks
        for i, track in enumerate(self.tracks):
            if i not in used:
                track.predict(timestamp_ms)

        # Remove dead tracks
        self.tracks = [t for t in self.tracks if not t.is_dead]

    def get_confirmed_tracks(self) -> list[Track]:
        return [t for t in self.tracks if t.confirmed]


# ─── Pipeline: Detection → Track → Geolocate → tar1090 JSON ────────

class PassiveRadarPipeline:
    """Full pipeline from detection files to tar1090 aircraft.json."""

    def __init__(self, node_config: dict = None):
        config = node_config or DEFAULT_NODE_CONFIG
        self.config = config
        self.geometry = BistaticGeometry(config)
        self.tracker = Tracker(
            snr_threshold=6.0,
            min_doppler=config.get("min_doppler", 15.0),
        )
        self.node_id = config.get("node_id", "net13")

    def process_frame(self, frame: dict):
        """Process a single detection frame {timestamp, delay[], doppler[], snr[]}."""
        ts = frame["timestamp"]
        delays = frame.get("delay", [])
        dopplers = frame.get("doppler", [])
        snrs = frame.get("snr", [])

        detections = [
            {"delay": d, "doppler": f, "snr": s}
            for d, f, s in zip(delays, dopplers, snrs)
        ]

        self.tracker.update(detections, ts)

        # Geolocate confirmed tracks
        for track in self.tracker.get_confirmed_tracks():
            lat, lon, speed = self.geometry.geolocate(track.delay, track.doppler)
            track.lat = lat
            track.lon = lon
            track.speed_knots = speed
            # Estimate track angle from Doppler sign
            if track.doppler > 0:
                track.track_angle = (math.degrees(self.geometry.baseline_bearing) + 90) % 360
            else:
                track.track_angle = (math.degrees(self.geometry.baseline_bearing) - 90) % 360

    def process_file(self, filepath: str) -> list[Track]:
        """Process an entire .detection file. Returns confirmed tracks after processing."""
        with open(filepath, "r") as f:
            content = f.read().strip()
            # Detection files are JSON arrays (may lack outer brackets)
            if not content.startswith("["):
                content = "[" + content + "]"
            frames = json.loads(content)

        for frame in frames:
            self.process_frame(frame)

        return self.tracker.get_confirmed_tracks()

    def generate_aircraft_json(self) -> dict:
        """Generate tar1090-compatible aircraft.json from current tracked targets."""
        now = time.time()
        aircraft = []

        for track in self.tracker.get_confirmed_tracks():
            if track.lat is None:
                continue

            ac = {
                "hex": track.hex_id,
                "type": "tisb_other",
                "flight": f"PR{track.track_id:04d} ",  # 8 chars padded
                "alt_baro": 10000,  # estimated altitude (passive radar can't determine)
                "alt_geom": 10000,
                "gs": round(track.speed_knots, 1) if track.speed_knots else 0,
                "track": round(track.track_angle, 1) if track.track_angle else 0,
                "lat": round(track.lat, 6),
                "lon": round(track.lon, 6),
                "seen": 0,
                "seen_pos": 0,
                "messages": track.hit_count,
                "rssi": -round(50 - track.snr, 1),
                "category": "A3",
            }
            aircraft.append(ac)

        return {
            "now": now,
            "messages": sum(t.hit_count for t in self.tracker.tracks),
            "aircraft": aircraft,
        }

    def generate_receiver_json(self) -> dict:
        """Generate tar1090-compatible receiver.json for the RX site."""
        return {
            "version": "retina-passive-radar",
            "refresh": 1000,
            "history": 0,
            "lat": self.config["rx_lat"],
            "lon": self.config["rx_lon"],
        }


def process_detection_folder(folder: str, output_dir: str, node_config: dict = None):
    """Process all .detection files in a folder and write tar1090 JSON to output_dir.

    Args:
        folder: path to folder containing .detection files
        output_dir: path for tar1090 data/ output (aircraft.json, receiver.json)
        node_config: optional node configuration dict
    """
    pipeline = PassiveRadarPipeline(node_config)

    detection_files = sorted(glob.glob(os.path.join(folder, "*.detection")))
    if not detection_files:
        print(f"No .detection files found in {folder}")
        return

    os.makedirs(output_dir, exist_ok=True)

    # Write receiver.json once
    receiver = pipeline.generate_receiver_json()
    with open(os.path.join(output_dir, "receiver.json"), "w") as f:
        json.dump(receiver, f)

    # Process each detection file
    for filepath in detection_files:
        print(f"Processing: {os.path.basename(filepath)}")
        pipeline.process_file(filepath)

    # Write final aircraft.json
    aircraft_data = pipeline.generate_aircraft_json()
    with open(os.path.join(output_dir, "aircraft.json"), "w") as f:
        json.dump(aircraft_data, f)

    print(f"Output: {len(aircraft_data['aircraft'])} tracked targets")
    return aircraft_data


if __name__ == "__main__":
    import sys
    folder = sys.argv[1] if len(sys.argv) > 1 else "."
    output = sys.argv[2] if len(sys.argv) > 2 else "./tar1090_data"
    process_detection_folder(folder, output)
