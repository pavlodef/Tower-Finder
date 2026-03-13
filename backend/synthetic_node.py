"""
Synthetic Node Microservice for Retina Passive Radar Network.

Generates and streams synthetic detection data over TCP to the
tracker server for testing. Supports three modes:

1. Detection-only: delay/doppler/snr data (simulated aircraft tracks)
2. With ADS-B: some detections include ADS-B truth data
3. With anomalous objects: objects that have no ADS-B correlation

Usage:
    python synthetic_node.py                           # defaults
    python synthetic_node.py --host localhost --port 3012
    python synthetic_node.py --mode adsb               # include ADS-B
    python synthetic_node.py --mode anomalous          # include anomalous
    python synthetic_node.py --file data.detection     # replay file
    python synthetic_node.py --http http://localhost:8000/api/radar/detections

Node configuration (tower/receiver geometry) is loaded from
node_config.json or CLI arguments.
"""

import argparse
import json
import math
import os
import random
import socket
import sys
import time
from dataclasses import dataclass, field
from typing import Optional

# Speed of light km/μs
C_KM_US = 0.299792458


@dataclass
class NodeConfig:
    """Passive radar node configuration."""
    node_id: str = "synthetic-01"
    rx_lat: float = 33.939182
    rx_lon: float = -84.651910
    rx_alt_ft: float = 950.0
    tx_lat: float = 33.75667
    tx_lon: float = -84.331844
    tx_alt_ft: float = 1600.0
    fc_hz: float = 195_000_000.0
    fs_hz: float = 2_000_000.0
    doppler_min: float = -300.0
    doppler_max: float = 300.0
    min_doppler: float = 15.0


@dataclass
class SyntheticTarget:
    """A simulated moving target in ENU coordinates (km) relative to RX."""
    target_id: str
    # Position (ENU km)
    east: float
    north: float
    up: float
    # Velocity (km/s)
    vel_east: float
    vel_north: float
    vel_up: float
    # Properties
    is_anomalous: bool = False
    adsb_hex: Optional[str] = None
    adsb_callsign: Optional[str] = None
    # Track lifetime
    created_at: float = 0.0
    lifetime_s: float = 300.0


def _lla_to_enu(lat, lon, alt_m, ref_lat, ref_lon, ref_alt_m):
    """Convert LLA to ENU (km) relative to reference point."""
    R = 6371.0
    dlat = math.radians(lat - ref_lat)
    dlon = math.radians(lon - ref_lon)
    north = dlat * R
    east = dlon * R * math.cos(math.radians(ref_lat))
    up = (alt_m - ref_alt_m) / 1000.0
    return east, north, up


def _enu_to_lla(east_km, north_km, up_km, ref_lat, ref_lon, ref_alt_m):
    """Convert ENU (km) back to LLA."""
    R = 6371.0
    lat = ref_lat + math.degrees(north_km / R)
    lon = ref_lon + math.degrees(east_km / (R * math.cos(math.radians(ref_lat))))
    alt_m = ref_alt_m + up_km * 1000.0
    return lat, lon, alt_m


def _norm(v):
    return math.sqrt(sum(x * x for x in v))


def _bistatic_delay(target_enu, tx_enu, rx_enu=(0, 0, 0)):
    """Compute bistatic differential delay in microseconds."""
    d_tx_tgt = _norm([target_enu[i] - tx_enu[i] for i in range(3)])
    d_tgt_rx = _norm([rx_enu[i] - target_enu[i] for i in range(3)])
    d_tx_rx = _norm([rx_enu[i] - tx_enu[i] for i in range(3)])
    diff_range = (d_tx_tgt + d_tgt_rx) - d_tx_rx
    return diff_range / C_KM_US


def _bistatic_doppler(target_enu, vel_enu, tx_enu, rx_enu, freq_hz):
    """Compute bistatic Doppler shift in Hz."""
    c_km_s = 299792.458

    # Unit vectors from target to TX and RX
    to_tx = [tx_enu[i] - target_enu[i] for i in range(3)]
    to_rx = [rx_enu[i] - target_enu[i] for i in range(3)]
    d_tx = _norm(to_tx)
    d_rx = _norm(to_rx)

    if d_tx < 1e-9 or d_rx < 1e-9:
        return 0.0

    u_tx = [to_tx[i] / d_tx for i in range(3)]
    u_rx = [to_rx[i] / d_rx for i in range(3)]

    # Radial velocities (positive = approaching)
    v_rad_tx = sum(vel_enu[i] * u_tx[i] for i in range(3))
    v_rad_rx = sum(vel_enu[i] * u_rx[i] for i in range(3))

    return (freq_hz / c_km_s) * (v_rad_tx + v_rad_rx)


class SyntheticNodeGenerator:
    """Generates realistic synthetic detection data."""

    def __init__(self, config: NodeConfig, mode: str = "detection"):
        self.config = config
        self.mode = mode  # "detection", "adsb", "anomalous"
        self.targets: list[SyntheticTarget] = []
        self._next_target_id = 1

        # Compute TX position in ENU (km) relative to RX
        rx_alt_m = config.rx_alt_ft * 0.3048
        tx_alt_m = config.tx_alt_ft * 0.3048
        self.rx_enu = (0.0, 0.0, 0.0)
        self.tx_enu = _lla_to_enu(
            config.tx_lat, config.tx_lon, tx_alt_m,
            config.rx_lat, config.rx_lon, rx_alt_m,
        )
        self.rx_alt_m = rx_alt_m

    def _spawn_target(self, now: float) -> SyntheticTarget:
        """Create a new synthetic aircraft target."""
        tid = f"syn-{self._next_target_id:04d}"
        self._next_target_id += 1

        # Random position 10-60 km from RX, 3-12 km altitude
        angle = random.uniform(0, 2 * math.pi)
        dist = random.uniform(10, 60)
        east = dist * math.cos(angle)
        north = dist * math.sin(angle)
        up = random.uniform(3, 12)  # 3-12 km altitude

        # Random velocity 100-250 m/s (typical aircraft)
        speed_km_s = random.uniform(0.1, 0.25)
        heading = random.uniform(0, 2 * math.pi)
        vel_east = speed_km_s * math.cos(heading)
        vel_north = speed_km_s * math.sin(heading)
        vel_up = random.uniform(-0.005, 0.005)  # slight climb/descent

        is_anomalous = False
        adsb_hex = None
        adsb_callsign = None

        if self.mode == "anomalous" and random.random() < 0.3:
            # 30% chance of anomalous target (no ADS-B, unusual behavior)
            is_anomalous = True
            # Anomalous targets can be slower/faster than normal aircraft
            speed_km_s = random.uniform(0.01, 0.4)
            vel_east = speed_km_s * math.cos(heading)
            vel_north = speed_km_s * math.sin(heading)
            up = random.uniform(0.3, 15)  # wider altitude range
        elif self.mode in ("adsb", "anomalous"):
            # Normal target with ADS-B in adsb/anomalous modes
            adsb_hex = f"{random.randint(0x100000, 0xFFFFFF):06x}"
            adsb_callsign = f"{''.join(random.choices('ABCDEFGHIJKLMNOPQRSTUVWXYZ', k=3))}{random.randint(100, 9999)}"

        return SyntheticTarget(
            target_id=tid,
            east=east,
            north=north,
            up=up,
            vel_east=vel_east,
            vel_north=vel_north,
            vel_up=vel_up,
            is_anomalous=is_anomalous,
            adsb_hex=adsb_hex,
            adsb_callsign=adsb_callsign,
            created_at=now,
            lifetime_s=random.uniform(120, 600),
        )

    def _update_target(self, target: SyntheticTarget, dt: float):
        """Update target position based on velocity."""
        target.east += target.vel_east * dt
        target.north += target.vel_north * dt
        target.up += target.vel_up * dt

        # Slight random velocity perturbation (maneuvers)
        target.vel_east += random.gauss(0, 0.001) * dt
        target.vel_north += random.gauss(0, 0.001) * dt

    def _target_detection(self, target: SyntheticTarget) -> dict:
        """Generate a detection measurement for a target."""
        pos = (target.east, target.north, target.up)
        vel = (target.vel_east, target.vel_north, target.vel_up)

        delay = _bistatic_delay(pos, self.tx_enu, self.rx_enu)
        doppler = _bistatic_doppler(
            pos, vel, self.tx_enu, self.rx_enu, self.config.fc_hz
        )

        # Add measurement noise
        delay += random.gauss(0, 0.3)  # ~0.3 μs noise
        doppler += random.gauss(0, 2.0)  # ~2 Hz noise

        # SNR depends on distance (closer = stronger)
        dist = _norm(pos)
        base_snr = 25 - 10 * math.log10(max(dist, 1))
        snr = max(base_snr + random.gauss(0, 2), 4.0)

        return {
            "delay": round(delay, 2),
            "doppler": round(doppler, 2),
            "snr": round(snr, 2),
            "_target": target,  # internal, stripped before output
        }

    def _make_adsb_entry(self, target: SyntheticTarget) -> Optional[dict]:
        """Generate ADS-B data for a target (if it has ADS-B and mode allows)."""
        if target.adsb_hex is None:
            return None
        if self.mode not in ("adsb", "anomalous"):
            return None

        # Convert ENU to LLA for ADS-B position
        lat, lon, alt_m = _enu_to_lla(
            target.east, target.north, target.up,
            self.config.rx_lat, self.config.rx_lon, self.rx_alt_m,
        )

        speed_ms = _norm([target.vel_east * 1000, target.vel_north * 1000, 0])
        track_deg = math.degrees(math.atan2(target.vel_east, target.vel_north)) % 360

        return {
            "hex": target.adsb_hex,
            "flight": target.adsb_callsign,
            "lat": round(lat, 5),
            "lon": round(lon, 5),
            "alt_baro": round(alt_m / 0.3048),
            "gs": round(speed_ms * 1.94384, 1),
            "track": round(track_deg, 1),
        }

    def generate_frame(self, timestamp_ms: int) -> dict:
        """Generate a single detection frame.

        Returns a frame dict: {timestamp, delay[], doppler[], snr[], adsb?[]}
        """
        now = timestamp_ms / 1000.0

        # Manage target lifecycle
        # Remove expired targets
        self.targets = [
            t for t in self.targets
            if (now - t.created_at) < t.lifetime_s
        ]

        # Spawn new targets to maintain 3-8 active
        while len(self.targets) < 3:
            self.targets.append(self._spawn_target(now))
        if len(self.targets) < 8 and random.random() < 0.02:
            self.targets.append(self._spawn_target(now))

        # Update positions
        for target in self.targets:
            self._update_target(target, 0.5)  # ~0.5s between frames

        # Generate detections from all targets
        detections = []
        for target in self.targets:
            det = self._target_detection(target)
            detections.append(det)

        # Add some clutter/noise detections (false alarms)
        n_clutter = random.randint(0, 5)
        for _ in range(n_clutter):
            detections.append({
                "delay": round(random.uniform(0, 60), 2),
                "doppler": round(random.uniform(
                    self.config.doppler_min, self.config.doppler_max
                ), 2),
                "snr": round(random.uniform(4, 7), 2),
                "_target": None,
            })

        # Build output frame
        delays = [d["delay"] for d in detections]
        dopplers = [d["doppler"] for d in detections]
        snrs = [d["snr"] for d in detections]

        frame = {
            "timestamp": timestamp_ms,
            "delay": delays,
            "doppler": dopplers,
            "snr": snrs,
        }

        # Add ADS-B data in adsb/anomalous modes
        if self.mode in ("adsb", "anomalous"):
            adsb_list = []
            for det in detections:
                target = det.get("_target")
                if target is not None:
                    adsb_entry = self._make_adsb_entry(target)
                    adsb_list.append(adsb_entry)  # None for no-ADS-B targets
                else:
                    adsb_list.append(None)  # clutter has no ADS-B
            frame["adsb"] = adsb_list

        return frame


def _connect_tcp(host: str, port: int, max_retries: int = 0) -> socket.socket:
    """Connect to the tracker server via TCP with retry logic."""
    attempt = 0
    while True:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(10.0)
            sock.connect((host, port))
            sock.settimeout(None)
            print(f"Connected to {host}:{port}", file=sys.stderr)
            return sock
        except (ConnectionRefusedError, socket.timeout, OSError) as exc:
            attempt += 1
            if 0 < max_retries <= attempt:
                raise
            wait = min(2 ** attempt, 30)
            print(
                f"Connection to {host}:{port} failed ({exc}), "
                f"retrying in {wait}s...",
                file=sys.stderr,
            )
            time.sleep(wait)


def _stream_tcp(generator: SyntheticNodeGenerator, host: str, port: int,
                interval_ms: int = 500):
    """Stream detection frames to the tracker server over TCP."""
    sock = _connect_tcp(host, port)

    try:
        while True:
            timestamp_ms = int(time.time() * 1000)
            frame = generator.generate_frame(timestamp_ms)

            line = json.dumps(frame) + "\n"
            try:
                sock.sendall(line.encode("utf-8"))
            except (BrokenPipeError, ConnectionResetError):
                print("Connection lost, reconnecting...", file=sys.stderr)
                sock.close()
                sock = _connect_tcp(host, port)
                sock.sendall(line.encode("utf-8"))

            # Print summary to stderr
            n_det = len(frame["delay"])
            n_adsb = sum(
                1 for a in frame.get("adsb", []) if a is not None
            ) if "adsb" in frame else 0
            print(
                f"\r[{time.strftime('%H:%M:%S')}] "
                f"Sent frame: {n_det} detections"
                f"{f', {n_adsb} with ADS-B' if n_adsb else ''}"
                f" | targets: {len(generator.targets)}",
                end="", file=sys.stderr,
            )

            time.sleep(interval_ms / 1000.0)

    except KeyboardInterrupt:
        print("\nStopping synthetic node.", file=sys.stderr)
    finally:
        sock.close()


def _stream_http(generator: SyntheticNodeGenerator, url: str,
                 interval_ms: int = 500, batch_size: int = 10):
    """Stream detection frames to the server over HTTP POST."""
    import urllib.request

    frames_buffer = []

    try:
        while True:
            timestamp_ms = int(time.time() * 1000)
            frame = generator.generate_frame(timestamp_ms)
            frames_buffer.append(frame)

            if len(frames_buffer) >= batch_size:
                body = json.dumps({"frames": frames_buffer}).encode("utf-8")
                req = urllib.request.Request(
                    url,
                    data=body,
                    headers={"Content-Type": "application/json"},
                    method="POST",
                )
                try:
                    resp = urllib.request.urlopen(req, timeout=10)
                    result = json.loads(resp.read())
                    print(
                        f"\r[{time.strftime('%H:%M:%S')}] "
                        f"Sent {len(frames_buffer)} frames → "
                        f"{result.get('tracks', '?')} tracks",
                        end="", file=sys.stderr,
                    )
                except Exception as exc:
                    print(f"\nHTTP POST failed: {exc}", file=sys.stderr)
                frames_buffer = []

            time.sleep(interval_ms / 1000.0)

    except KeyboardInterrupt:
        print("\nStopping synthetic node.", file=sys.stderr)


def _replay_file(filepath: str, host: str, port: int, speed: float = 1.0):
    """Replay a .detection file over TCP."""
    with open(filepath, "r") as f:
        content = f.read().strip()
        if not content.startswith("["):
            content = "[" + content + "]"
        frames = json.loads(content)

    if not frames:
        print("No frames in file.", file=sys.stderr)
        return

    sock = _connect_tcp(host, port)

    try:
        prev_ts = frames[0].get("timestamp", 0)
        for i, frame in enumerate(frames):
            ts = frame.get("timestamp", 0)
            dt = (ts - prev_ts) / 1000.0 if ts > prev_ts else 0.5
            prev_ts = ts

            if dt > 0 and speed > 0:
                time.sleep(dt / speed)

            line = json.dumps(frame) + "\n"
            try:
                sock.sendall(line.encode("utf-8"))
            except (BrokenPipeError, ConnectionResetError):
                print("Connection lost, reconnecting...", file=sys.stderr)
                sock.close()
                sock = _connect_tcp(host, port)
                sock.sendall(line.encode("utf-8"))

            n_det = len(frame.get("delay", []))
            print(
                f"\r[{i+1}/{len(frames)}] "
                f"Replayed frame: {n_det} detections",
                end="", file=sys.stderr,
            )

        print(f"\nReplayed {len(frames)} frames.", file=sys.stderr)

    except KeyboardInterrupt:
        print("\nStopping replay.", file=sys.stderr)
    finally:
        sock.close()


def main():
    parser = argparse.ArgumentParser(
        description="Synthetic node for Retina passive radar network"
    )
    parser.add_argument(
        "--host", default="localhost",
        help="Tracker server host (default: localhost)",
    )
    parser.add_argument(
        "--port", type=int, default=3012,
        help="Tracker server TCP port (default: 3012)",
    )
    parser.add_argument(
        "--mode", choices=["detection", "adsb", "anomalous"],
        default="detection",
        help="Data mode: detection-only, with ADS-B, or with anomalous objects",
    )
    parser.add_argument(
        "--interval", type=int, default=500,
        help="Interval between frames in ms (default: 500)",
    )
    parser.add_argument(
        "--file",
        help="Replay a .detection file instead of generating synthetic data",
    )
    parser.add_argument(
        "--speed", type=float, default=1.0,
        help="Replay speed multiplier (default: 1.0)",
    )
    parser.add_argument(
        "--http",
        help="Stream via HTTP POST to this URL instead of TCP",
    )
    parser.add_argument(
        "--config",
        help="Path to node_config.json",
    )
    parser.add_argument(
        "--node-id", default="synthetic-01",
        help="Node identifier (default: synthetic-01)",
    )
    # Node geometry overrides
    parser.add_argument("--rx-lat", type=float)
    parser.add_argument("--rx-lon", type=float)
    parser.add_argument("--rx-alt-ft", type=float)
    parser.add_argument("--tx-lat", type=float)
    parser.add_argument("--tx-lon", type=float)
    parser.add_argument("--tx-alt-ft", type=float)
    parser.add_argument("--fc", type=float, help="Center frequency in Hz")

    args = parser.parse_args()

    # Build node config
    node_config = NodeConfig(node_id=args.node_id)

    if args.config and os.path.exists(args.config):
        with open(args.config) as f:
            cfg = json.load(f)
        for k, v in cfg.items():
            if hasattr(node_config, k):
                setattr(node_config, k, v)

    # CLI overrides
    for attr in ("rx_lat", "rx_lon", "rx_alt_ft", "tx_lat", "tx_lon", "tx_alt_ft"):
        cli_val = getattr(args, attr.replace("-", "_"), None)
        if cli_val is not None:
            setattr(node_config, attr, cli_val)
    if args.fc is not None:
        node_config.fc_hz = args.fc

    print(f"Synthetic Node: {node_config.node_id}", file=sys.stderr)
    print(f"  Mode: {args.mode}", file=sys.stderr)
    print(
        f"  RX: ({node_config.rx_lat:.6f}, {node_config.rx_lon:.6f}) "
        f"@ {node_config.rx_alt_ft:.0f} ft",
        file=sys.stderr,
    )
    print(
        f"  TX: ({node_config.tx_lat:.6f}, {node_config.tx_lon:.6f}) "
        f"@ {node_config.tx_alt_ft:.0f} ft",
        file=sys.stderr,
    )
    print(f"  FC: {node_config.fc_hz/1e6:.1f} MHz", file=sys.stderr)

    if args.file:
        print(f"  Replaying: {args.file} @ {args.speed}x speed", file=sys.stderr)
        _replay_file(args.file, args.host, args.port, args.speed)
    else:
        generator = SyntheticNodeGenerator(node_config, mode=args.mode)

        if args.http:
            print(f"  Streaming to: {args.http} (HTTP)", file=sys.stderr)
            _stream_http(generator, args.http, args.interval)
        else:
            print(
                f"  Streaming to: {args.host}:{args.port} (TCP)",
                file=sys.stderr,
            )
            _stream_tcp(generator, args.host, args.port, args.interval)


if __name__ == "__main__":
    main()
