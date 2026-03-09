import { useState } from "react";
import "./SearchForm.css";

export default function SearchForm({ onSearch, loading }) {
  const [lat, setLat] = useState("");
  const [lon, setLon] = useState("");
  const [altitude, setAltitude] = useState("");
  const [source, setSource] = useState("au");
  const [geoError, setGeoError] = useState(null);

  function handleSubmit(e) {
    e.preventDefault();
    const parsedLat = parseFloat(lat);
    const parsedLon = parseFloat(lon);
    if (isNaN(parsedLat) || isNaN(parsedLon)) return;
    onSearch({
      lat: parsedLat,
      lon: parsedLon,
      altitude: parseFloat(altitude) || 0,
      source,
    });
  }

  function useMyLocation() {
    if (!navigator.geolocation) {
      setGeoError("Geolocation not supported by your browser");
      return;
    }
    setGeoError(null);
    navigator.geolocation.getCurrentPosition(
      (pos) => {
        setLat(pos.coords.latitude.toFixed(6));
        setLon(pos.coords.longitude.toFixed(6));
        if (pos.coords.altitude != null) {
          setAltitude(Math.round(pos.coords.altitude).toString());
        }
      },
      (err) => setGeoError(err.message)
    );
  }

  return (
    <form className="search-form" onSubmit={handleSubmit}>
      <h2>Location</h2>

      <div className="field-row">
        <label>
          Latitude
          <input
            type="number"
            step="any"
            min={-90}
            max={90}
            value={lat}
            onChange={(e) => setLat(e.target.value)}
            placeholder="e.g. 38.8977"
            required
          />
        </label>
        <label>
          Longitude
          <input
            type="number"
            step="any"
            min={-180}
            max={180}
            value={lon}
            onChange={(e) => setLon(e.target.value)}
            placeholder="e.g. -77.0365"
            required
          />
        </label>
      </div>

      <div className="field-row">
        <label>
          Altitude (m)
          <input
            type="number"
            step="any"
            min={0}
            value={altitude}
            onChange={(e) => setAltitude(e.target.value)}
            placeholder="Optional"
          />
        </label>
        <label>
          Data Source
          <select value={source} onChange={(e) => setSource(e.target.value)}>
            <option value="au">Australia (ACMA)</option>
            <option value="us">United States (FCC)</option>
            <option value="ca">Canada (ISED)</option>
          </select>
        </label>
      </div>

      <div className="form-actions">
        <button type="submit" className="btn-primary" disabled={loading}>
          {loading ? "Searching…" : "Find Towers"}
        </button>
        <button
          type="button"
          className="btn-secondary"
          onClick={useMyLocation}
          disabled={loading}
        >
          Use My Location
        </button>
      </div>

      {geoError && <p className="geo-error">{geoError}</p>}
    </form>
  );
}
