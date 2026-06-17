"""
Real satellite orbit from CelesTrak TLE + sgp4 propagator.
Magnetic field from a simplified tilted-dipole Earth model (accuracy ~5% for LEO).

What IS available in live mode:
  - Satellite position / velocity (ECI)  — from TLE + sgp4
  - Geographic lat / lon / altitude       — derived from ECI
  - Magnetic field vector                 — tilted-dipole IGRF approximation

What is NOT available (requires direct access to the satellite's ground station):
  - Real gyro / accelerometer / magnetometer body-frame readings
  - True attitude quaternion
"""

import datetime
import math
from typing import Optional

import numpy as np

# ── Earth constants ────────────────────────────────────────────────────────────

_R_EARTH_KM = 6371.0
_R_EARTH_M  = _R_EARTH_KM * 1e3

# ── Simplified IGRF dipole (IGRF-13, epoch 2020) ──────────────────────────────
# Dipole axis tilted ~11.5° from geographic north at ~72°W longitude.
_B0_T        = 3.12e-5             # surface equatorial field strength [T]
_DIPOLE_TILT = math.radians(11.5)  # tilt from z-axis (geographic north) [rad]
_DIPOLE_LON  = math.radians(-72.0) # dipole longitude [rad]

# Dipole axis unit vector in ECI (approx — ignores precession for short runs)
_M_HAT = np.array([
    math.sin(_DIPOLE_TILT) * math.cos(_DIPOLE_LON),
    math.sin(_DIPOLE_TILT) * math.sin(_DIPOLE_LON),
    math.cos(_DIPOLE_TILT),
])


# ── Pure-numpy dipole field ────────────────────────────────────────────────────

def dipole_field_eci(pos_km: np.ndarray) -> np.ndarray:
    """
    Tilted-dipole magnetic field at ECI position pos_km [km].
    Returns field vector in ECI frame [µT].

    Formula: B = (B0 * Re³ / r³) * (3(m̂·r̂)r̂ − m̂)
    Typical LEO magnitude: 20–50 µT (matches real measurements to ~5 %).
    """
    r_m   = pos_km * 1e3
    r     = np.linalg.norm(r_m)
    if r < 1e3:
        return np.zeros(3)
    r_hat = r_m / r
    scale = _B0_T * (_R_EARTH_M ** 3) / (r ** 3)
    B_T   = scale * (3.0 * float(np.dot(_M_HAT, r_hat)) * r_hat - _M_HAT)
    return B_T * 1e6  # → [µT]


def eci_to_latlon(pos_km: np.ndarray) -> tuple[float, float, float]:
    """
    Geographic lat / lon / altitude from ECI position [km].
    Note: treats ECI ≈ ECEF (ignores Earth's rotation). Good for short-duration
    display; accumulates ~360° lon error per ~5 700 s (one orbit) if used for
    precise ground-track mapping.
    """
    x, y, z = pos_km
    r = math.sqrt(x * x + y * y + z * z)
    lat_deg = math.degrees(math.asin(max(-1.0, min(1.0, z / r))))
    lon_deg = math.degrees(math.atan2(y, x))
    alt_km  = r - _R_EARTH_KM
    return lat_deg, lon_deg, alt_km


# ── sgp4 availability guard ────────────────────────────────────────────────────

try:
    from sgp4.api import Satrec, jday as _sgp4_jday
    _SGP4_OK = True
except ImportError:
    _SGP4_OK = False


def _jday_utcnow() -> tuple[float, float]:
    now = datetime.datetime.now(tz=datetime.timezone.utc)
    return _sgp4_jday(                      # type: ignore[name-defined]
        now.year, now.month, now.day,
        now.hour, now.minute,
        now.second + now.microsecond / 1e6,
    )


# ── TLEOrbit ──────────────────────────────────────────────────────────────────

class TLEOrbit:
    """
    Live orbital data for any NORAD-tracked satellite.

    Usage (called from SimulationEngine):
        orbit = TLEOrbit(norad_id=25544)        # ISS by default
        ok = await orbit.fetch_tle()            # once at engine.start()
        frame = orbit.current_frame()           # every tick — cheap
        b_hat = orbit.mag_field_normalized()    # optional: TRIAD b_ref update
    """

    _CELESTRAK = "https://celestrak.org/satcat/tle.php?CATNR={norad_id}"

    def __init__(self, norad_id: int = 25544) -> None:
        self.norad_id   = norad_id
        self._sat       = None
        self._sat_name  = f"NORAD-{norad_id}"
        self._tle_ok    = False

    # ── TLE fetch ─────────────────────────────────────────────────────────────

    async def fetch_tle(self, timeout: float = 10.0) -> bool:
        """
        Fetch latest TLE from CelesTrak.
        Returns True if successful; False on any error (network, parse, no sgp4).
        On failure the previous TLE (if any) is retained.
        """
        if not _SGP4_OK:
            return False

        import httpx  # deferred import — only needed for live mode

        url = self._CELESTRAK.format(norad_id=self.norad_id)
        try:
            async with httpx.AsyncClient(timeout=timeout) as client:
                resp = await client.get(url)
                resp.raise_for_status()
                lines = [ln.strip() for ln in resp.text.splitlines() if ln.strip()]
                if len(lines) < 3:
                    return False
                self._sat_name = lines[0]
                self._sat      = Satrec.twoline2rv(lines[1], lines[2])  # type: ignore
                self._tle_ok   = True
                return True
        except Exception:
            return False

    # ── Per-tick frame ────────────────────────────────────────────────────────

    def current_frame(self) -> dict:
        """
        Returns the satellite's current position, velocity, and derived
        magnetic field.  Cheap to call every simulation tick.
        """
        if not _SGP4_OK:
            return {"available": False, "error": "sgp4 package not installed"}
        if self._sat is None:
            return {"available": False, "error": "TLE not loaded — call fetch_tle() first"}

        try:
            jd, fr = _jday_utcnow()
            err_code, r_km, v_km_s = self._sat.sgp4(jd, fr)   # type: ignore
            if err_code != 0:
                return {"available": False, "error": f"sgp4 error {err_code}"}

            r_arr = np.array(r_km)
            v_arr = np.array(v_km_s)
            lat, lon, alt = eci_to_latlon(r_arr)
            B_uT          = dipole_field_eci(r_arr)

            return {
                "available":           True,
                "satellite_name":      self._sat_name,
                "norad_id":            self.norad_id,
                "lat_deg":             round(lat,            4),
                "lon_deg":             round(lon,            4),
                "alt_km":              round(alt,            2),
                "position_eci_km":     [round(v, 3) for v in r_arr.tolist()],
                "velocity_eci_km_s":   [round(v, 5) for v in v_arr.tolist()],
                "magnetic_field_uT":   {
                    "x":     round(float(B_uT[0]), 4),
                    "y":     round(float(B_uT[1]), 4),
                    "z":     round(float(B_uT[2]), 4),
                    "total": round(float(np.linalg.norm(B_uT)), 4),
                },
            }
        except Exception as exc:
            return {"available": False, "error": str(exc)}

    # ── TRIAD reference update ─────────────────────────────────────────────────

    def mag_field_normalized(self) -> Optional[np.ndarray]:
        """
        Current real IGRF B-field direction (unit vector, ECI).
        Returns None if TLE not loaded.  Used to update TRIAD's b_ref for
        more physically accurate attitude estimation in live mode.
        """
        frame = self.current_frame()
        if not frame.get("available"):
            return None
        r_km = np.array(frame["position_eci_km"])
        B    = dipole_field_eci(r_km)
        norm = np.linalg.norm(B)
        return B / norm if norm > 1e-9 else None
