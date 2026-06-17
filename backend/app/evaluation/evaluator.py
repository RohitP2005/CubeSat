"""
PerformanceEvaluator — per-cycle attitude error recording and aggregate statistics.

Tracks angular and per-axis (roll / pitch / yaw) errors for both the EKF and TRIAD
estimators in capped circular buffers so memory is bounded regardless of run length.

A configurable convergence window excludes the first N steps from aggregate metrics
so that EKF startup transients do not pollute steady-state RMSE values.

All error values are in degrees.
"""

import collections

import numpy as np

from ..ekf.quaternion import to_euler, error_angle


# ── Angle-wrapping helper ─────────────────────────────────────────────────────

def _wrap_deg(diff: float) -> float:
    """Wrap an angular difference to (-180, 180] degrees."""
    return (diff + 180.0) % 360.0 - 180.0


# ── Per-algorithm buffer ──────────────────────────────────────────────────────

class _ChannelStats:
    """
    Stores four error signals (angular, roll, pitch, yaw) plus the simulation
    step index in five parallel capped deques of equal maxlen.

    The step index is stored so that the convergence window can be applied
    correctly even after the buffer wraps around.
    """

    _EMPTY = {
        "sample_count":   0,
        "mean_error_deg": None,
        "rmse_angular":   None,
        "rmse_roll":      None,
        "rmse_pitch":     None,
        "rmse_yaw":       None,
    }

    def __init__(self, maxlen: int) -> None:
        self._angular: collections.deque[float] = collections.deque(maxlen=maxlen)
        self._roll:    collections.deque[float] = collections.deque(maxlen=maxlen)
        self._pitch:   collections.deque[float] = collections.deque(maxlen=maxlen)
        self._yaw:     collections.deque[float] = collections.deque(maxlen=maxlen)
        self._steps:   collections.deque[int]   = collections.deque(maxlen=maxlen)

    def push(
        self,
        *,
        angular: float,
        roll:    float,
        pitch:   float,
        yaw:     float,
        step:    int,
    ) -> None:
        self._angular.append(angular)
        self._roll.append(roll)
        self._pitch.append(pitch)
        self._yaw.append(yaw)
        self._steps.append(step)

    @property
    def latest(self) -> dict | None:
        if not self._angular:
            return None
        return {
            "angular_error_deg": self._angular[-1],
            "roll_error_deg":    self._roll[-1],
            "pitch_error_deg":   self._pitch[-1],
            "yaw_error_deg":     self._yaw[-1],
        }

    def aggregate(self, convergence_steps: int) -> dict:
        """
        Compute RMSE and mean over samples with step >= convergence_steps.
        Falls back to all samples when not enough post-convergence data exists.
        """
        if not self._angular:
            return dict(self._EMPTY)

        steps   = np.array(self._steps,   dtype=np.int64)
        angular = np.array(self._angular)
        roll    = np.array(self._roll)
        pitch   = np.array(self._pitch)
        yaw     = np.array(self._yaw)

        mask = steps >= convergence_steps
        if not mask.any():
            mask = np.ones(len(steps), dtype=bool)   # not enough data yet

        ang  = angular[mask]
        rol  = roll[mask]
        pit  = pitch[mask]
        yaw_ = yaw[mask]

        return {
            "sample_count":   int(len(ang)),
            "mean_error_deg": float(np.mean(ang)),
            "rmse_angular":   float(np.sqrt(np.mean(ang ** 2))),
            "rmse_roll":      float(np.sqrt(np.mean(rol ** 2))),
            "rmse_pitch":     float(np.sqrt(np.mean(pit ** 2))),
            "rmse_yaw":       float(np.sqrt(np.mean(yaw_ ** 2))),
        }

    def clear(self) -> None:
        self._angular.clear()
        self._roll.clear()
        self._pitch.clear()
        self._yaw.clear()
        self._steps.clear()

    def __len__(self) -> int:
        return len(self._angular)


# ── Public evaluator ──────────────────────────────────────────────────────────

class PerformanceEvaluator:
    """
    Records per-cycle attitude errors and exposes aggregate statistics.

    Usage
    -----
    evaluator = PerformanceEvaluator()
    evaluator.record(q_ekf, q_triad_or_None, q_true)
    print(evaluator.summary())         # full RMSE breakdown
    print(evaluator.angular_error_ekf())  # latest-step scalar
    evaluator.clear()                  # reset for next run
    """

    def __init__(
        self,
        maxlen:            int = 10_000,
        convergence_steps: int = 100,
    ) -> None:
        """
        maxlen:            Maximum samples retained per channel (memory cap).
        convergence_steps: Steps excluded from aggregate metrics (EKF warmup).
        """
        self._convergence_steps = convergence_steps
        self._ekf_buf   = _ChannelStats(maxlen)
        self._triad_buf = _ChannelStats(maxlen)
        self._step      = 0

    # ── Recording ─────────────────────────────────────────────────────────────

    def record(
        self,
        q_ekf:   np.ndarray,
        q_triad: np.ndarray | None,
        q_true:  np.ndarray,
    ) -> None:
        """
        Record one simulation cycle.

        q_ekf:   EKF quaternion estimate   (always present)
        q_triad: TRIAD quaternion estimate, or None on singularity
        q_true:  Ground-truth quaternion
        """
        r_t, p_t, y_t = to_euler(q_true)
        r_e, p_e, y_e = to_euler(q_ekf)

        # Euler differences in degrees (roll/yaw wrapped to ±180°)
        roll_e  = float(abs(_wrap_deg(np.degrees(r_e) - np.degrees(r_t))))
        pitch_e = float(abs(np.degrees(p_e) - np.degrees(p_t)))
        yaw_e   = float(abs(_wrap_deg(np.degrees(y_e) - np.degrees(y_t))))

        self._ekf_buf.push(
            angular = float(np.degrees(error_angle(q_ekf, q_true))),
            roll    = roll_e,
            pitch   = pitch_e,
            yaw     = yaw_e,
            step    = self._step,
        )

        if q_triad is not None:
            r_tr, p_tr, y_tr = to_euler(q_triad)
            self._triad_buf.push(
                angular = float(np.degrees(error_angle(q_triad, q_true))),
                roll    = float(abs(_wrap_deg(np.degrees(r_tr) - np.degrees(r_t)))),
                pitch   = float(abs(np.degrees(p_tr) - np.degrees(p_t))),
                yaw     = float(abs(_wrap_deg(np.degrees(y_tr) - np.degrees(y_t)))),
                step    = self._step,
            )

        self._step += 1

    # ── Latest-cycle access ───────────────────────────────────────────────────

    def angular_error_ekf(self) -> float | None:
        """Latest EKF angular error in degrees, or None before first record()."""
        latest = self._ekf_buf.latest
        return latest["angular_error_deg"] if latest else None

    def angular_error_triad(self) -> float | None:
        """Latest TRIAD angular error in degrees, or None if no valid TRIAD estimate."""
        latest = self._triad_buf.latest
        return latest["angular_error_deg"] if latest else None

    def latest_ekf(self) -> dict | None:
        """Latest per-axis EKF errors, or None before first record()."""
        return self._ekf_buf.latest

    def latest_triad(self) -> dict | None:
        """Latest per-axis TRIAD errors, or None if no TRIAD estimate yet."""
        return self._triad_buf.latest

    # ── Aggregate statistics ──────────────────────────────────────────────────

    def summary(self) -> dict:
        """
        Full RMSE breakdown over the post-convergence window.
        All error values are in degrees.

        Keys:
          step_count        — total steps recorded (including pre-convergence)
          convergence_steps — window applied (configurable at init)
          ekf / triad:
            sample_count   — post-convergence samples
            mean_error_deg — mean angular error
            rmse_angular   — RMS total angular error
            rmse_roll      — RMS roll error
            rmse_pitch     — RMS pitch error
            rmse_yaw       — RMS yaw error
        """
        return {
            "step_count":        self._step,
            "convergence_steps": self._convergence_steps,
            "ekf":               self._ekf_buf.aggregate(self._convergence_steps),
            "triad":             self._triad_buf.aggregate(self._convergence_steps),
        }

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def step_count(self) -> int:
        return self._step

    @property
    def ekf_buffer_size(self) -> int:
        return len(self._ekf_buf)

    @property
    def triad_buffer_size(self) -> int:
        return len(self._triad_buf)

    # ── Reset ─────────────────────────────────────────────────────────────────

    def clear(self) -> None:
        """Reset all buffers and step counter."""
        self._ekf_buf.clear()
        self._triad_buf.clear()
        self._step = 0
