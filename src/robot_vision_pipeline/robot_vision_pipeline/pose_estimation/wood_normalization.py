"""Wood pose normalization helpers shared by vision nodes."""

from __future__ import annotations

from typing import Optional, Tuple

WOOD_Z_BAND_LOW_20MM = 'LOW_20MM'
WOOD_Z_BAND_HIGH_60MM = 'HIGH_60MM'
WOOD_Z_BAND_BETWEEN_BANDS = 'BETWEEN_BANDS'
WOOD_Z_BAND_OUT_OF_RANGE = 'OUT_OF_RANGE'
WOOD_CUBE_YAW_OUTPUT_DEG = 0.0


def normalize_wood_yaw_deg(angle_deg: float) -> float:
    """Normalize an axis yaw in degrees to the wood gripper range [-45, 45]."""
    yaw_deg = float(angle_deg) % 180.0
    if yaw_deg > 90.0:
        yaw_deg = 180.0 - yaw_deg
    if yaw_deg > 45.0:
        yaw_deg -= 90.0
    return float(yaw_deg)


def wood_cube_yaw_output_deg(_yaw_raw_deg: float = 0.0) -> float:
    """Return the fixed grasp yaw for cube-shaped wood objects."""
    return WOOD_CUBE_YAW_OUTPUT_DEG


def normalize_wood_z_output_m(
    z_out_m: float,
    last_valid_m: Optional[float] = None,
) -> Tuple[float, float, str, Optional[float], bool]:
    """Normalize wood Z with last-valid hysteresis."""
    z_raw_cm = float(z_out_m) * 100.0
    eps = 1e-9
    if (1.0 - eps) <= z_raw_cm <= (3.0 + eps):
        return 0.020, z_raw_cm, WOOD_Z_BAND_LOW_20MM, 0.020, False
    if (4.0 - eps) <= z_raw_cm <= (7.0 + eps):
        return 0.060, z_raw_cm, WOOD_Z_BAND_HIGH_60MM, 0.060, False

    band = (
        WOOD_Z_BAND_BETWEEN_BANDS
        if (3.0 + eps) < z_raw_cm < (4.0 - eps)
        else WOOD_Z_BAND_OUT_OF_RANGE
    )
    if last_valid_m is not None:
        return float(last_valid_m), z_raw_cm, band, float(last_valid_m), True
    return float(z_out_m), z_raw_cm, band, None, False
