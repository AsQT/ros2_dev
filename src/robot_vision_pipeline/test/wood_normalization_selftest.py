#!/usr/bin/env python3
"""Self-test for wood yaw/Z normalization rules."""

from __future__ import annotations

import math

from robot_vision_pipeline.pose_estimation.wood_normalization import (
    normalize_wood_z_output_m,
    wood_cube_yaw_output_deg,
    WOOD_Z_BAND_BETWEEN_BANDS,
    WOOD_Z_BAND_HIGH_60MM,
    WOOD_Z_BAND_LOW_20MM,
    WOOD_Z_BAND_OUT_OF_RANGE,
)


def assert_close(actual: float, expected: float, label: str) -> None:
    if not math.isclose(actual, expected, abs_tol=1e-9):
        raise AssertionError(f'{label}: expected {expected}, got {actual}')


def test_yaw() -> None:
    raw_values = [0.0, 30.0, 45.0, 60.0, 75.0, 90.0, -30.0, 135.0]
    for raw in raw_values:
        actual = wood_cube_yaw_output_deg(raw)
        assert_close(actual, 0.0, f'yaw {raw}')
        print(f'[PASS] yaw_raw={raw:.0f} -> yaw_output={actual:.0f}')


def test_z() -> None:
    cases = [
        (0.010, 0.020, WOOD_Z_BAND_LOW_20MM, 0.020, False),
        (0.020, 0.020, WOOD_Z_BAND_LOW_20MM, 0.020, False),
        (0.030, 0.020, WOOD_Z_BAND_LOW_20MM, 0.020, False),
        (0.040, 0.060, WOOD_Z_BAND_HIGH_60MM, 0.060, False),
        (0.055, 0.060, WOOD_Z_BAND_HIGH_60MM, 0.060, False),
        (0.070, 0.060, WOOD_Z_BAND_HIGH_60MM, 0.060, False),
        (0.008, 0.008, WOOD_Z_BAND_OUT_OF_RANGE, None, False),
        (0.035, 0.035, WOOD_Z_BAND_BETWEEN_BANDS, None, False),
        (0.080, 0.080, WOOD_Z_BAND_OUT_OF_RANGE, None, False),
    ]
    for raw_m, expected_m, expected_band, expected_last, expected_used_last in cases:
        actual_m, raw_cm, band, last_valid_m, used_last = normalize_wood_z_output_m(raw_m)
        assert_close(actual_m, expected_m, f'z {raw_m}')
        if band != expected_band:
            raise AssertionError(
                f'z {raw_m}: expected band={expected_band}, got {band}'
            )
        if expected_last is None:
            if last_valid_m is not None:
                raise AssertionError(f'z {raw_m}: expected last_valid=None, got {last_valid_m}')
        else:
            assert_close(float(last_valid_m), expected_last, f'z last valid {raw_m}')
        if used_last != expected_used_last:
            raise AssertionError(
                f'z {raw_m}: expected used_last={expected_used_last}, got {used_last}'
            )
        assert_close(raw_cm, raw_m * 100.0, f'z raw cm {raw_m}')
        print(
            f'[PASS] z_raw={raw_cm:.1f}cm band={band} '
            f'-> z_out={actual_m:.3f}m last={last_valid_m}'
        )


def test_z_sequences() -> None:
    sequences = [
        ([2.1, 2.4, 2.8], [0.020, 0.020, 0.020]),
        ([5.0, 5.4, 6.2], [0.060, 0.060, 0.060]),
        ([2.8, 3.2, 3.6, 3.9], [0.020, 0.020, 0.020, 0.020]),
        ([4.2, 3.8, 3.5, 3.2], [0.060, 0.060, 0.060, 0.060]),
        ([5.2, 8.0, 9.0], [0.060, 0.060, 0.060]),
    ]
    for raw_cm_values, expected_m_values in sequences:
        last_valid_m = None
        actual_values = []
        bands = []
        for raw_cm in raw_cm_values:
            out_m, _, band, last_valid_m, _ = normalize_wood_z_output_m(
                raw_cm / 100.0,
                last_valid_m,
            )
            actual_values.append(out_m)
            bands.append(band)
        for actual_m, expected_m in zip(actual_values, expected_m_values):
            assert_close(actual_m, expected_m, f'z sequence {raw_cm_values}')
        print(
            f'[PASS] z_sequence raw_cm={raw_cm_values} '
            f'bands={bands} outputs_m={actual_values}'
        )


def main() -> None:
    test_yaw()
    test_z()
    test_z_sequences()
    print('ALL WOOD NORMALIZATION SELF-TESTS PASSED')


if __name__ == '__main__':
    main()
