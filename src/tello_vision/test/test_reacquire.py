import pytest

from tello_vision.reacquire import select_reacquire_candidate


DEFAULTS = dict(
    timeout_s=2.0,
    image_width=640,
    image_height=480,
    center_ratio=0.15,
    area_ratio_min=0.5,
    area_ratio_max=2.0,
)


def test_selects_closest_candidate_within_time_distance_and_area_gates():
    index = select_reacquire_candidate(
        (100.0, 100.0, 80.0, 160.0),
        [(120.0, 105.0, 78.0, 155.0), (110.0, 102.0, 81.0, 159.0)],
        elapsed_s=1.0,
        **DEFAULTS,
    )
    assert index == 1


@pytest.mark.parametrize(
    'candidate,elapsed',
    [
        ((500.0, 300.0, 80.0, 160.0), 1.0),
        ((105.0, 105.0, 20.0, 20.0), 1.0),
        ((105.0, 105.0, 200.0, 300.0), 1.0),
        ((105.0, 105.0, 80.0, 160.0), 2.01),
    ],
)
def test_rejects_mismatched_or_late_candidates(candidate, elapsed):
    assert select_reacquire_candidate(
        (100.0, 100.0, 80.0, 160.0), [candidate], elapsed_s=elapsed, **DEFAULTS
    ) is None


def test_rejects_invalid_bounds():
    with pytest.raises(ValueError):
        select_reacquire_candidate(
            (0.0, 0.0, 10.0, 10.0),
            [],
            elapsed_s=0.0,
            **{**DEFAULTS, 'area_ratio_min': 2.0, 'area_ratio_max': 1.0},
        )
