from swarm_autonomy_mapping.merge import merge_log_odds


def test_merge_accumulates():
    assert merge_log_odds(10, 5) == 15


def test_merge_saturates_positive():
    assert merge_log_odds(120, 50, clamp=127) == 127


def test_merge_saturates_negative():
    assert merge_log_odds(-120, -50, clamp=127) == -127
