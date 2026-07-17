from tello_vision.latest_slot import LatestSlot


def test_latest_slot_drops_intermediate_values():
    slot = LatestSlot[int]()
    slot.put(1)
    sequence = slot.put(2)
    new_sequence, value = slot.wait_next(0, timeout=0.01)
    assert new_sequence == sequence
    assert value == 2


def test_latest_slot_close_unblocks_reader():
    slot = LatestSlot[int]()
    slot.close()
    _, value = slot.wait_next(0, timeout=0.01)
    assert value is None
