from tello_mock.protocol import MockTelloProtocol


def test_requires_command_mode_and_rejects_duplicate_takeoff():
    protocol = MockTelloProtocol()
    assert protocol.handle('takeoff').startswith('error')
    assert protocol.handle('command') == 'ok'
    assert protocol.handle('takeoff') == 'ok'
    assert protocol.handle('takeoff').startswith('error')
    assert protocol.takeoff_count == 1


def test_rc_has_no_ack_and_land_updates_state():
    protocol = MockTelloProtocol()
    protocol.handle('command')
    protocol.handle('takeoff')
    assert protocol.handle('rc 1 2 3 4') is None
    assert protocol.handle('land') == 'ok'
    assert protocol.airborne is False
    assert protocol.land_count == 1
