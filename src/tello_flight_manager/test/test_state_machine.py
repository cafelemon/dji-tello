from tello_flight_manager.state_machine import FlightState, FlightStateMachine


def ready_machine():
    machine = FlightStateMachine()
    machine.update_links(True, True, True)
    assert machine.state == FlightState.CONNECTED
    machine.update_links(True, True, True)
    assert machine.state == FlightState.READY
    return machine


def test_low_battery_and_duplicate_takeoff_are_rejected():
    machine = ready_machine()
    assert not machine.request_takeoff(19.0, 20.0).accepted
    assert machine.request_takeoff(80.0, 20.0).accepted
    assert not machine.request_takeoff(80.0, 20.0).accepted


def test_target_reacquire_and_timeout_land():
    machine = ready_machine()
    machine.request_takeoff(80.0, 20.0)
    machine.takeoff_result(True, 'ok')
    machine.update_tracking(False, 10.0)
    assert machine.state == FlightState.LOST_TARGET
    machine.update_tracking(True, 11.0)
    assert machine.state == FlightState.TRACKING
    machine.update_tracking(False, 20.0)
    assert machine.tick(24.9) is None
    assert machine.tick(25.0) == 'land'


def test_unhealthy_link_forces_error_and_land_action():
    machine = ready_machine()
    machine.request_takeoff(80.0, 20.0)
    machine.takeoff_result(True, 'ok')
    machine.update_links(False, True, True)
    assert machine.state == FlightState.ERROR
    assert machine.tick(1.0) == 'land'
    assert not machine.motion_allowed
