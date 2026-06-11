from swarm_autonomy_control.pid import PID


def test_proportional_drives_toward_zero_error():
    pid = PID(kp=2.0)
    assert pid.step(error=3.0, dt=0.1) == 6.0


def test_output_is_clamped():
    pid = PID(kp=10.0, out_min=-1.0, out_max=1.0)
    assert pid.step(error=5.0, dt=0.1) == 1.0
    assert pid.step(error=-5.0, dt=0.1) == -1.0


def test_integral_accumulates_and_is_limited():
    pid = PID(kp=0.0, ki=1.0, integral_limit=2.0)
    out = [pid.step(1.0, 1.0) for _ in range(5)]
    assert out[-1] == 2.0  # integral saturates at the limit


def test_settles_on_a_simple_first_order_plant():
    # x_{k+1} = x_k + u*dt, target 0; PID should drive x toward 0 and stay bounded.
    pid = PID(kp=1.0, ki=0.0, kd=0.1, out_min=-10, out_max=10)
    x, dt = 5.0, 0.1
    for _ in range(300):
        u = pid.step(error=(0.0 - x), dt=dt)
        x += u * dt
    assert abs(x) < 0.05
