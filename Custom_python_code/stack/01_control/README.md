Control: make the car move, stop, and hold a speed.

Legal inputs: velocity, acceleration, your last `VehicleControl`.

Starter you already have: `drive_throttle.py`
Speed hold: `hold_speed.py` — proportional throttle/brake caps output to hold `TARGET_SPEED` instead of flooring it.

Next here: swap the P controller for PI (integral term) to kill steady-state error, then try a curve/hill to see it matter.
