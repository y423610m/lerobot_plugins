# dev
```
uv pip uninstall lerobot-robot-four-mecanum-wheels-car lerobot-teleoperator-four-mecanum-wheels-car
uv pip install robots/four_mecanum_wheels_car/ teleoperators/four_mecanum_wheels_car/
```

# teleoperate
```
uv run lerobot-teleoperate \
    --robot.type=four_mecanum_wheels_car_follower \
    --teleop.type=four_mecanum_wheels_car_leader \
    --fps=2
```
