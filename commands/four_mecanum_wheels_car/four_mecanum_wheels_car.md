# dev
```
uv pip uninstall lerobot-robot-four-mecanum-wheels-car lerobot-teleoperator-four-mecanum-wheels-car
uv pip install robots/four_mecanum_wheels_car/ teleoperators/four_mecanum_wheels_car/
```

# teleoperate
```
uv run lerobot-teleoperate \
    --config commands/four_mecanum_wheels_car/four_mecanum_wheels_car.yaml \
    --fps=30
```
