# teleoperate
```
uv run lerobot-teleoperate \
    --robot.type=template_follower \
    --teleop.type=template_leader \
    --robot.cameras="{ front: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
    --fps=60
```


# dev
```
uv pip uninstall lerobot-robot-template lerobot-teleoperator-template

uv pip install robots/template/ teleoperators/template/
```

