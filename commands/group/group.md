# dev
```
uv pip uninstall lerobot-robot-group lerobot-teleoperator-group
uv pip install robots/group/ teleoperators/group/
```

# teleoperate
```
uv run lerobot-teleoperate \
    --config commands/group/group.yaml \
    --fps=2
```
