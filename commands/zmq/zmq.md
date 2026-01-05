# teleoperate

## host (raspberry pi)
```
uv run scripts/zmq/zmq_host.py \
    --config commands/zmq/zmq_host.yaml
```

## client (desktop)
```
uv run lerobot-teleoperate \
    --config commands/zmq/zmq_host.yaml \
    --robot.type=template_follower \
    --teleop.type=zmq \
    --robot.cameras="{ front: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \
    --fps=60
```


# dev
```
uv pip uninstall lerobot-robot-zmq
uv pip install robots/zmq/
```

# Config example
## Desktop
lerobot_teleoperate.py
```
teleop:
    type: XXX_leader
robot:
    type: zmq_client_follower
```

## RaspberryPi
zmq_host.py
```
robot:
    type: 
host:
    ip: XXX
```