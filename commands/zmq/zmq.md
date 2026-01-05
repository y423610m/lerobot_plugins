# teleoperate

## host (raspberry pi)
```
uv run scripts/zmq/zmq_host.py \
    --config commands/zmq/zmq_host.yaml
```

## client (desktop)
```
uv run lerobot-teleoperate \
    --config commands/zmq/zmq_client.yaml \
    --fps=60


    <!-- --robot.cameras="{ front: {type: opencv, index_or_path: 1, width: 640, height: 480, fps: 30}}" \ -->

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