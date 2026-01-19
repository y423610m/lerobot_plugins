# How to install
```
sudo apt update
sudo apt install -y build-essential python3-dev python3-evdev
```

# Notes

## editable install is not allowed.
`pip install -e ./path/to/module` and `uv add ./path/to/module` are not supported by lerobot. `register_third_party_devices` searches modulesa with `lerobot_` prefixes but editable modules' names become like `__editable__modulename` aand they cannot be found. Use `uv pip install` or `uv add --no-editable`
