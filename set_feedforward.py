import json
import socket
from collections import OrderedDict as OD

ip = "192.168.1.100"

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((ip, 1237))

    sin_amplitudes = [0.5, 0., 0., 0., 0.]
    cos_amplitudes = [0., 0., 0.1, 0., 0.]

    msg = OD([("sin_amplitudes", sin_amplitudes), ("cos_amplitudes", cos_amplitudes)])
    raw_msg = json.dumps(msg, separators=(",", ":"))
    s.sendall(raw_msg.encode() + b"\n")
    data = s.recv(1024)
    print(data.decode())
