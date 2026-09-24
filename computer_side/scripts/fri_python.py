# current_thetta_[0], current_thetta_[1], current_thetta_[2], current_thetta_[3], current_thetta_[4], current_thetta_[5], current_thetta_[6], 
# current_pos_[0], current_pos_[1], current_pos_[2],
# current_rot_(0,0), current_rot_(0,1), current_rot_(0,2),
# current_rot_(1,0), current_rot_(1,1), current_rot_(1,2),
# current_rot_(2,0), current_rot_(2,1), current_rot_(2,2),
# force_msg_[0], force_msg_[1], force_msg_[2], force_msg_[3], force_msg_[4], force_msg_[5];

import json
import socket

import kuka_fri_py as fri
import numpy as np

haptic_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
haptic_sock.bind(("127.0.0.1", 8081))
haptic_sock.settimeout(0.001)

controller = fri.KukaController(
    fri.ControlMode.JOINT_POSITION,
    "robots/iiwa.urdf",
    False,
)

controller.start()
last_sequence = None

pos = np.array([0.65, 0.0,  0.35], dtype=np.float64)
joint_target_deg = np.zeros(7, dtype=np.float64)

rot = np.array([
    [-1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, -1.0],
], dtype=np.float64)

while 1:
    obs = controller.get_observation()
    print(obs)

    try:
        data, addr = haptic_sock.recvfrom(1024)
        packet = json.loads(data.decode("utf-8"))

        if isinstance(packet, dict):
            sequence = packet.get("sequence")
            payload = packet.get("data")
            if (
                not isinstance(sequence, int)
                or isinstance(sequence, bool)
                or sequence < 0
                or not isinstance(payload, list)
            ):
                print(f"Ignoring malformed UDP envelope from {addr}: {packet!r}")
                continue
            if last_sequence is not None and sequence <= last_sequence:
                continue
        elif isinstance(packet, list):
            # Legacy packets cannot be checked for duplication or reordering.
            sequence = None
            payload = packet
        else:
            print(f"Ignoring unsupported UDP packet from {addr}: {packet!r}")
            continue

        message = np.asarray(payload, dtype=np.float64)
        if message.size >= 7:
            joint_target_deg[:] = message[:7]
            controller.set_target_joints_degrees(joint_target_deg)
            if sequence is not None:
                last_sequence = sequence

    except socket.timeout:
        data, addr = None, None  # или просто continue
    except (json.JSONDecodeError, UnicodeDecodeError, TypeError, ValueError) as error:
        print(f"Ignoring invalid UDP packet: {error}")


controller.stop()

# 6.67716140e-01 6.13374614e-05  3.09092617e-01 
# -9.39753313e-01 -5.30466826e-05 3.41853342e-01 
# -2.85829922e-05  9.99999997e-01  7.65992829e-05
# -3.41853344e-01  6.22132385e-05 -9.39753312e-01  
