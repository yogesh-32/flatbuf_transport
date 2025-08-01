import socket, struct
from NavData import NavData

from visualizer import LiveNavVisualizer

def recv_exact(conn, n):
    buf = b''
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

def handle_connection(conn, visual):
    print("[RECV] Connected.")
    while True:
        try:
            header = recv_exact(conn, 12)
            if not header:
                break
            msg_id, type_id, length = struct.unpack('<III', header)
            payload = recv_exact(conn, length)
            if not payload:
                break

            if type_id != 1:
                continue

            nav = NavData.GetRootAsNavData(payload, 0)
            pose = nav.Pose()
            x, y, yaw = pose.X(), pose.Y(), pose.Yaw()
            obstacles = [(nav.Obstacles(i).X(), nav.Obstacles(i).Y()) for i in range(nav.ObstaclesLength())]
            path = [(nav.Path(i).X(), nav.Path(i).Y()) for i in range(nav.PathLength())]

            print(f"[RECV] ID={msg_id} | Pose: ({x:.2f}, {y:.2f}, {yaw:.2f}) | Obstacles={len(obstacles)} | Path={len(path)}")

            visual.update_data((x, y, yaw), obstacles, path)

        except Exception as e:
            print(f"[ERROR] {e}")
            break
    conn.close()


def start_server():
    visual = LiveNavVisualizer(follow_drone=True)
    sock = socket.socket()
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("0.0.0.0", 9001))
    sock.listen(1)
    print("[RECV] Listening...")
    conn, _ = sock.accept()

    import threading
    threading.Thread(target=handle_connection, args=(conn, visual), daemon=True).start()
    visual.show()

if __name__ == "__main__":
    start_server()
