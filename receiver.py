import socket, struct
from NavData import NavData

def recv_exact(conn, n):
    buf = b''
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf

def handle_connection(conn):
    print("[RECV] Connected.")
    while True:
        try:
            header = recv_exact(conn, 12)
            if not header:
                break
            msg_id, type_id, length = struct.unpack('<III', header)

            payload = recv_exact(conn, length)
            if not payload or len(payload) != length:
                print("[WARN] Incomplete payload.")
                break

            if type_id != 1:
                print(f"[WARN] Unknown message type: {type_id}")
                continue

            nav = NavData.GetRootAsNavData(payload, 0)
            pose = nav.Pose()
            print(f"[RECV] ID={msg_id} | Pose: x={pose.X():.2f}, y={pose.Y():.2f}, yaw={pose.Yaw():.2f} | Obstacles: {nav.ObstaclesLength()}, Path: {nav.PathLength()}")

        except Exception as e:
            print(f"[ERROR] {e}")
            break
    conn.close()

def start_server():
    sock = socket.socket()
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("127.0.0.1", 9001))
    sock.listen(1)
    print("[RECV] Listening...")
    conn, _ = sock.accept()
    handle_connection(conn)

if __name__ == "__main__":
    start_server()
