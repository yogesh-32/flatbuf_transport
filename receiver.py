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
            length_bytes = recv_exact(conn, 4)
            if not length_bytes:
                break
            msg_len = struct.unpack('<I', length_bytes)[0]
            data = recv_exact(conn, msg_len)
            nav = NavData.GetRootAsNavData(data, 0)
            print(f"[DEBUG] Length bytes: {length_bytes} → {msg_len}")
            pose = nav.Pose()
            print(f"Pose: x={pose.X():.2f}, y={pose.Y():.2f}, yaw={pose.Yaw():.2f}")
            print(f"Obstacles: {nav.ObstaclesLength()}, Path: {nav.PathLength()}")

        except Exception as e:
            print(f"[ERROR] {e}")
            break
    conn.close()

def start_server():

    sock = socket.socket()
    sock.bind(('127.0.0.1', 9001))
    sock.listen(1)
    print("[RECV] Listening...")
    conn, _ = sock.accept()
    handle_connection(conn)

if __name__ == "__main__":
    start_server()
