from picamera2 import Picamera2
import cv2, socket, base64, time
import numpy as np

BUFF_SIZE = 65536
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)

host_ip = '192.168.4.111'  # Replace with your wifi's IP
port = 8888
server_socket.bind((host_ip, port))
print('Server listening at:', (host_ip, port))

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640,480)}))  # Smaller size for stability
picam2.start()

fps, st, frames_to_count, cnt = (0, 0, 20, 0)

try:
    print("Waiting for a client...")
    while True:
        msg, client_addr = server_socket.recvfrom(BUFF_SIZE)
        print('Connection from:', client_addr)

        while True:
            frame = picam2.capture_array()
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 50])  # Lower quality for smaller size
            message = base64.b64encode(buffer)

            # Ensure packet size is within limit
            if len(message) > BUFF_SIZE:
                print("Frame size too large, adjust resolution or quality.")
                continue

            server_socket.sendto(message, client_addr)

            # FPS Counter
            if cnt == frames_to_count:
                try:
                    fps = round(frames_to_count / (time.time() - st))
                    st = time.time()
                    cnt = 0
                except ZeroDivisionError:
                    pass
            cnt += 1

except KeyboardInterrupt:
    print("\nServer stopped by user.")

finally:
    picam2.stop()
    server_socket.close()
    print("Resources released.")


