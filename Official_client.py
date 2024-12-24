import cv2
import socket
import numpy as np
import base64
import threading
import os
import datetime

BUFF_SIZE = 65536

# Variables to store frames from both cameras
frame1 = None
frame2 = None
frame_lock = threading.Lock()  # Ensure thread safety

# Function to handle receiving frames from each camera
def receive_stream(camera_id, host_ip, port, frame_storage):
    global frame1, frame2

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)
    server_address = (host_ip, port)

    # Send initial signal to server
    client_socket.sendto(f"Hello from Camera {camera_id}".encode('utf-8'), server_address)
    print(f"Connecting to server for Camera {camera_id} at {server_address}")

    try:
        while True:
            packet, _ = client_socket.recvfrom(BUFF_SIZE)
            data = base64.b64decode(packet)
            npdata = np.frombuffer(data, dtype=np.uint8)
            frame = cv2.imdecode(npdata, cv2.IMREAD_COLOR)

            if frame is None:
                print(f"Error decoding frame from Camera {camera_id}")
                continue

            # Update the global frame variable
            with frame_lock:
                if frame_storage == "frame1":
                    frame1 = frame
                elif frame_storage == "frame2":
                    frame2 = frame

    except KeyboardInterrupt:
        print(f"\nCamera {camera_id} stream stopped by user.")

    finally:
        client_socket.close()
        print(f"Camera {camera_id} resources released.")

# Function to display the overlayed video streams
def display_overlay():
    global frame1, frame2

    while True:
        with frame_lock:
            if frame1 is not None and frame2 is not None:
                # Resize both frames to the same size
                height = min(frame1.shape[0], frame2.shape[0])
                width = min(frame1.shape[1], frame2.shape[1])
                center = (width // 2, height // 2)
                center_x, center_y = width // 2, height // 2

                # Rotate resized_frame1
                angle1 = 0  # Rotation angle
                scale1 = 1  # Scaling factor (1.0 means no scaling)
                rotation_matrix1 = cv2.getRotationMatrix2D(center, angle1, scale1)
                rotated_frame1 = cv2.warpAffine(frame1, rotation_matrix1, (width, height))

                # Flip thermai image horizontally ( 1 - horizontal; 0 - vertical; -1 - both)
                flipped_frame2 = cv2.flip(frame2, 1)
                
                # Rotate resized_frame2
                angle2 = 0  # Rotation angle
                scale2 = 1.2  # Scaling factor (1.0 means no scaling)
                rotation_matrix2 = cv2.getRotationMatrix2D(center, angle2, scale2)
                rotated_frame2 = cv2.warpAffine(flipped_frame2, rotation_matrix2, (width, height))

                # Get the RGB values at the center pixel
                b, g, r = rotated_frame2[center_y, center_x]
                rgb_text = f"R: {r} G: {g} B: {b}"

                # Draw a tiny cross at the center of the thermal frame
                color = (0, 0, 0)  # Green color for the cross
                thickness = 1  # Thickness of the cross lines
                length = 3  # Length of the cross arms
                cv2.line(rotated_frame2, (center_x - length, center_y), (center_x + length, center_y), color, thickness)
                cv2.line(rotated_frame2, (center_x, center_y - length), (center_x, center_y + length), color, thickness)

                # Display the RGB values on the frame
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.5
                text_color = (0, 255, 0)

                # Blend the frames using cv2.addWeighted
                alpha = 0
                gamma = 0     # Scalar added to the sum
                blended_frame = cv2.addWeighted(rotated_frame1, alpha, rotated_frame2, 1 - alpha, gamma)
                blended_frame = cv2.flip(blended_frame, -1)
                cv2.putText(blended_frame, rgb_text, (10, 30), font, font_scale, text_color, 1, cv2.LINE_AA)

                # Display the blended frame
                cv2.imshow("Overlayed Video Streams", blended_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Shutting down overlay display...")
            break

    cv2.destroyAllWindows()

# Function to capture and save the current blended frame
def capture(save_dir=None):
    global frame1, frame2
    with frame_lock:
        if frame1 is not None and frame2 is not None:
            height = min(frame1.shape[0], frame2.shape[0])
            width = min(frame1.shape[1], frame2.shape[1])
            center = (width // 2, height // 2)
            center_x, center_y = width // 2, height // 2

            # Rotate resized_frame1
            angle1 = 0  # Rotation angle
            scale1 = 1  # Scaling factor (1.0 means no scaling)
            rotation_matrix1 = cv2.getRotationMatrix2D(center, angle1, scale1)
            rotated_frame1 = cv2.warpAffine(frame1, rotation_matrix1, (width, height))

            # Flip thermai image horizontally ( 1 - horizontal; 0 - vertical; -1 - both)
            flipped_frame2 = cv2.flip(frame2, 1)

            # Rotate resized_frame2
            angle2 = 0  # Rotation angle
            scale2 = 1.2  # Scaling factor (1.0 means no scaling)
            rotation_matrix2 = cv2.getRotationMatrix2D(center, angle2, scale2)
            rotated_frame2 = cv2.warpAffine(flipped_frame2, rotation_matrix2, (width, height))

            # Blend the frames using cv2.addWeighted
            alpha = 0.3
            gamma = 0     # Scalar added to the sum
            blended_frame = cv2.addWeighted(rotated_frame1, alpha, rotated_frame2, 1 - alpha, gamma)
            blended_frame = cv2.flip(blended_frame, -1)

            rotated_frame2 = cv2.flip(rotated_frame2,-1)

            # Get the RGB values at the center pixel
            b, g, r = rotated_frame2[center_y, center_x]
            rgb_text = f"R: {r} G: {g} B: {b}"

            # Draw a tiny cross at the center of the thermal frame
            color = (0, 0, 0)  # Green color for the cross
            thickness = 1  # Thickness of the cross lines
            length = 3  # Length of the cross arms
            cv2.line(rotated_frame2, (center_x - length, center_y), (center_x + length, center_y), color, thickness)
            cv2.line(rotated_frame2, (center_x, center_y - length), (center_x, center_y + length), color, thickness)

            # Display the RGB values on the frame
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            text_color = (0, 255, 0)
            cv2.putText(rotated_frame2, rgb_text, (10, 30), font, font_scale, text_color, 1, cv2.LINE_AA)

            # Generate the filename with date and time
            timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
            filename = f"blended_{timestamp}.png"
            filename1 = f"thermal_{timestamp}.png"

            # Set save directory or use current directory
            if not save_dir:
                save_dir = os.getcwd()
            os.makedirs(save_dir, exist_ok=True)
            file_path = os.path.join(save_dir, filename)
            file_path1 = os.path.join(save_dir, filename1)

            # Save the frame
            cv2.imwrite(file_path, blended_frame)
            cv2.imwrite(file_path1, rotated_frame2)
            print(f"Blended frame captured and saved at: {file_path}")
            print(f"Thermal frame captured and saved at: {file_path1}")
            return file_path, file_path1
        else:
            print("No frames available to capture.")
            return None

# Main function
def start():
    host_ip = '192.168.4.111'  # Replace with server's IP
    camera1_port = 8888
    camera2_port = 9999

    # Start threads to receive streams from both cameras
    thread1 = threading.Thread(target=receive_stream, args=(1, host_ip, camera1_port, "frame1"))
    thread2 = threading.Thread(target=receive_stream, args=(2, host_ip, camera2_port, "frame2"))

    thread1.start()
    thread2.start()

    # Start the overlay display
    display_overlay()

    thread1.join()
    thread2.join()

if __name__ == "__main__":
    start()
