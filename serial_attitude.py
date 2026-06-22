import serial
import re
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

# ================= Configuration =================
SERIAL_PORT = 'COM14'     # Modify to actual port
BAUD_RATE = 115200
TIMEOUT = 0.05           
# =================================================

def parse_imu_data(line):
    """
    Parse IMU data line from serial port
    Format: IMU_APP: Yaw:   359.99 , P:    -6.71, R:     4.80
    """
    # Regex updated to match new format: Yaw: value , P: value, R: value
    pattern = r"Yaw:\s*([+-]?\d+(?:\.\d+)?)\s*,?\s*P:\s*([+-]?\d+(?:\.\d+)?)\s*,?\s*R:\s*([+-]?\d+(?:\.\d+)?)"
    match = re.search(pattern, line)
    
    if match:
        yaw = float(match.group(1))
        pitch = float(match.group(2))
        roll = float(match.group(3))
        return pitch, roll, yaw
    return None

def init_3d_figure():
    """
    Initialize 3D coordinate system display environment
    """
    plt.ion()
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Real-time IMU Attitude Monitoring (Relative Axes)")
    
    # Set display range
    ax.set_xlim([-1.5, 1.5])
    ax.set_ylim([-1.5, 1.5])
    ax.set_zlim([-1.5, 1.5])
    ax.set_xlabel('Initial X')
    ax.set_ylabel('Initial Y')
    ax.set_zlabel('Initial Z')
    
    # Draw fixed world coordinate system (light gray dashed lines) as reference
    ax.plot([0, 1.5], [0, 0], [0, 0], color='gray', linestyle='--', alpha=0.5)
    ax.plot([0, 0], [0, 1.5], [0, 0], color='gray', linestyle='--', alpha=0.5)
    ax.plot([0, 0], [0, 0], [0, 1.5], color='gray', linestyle='--', alpha=0.5)

    # Initialize the three axes of the body coordinate system (all starting at origin [0,0,0])
    # Red represents body X axis (Forward)
    line_x, = ax.plot([], [], [], color='red', linewidth=3, label='Body X (Forward)')
    # Green represents body Y axis (Right)
    line_y, = ax.plot([], [], [], color='green', linewidth=3, label='Body Y (Right)')
    # Blue represents body Z axis (Down/Up)
    line_z, = ax.plot([], [], [], color='blue', linewidth=3, label='Body Z')
    
    ax.legend(loc='upper right')
    
    # Text in the upper left corner to display real-time data
    text_info = ax.text2D(0.05, 0.95, "Waiting for data...", transform=ax.transAxes)
    
    return fig, ax, (line_x, line_y, line_z), text_info

def main():
    fig, ax, lines, text_info = init_3d_figure()
    line_x, line_y, line_z = lines

    try:
        # Prevent ESP32 from resetting into bootloader mode by disabling DTR/RTS
        ser = serial.Serial()
        ser.port = SERIAL_PORT
        ser.baudrate = BAUD_RATE
        ser.timeout = TIMEOUT
        ser.dtr = False
        ser.rts = False
        ser.open()
        print(f"Successfully opened serial port: {SERIAL_PORT}")
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        return

    initial_rotation = None

    try:
        while True:
            latest_valid_line = None
            # Clear buffer, only take the latest frame of data to avoid rendering delay accumulation
            while ser.in_waiting > 0:
                raw_line = ser.readline()
                try:
                    line_str = raw_line.decode('utf-8').strip()
                    if "IMU_APP:" in line_str:
                        latest_valid_line = line_str
                except UnicodeDecodeError:
                    pass

            if latest_valid_line:
                parsed_data = parse_imu_data(latest_valid_line)
                
                if parsed_data:
                    pitch, roll, yaw = parsed_data
                    
                    # Generate current rotation matrix from absolute euler angles
                    current_rotation = R.from_euler('ZYX', [yaw, pitch, roll], degrees=True)
                    
                    # Set the first received frame as the zero reference
                    if initial_rotation is None:
                        initial_rotation = current_rotation
                        print(f"Initial reference set -> Yaw: {yaw}, Pitch: {pitch}, Roll: {roll}")
                    
                    # Calculate relative rotation: Current Body -> World -> Initial Body
                    relative_rotation = initial_rotation.inv() * current_rotation
                    
                    # Get relative euler angles for display
                    rel_yaw, rel_pitch, rel_roll = relative_rotation.as_euler('ZYX', degrees=True)
                    text_info.set_text(f"Rel Yaw:   {rel_yaw:>7.2f}°\nRel Pitch: {rel_pitch:>7.2f}°\nRel Roll:  {rel_roll:>7.2f}°")
                    
                    rot_matrix = relative_rotation.as_matrix()
                    
                    # The column vectors of the rotation matrix are the direction vectors of the body's three axes in the initial coordinate system
                    x_axis = rot_matrix[:, 0]
                    y_axis = rot_matrix[:, 1]
                    z_axis = rot_matrix[:, 2]
                    
                    # Update endpoint coordinates of body X axis (red line)
                    line_x.set_data([0, x_axis[0]], [0, x_axis[1]])
                    line_x.set_3d_properties([0, x_axis[2]])
                    
                    # Update endpoint coordinates of body Y axis (green line)
                    line_y.set_data([0, y_axis[0]], [0, y_axis[1]])
                    line_y.set_3d_properties([0, y_axis[2]])
                    
                    # Update endpoint coordinates of body Z axis (blue line)
                    line_z.set_data([0, z_axis[0]], [0, z_axis[1]])
                    line_z.set_3d_properties([0, z_axis[2]])
            
            # Refresh image
            fig.canvas.draw()
            fig.canvas.flush_events()
            
            if not plt.fignum_exists(fig.number):
                print("Plot window closed, exiting program.")
                break

    except KeyboardInterrupt:
        print("\nManually stopped.")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("Serial port closed.")

if __name__ == "__main__":
    main()