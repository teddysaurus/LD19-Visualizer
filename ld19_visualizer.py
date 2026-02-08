import serial
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque

class LD19Lidar:
    """Parser for LD19 lidar data packets"""
    
    HEADER = 0x54
    PACKET_SIZE = 47
    
    def __init__(self, port='/dev/tty.usbserial-0001', baudrate=230400):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        self.points = deque(maxlen=5000)  # Store recent points
        
    def parse_packet(self, data):
        """Parse a single LD19 data packet"""
        if len(data) != self.PACKET_SIZE or data[0] != self.HEADER:
            return None
        
        # Extract header info
        ver_len = data[1]
        speed = struct.unpack('<H', data[2:4])[0]  # RPM
        start_angle = struct.unpack('<H', data[4:6])[0] / 100.0  # degrees
        
        # Extract 12 measurement points
        points = []
        for i in range(12):
            offset = 6 + i * 3
            distance = struct.unpack('<H', data[offset:offset+2])[0]  # mm
            intensity = data[offset+2]
            
            if distance > 0:  # Valid measurement
                angle = (start_angle + i * 0.83) % 360  # Each point ~0.83° apart
                angle_rad = np.radians(angle)
                
                # Convert to Cartesian coordinates
                x = distance * np.cos(angle_rad)
                y = distance * np.sin(angle_rad)
                points.append((x, y, intensity))
        
        end_angle = struct.unpack('<H', data[42:44])[0] / 100.0
        timestamp = struct.unpack('<H', data[44:46])[0]
        crc = data[46]
        
        return points
    
    def read_packet(self):
        """Read and parse one packet from serial port"""
        # Look for header byte
        while True:
            byte = self.ser.read(1)
            if len(byte) == 0:
                return None
            if byte[0] == self.HEADER:
                # Read rest of packet
                rest = self.ser.read(self.PACKET_SIZE - 1)
                if len(rest) == self.PACKET_SIZE - 1:
                    packet = byte + rest
                    return self.parse_packet(packet)
    
    def update_points(self):
        """Read new data and update point cloud"""
        packet_points = self.read_packet()
        if packet_points:
            self.points.extend(packet_points)
    
    def close(self):
        self.ser.close()


class LidarVisualizer:
    """Real-time visualization of lidar point cloud"""
    
    def __init__(self, lidar):
        self.lidar = lidar
        
        # Set up the plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlim(-5000, 5000)
        self.ax.set_ylim(-5000, 5000)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_title('LD19 Lidar Point Cloud')
        self.ax.grid(True, alpha=0.3)
        
        # Create scatter plot
        self.scatter = self.ax.scatter([], [], c=[], cmap='hot', 
                                      s=1, vmin=0, vmax=255)
        
        # Add colorbar for intensity
        self.fig.colorbar(self.scatter, ax=self.ax, label='Intensity')
        
        # Add center marker
        self.ax.plot(0, 0, 'b+', markersize=10, markeredgewidth=2)
        
    def update(self, frame):
        """Animation update function"""
        # Read new data
        for _ in range(10):  # Read multiple packets per frame
            self.lidar.update_points()
        
        # Extract points for plotting
        if len(self.lidar.points) > 0:
            x = [p[0] for p in self.lidar.points]
            y = [p[1] for p in self.lidar.points]
            intensity = [p[2] for p in self.lidar.points]
            
            self.scatter.set_offsets(np.c_[x, y])
            self.scatter.set_array(np.array(intensity))
        
        return self.scatter,
    
    def run(self):
        """Start the visualization"""
        anim = FuncAnimation(self.fig, self.update, interval=5, 
                           blit=True, cache_frame_data=False)
        plt.show()


def main():
    print("Starting LD19 Lidar Visualizer...")
    print("Connecting to /dev/tty.usbserial-0001...")
    
    try:
        lidar = LD19Lidar('/dev/tty.usbserial-0001')
        print("Connected! Starting visualization...")
        
        visualizer = LidarVisualizer(lidar)
        visualizer.run()
        
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        print("Make sure the lidar is connected and the port is correct.")
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        if 'lidar' in locals():
            lidar.close()
        print("Closed.")


if __name__ == "__main__":
    main()
