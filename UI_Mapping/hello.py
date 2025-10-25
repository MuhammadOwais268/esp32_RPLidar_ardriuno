"""
2D LiDAR UDP Data Receiver and Real-Time Visualizer
Receives binary UDP packets from ESP32 LiDAR and displays them in real-time
"""

import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import threading
import time

# Configuration
UDP_PORT = 4210
BUFFER_SIZE = 1024
MAX_DISTANCE = 8000  # Maximum distance in mm to display (8 meters)
HISTORY_SIZE = 360   # Keep last N points for smoother visualization

class LidarReceiver:
    def __init__(self, port=UDP_PORT):
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("", self.port))
        self.sock.settimeout(1.0)
        
        # Store points: angle -> (distance, quality, timestamp)
        self.points = {}
        self.lock = threading.Lock()
        self.running = False
        self.packet_count = 0
        self.error_count = 0
        
        print(f"LiDAR Receiver initialized on port {self.port}")
        
    def decode_packet(self, data):
        """Decode binary LiDAR packet"""
        if len(data) < 4:
            return []
        
        # Check magic bytes
        if data[0] != 0xAA or data[1] != 0x55:
            self.error_count += 1
            return []
        
        point_count = data[2]
        expected_size = 3 + (point_count * 5) + 1  # header + points + checksum
        
        if len(data) < expected_size:
            self.error_count += 1
            return []
        
        # Verify checksum
        received_checksum = data[expected_size - 1]
        calculated_checksum = 0
        for i in range(expected_size - 1):
            calculated_checksum ^= data[i]
        
        if received_checksum != calculated_checksum:
            self.error_count += 1
            print(f"Checksum error: expected {calculated_checksum}, got {received_checksum}")
            return []
        
        # Decode points
        points = []
        for i in range(point_count):
            offset = 3 + (i * 5)
            angle_x100, dist_mm, quality = struct.unpack('<HHB', data[offset:offset+5])
            angle = angle_x100 / 100.0
            points.append((angle, dist_mm, quality))
        
        return points
    
    def receive_loop(self):
        """Background thread to receive UDP packets"""
        print("Starting receiver thread...")
        self.running = True
        
        while self.running:
            try:
                data, addr = self.sock.recvfrom(BUFFER_SIZE)
                points = self.decode_packet(data)
                
                if points:
                    self.packet_count += 1
                    timestamp = time.time()
                    
                    with self.lock:
                        for angle, distance, quality in points:
                            # Filter out invalid readings
                            if distance > 0 and distance < MAX_DISTANCE:
                                self.points[angle] = (distance, quality, timestamp)
                    
                    if self.packet_count % 100 == 0:
                        print(f"Packets received: {self.packet_count}, "
                              f"Errors: {self.error_count}, "
                              f"Active points: {len(self.points)}")
                        
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error in receive loop: {e}")
                
        print("Receiver thread stopped")
    
    def get_points(self):
        """Get current points as numpy arrays for plotting"""
        with self.lock:
            if not self.points:
                return np.array([]), np.array([]), np.array([])
            
            angles = []
            distances = []
            qualities = []
            
            current_time = time.time()
            # Remove old points (older than 1 second)
            stale_angles = [a for a, (d, q, t) in self.points.items() 
                           if current_time - t > 1.0]
            for a in stale_angles:
                del self.points[a]
            
            for angle, (distance, quality, timestamp) in self.points.items():
                angles.append(angle)
                distances.append(distance)
                qualities.append(quality)
            
            return np.array(angles), np.array(distances), np.array(qualities)
    
    def start(self):
        """Start receiving data in background thread"""
        self.thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Stop receiving data"""
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=2.0)
        self.sock.close()


class LidarVisualizer:
    def __init__(self, receiver):
        self.receiver = receiver
        
        # Create figure with polar plot
        self.fig = plt.figure(figsize=(12, 10))
        
        # Main polar plot
        self.ax_polar = plt.subplot(221, projection='polar')
        self.ax_polar.set_title('LiDAR 2D Scan (Polar View)', pad=20)
        self.ax_polar.set_ylim(0, MAX_DISTANCE)
        self.ax_polar.set_theta_zero_location('N')
        self.ax_polar.set_theta_direction(-1)
        self.scatter_polar = self.ax_polar.scatter([], [], c=[], s=20, cmap='jet', 
                                                   vmin=0, vmax=255, alpha=0.6)
        
        # Cartesian plot
        self.ax_cart = plt.subplot(222)
        self.ax_cart.set_title('LiDAR 2D Scan (Top-Down View)', pad=20)
        self.ax_cart.set_xlabel('X (mm)')
        self.ax_cart.set_ylabel('Y (mm)')
        self.ax_cart.set_xlim(-MAX_DISTANCE, MAX_DISTANCE)
        self.ax_cart.set_ylim(-MAX_DISTANCE, MAX_DISTANCE)
        self.ax_cart.grid(True, alpha=0.3)
        self.ax_cart.set_aspect('equal')
        self.scatter_cart = self.ax_cart.scatter([], [], c=[], s=20, cmap='jet',
                                                 vmin=0, vmax=255, alpha=0.6)
        
        # Distance histogram
        self.ax_hist = plt.subplot(223)
        self.ax_hist.set_title('Distance Distribution')
        self.ax_hist.set_xlabel('Distance (mm)')
        self.ax_hist.set_ylabel('Count')
        self.ax_hist.set_xlim(0, MAX_DISTANCE)
        
        # Quality histogram
        self.ax_quality = plt.subplot(224)
        self.ax_quality.set_title('Quality Distribution')
        self.ax_quality.set_xlabel('Quality')
        self.ax_quality.set_ylabel('Count')
        self.ax_quality.set_xlim(0, 255)
        
        # Add colorbar
        cbar = plt.colorbar(self.scatter_polar, ax=self.ax_polar)
        cbar.set_label('Signal Quality')
        
        # Stats text
        self.stats_text = self.fig.text(0.02, 0.98, '', va='top', fontsize=10,
                                        family='monospace',
                                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        plt.tight_layout()
    
    def update(self, frame):
        """Update plot with new data"""
        angles, distances, qualities = self.receiver.get_points()
        
        if len(angles) == 0:
            return self.scatter_polar, self.scatter_cart
        
        # Convert to radians for polar plot
        angles_rad = np.deg2rad(angles)
        
        # Update polar plot
        self.scatter_polar.set_offsets(np.c_[angles_rad, distances])
        self.scatter_polar.set_array(qualities)
        
        # Convert to Cartesian coordinates
        x = distances * np.sin(angles_rad)
        y = distances * np.cos(angles_rad)
        
        # Update Cartesian plot
        self.scatter_cart.set_offsets(np.c_[x, y])
        self.scatter_cart.set_array(qualities)
        
        # Update distance histogram
        self.ax_hist.clear()
        self.ax_hist.hist(distances, bins=50, color='blue', alpha=0.7, edgecolor='black')
        self.ax_hist.set_title('Distance Distribution')
        self.ax_hist.set_xlabel('Distance (mm)')
        self.ax_hist.set_ylabel('Count')
        self.ax_hist.set_xlim(0, MAX_DISTANCE)
        self.ax_hist.grid(True, alpha=0.3)
        
        # Update quality histogram
        self.ax_quality.clear()
        self.ax_quality.hist(qualities, bins=50, color='green', alpha=0.7, edgecolor='black')
        self.ax_quality.set_title('Quality Distribution')
        self.ax_quality.set_xlabel('Quality')
        self.ax_quality.set_ylabel('Count')
        self.ax_quality.set_xlim(0, 255)
        self.ax_quality.grid(True, alpha=0.3)
        
        # Update stats
        stats = f"Points: {len(angles)}\n"
        stats += f"Packets: {self.receiver.packet_count}\n"
        stats += f"Errors: {self.receiver.error_count}\n"
        stats += f"Dist: {distances.min():.0f}-{distances.max():.0f}mm\n"
        stats += f"Avg Dist: {distances.mean():.0f}mm\n"
        stats += f"Avg Quality: {qualities.mean():.1f}"
        self.stats_text.set_text(stats)
        
        return self.scatter_polar, self.scatter_cart
    
    def show(self):
        """Start animation and show plot"""
        ani = FuncAnimation(self.fig, self.update, interval=50, blit=False, cache_frame_data=False)
        plt.show()


def main():
    print("=" * 60)
    print("2D LiDAR UDP Receiver and Visualizer")
    print("=" * 60)
    print(f"Listening on UDP port {UDP_PORT}")
    print(f"Maximum display distance: {MAX_DISTANCE}mm ({MAX_DISTANCE/1000}m)")
    print("\nWaiting for LiDAR data...")
    print("Press Ctrl+C to exit\n")
    
    # Create receiver and start receiving
    receiver = LidarReceiver(UDP_PORT)
    receiver.start()
    
    # Wait a moment for first data
    time.sleep(1)
    
    try:
        # Create and show visualizer
        visualizer = LidarVisualizer(receiver)
        visualizer.show()
    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        receiver.stop()
        print("Receiver stopped")
        print(f"Total packets received: {receiver.packet_count}")
        print(f"Total errors: {receiver.error_count}")


if __name__ == "__main__":
    main()