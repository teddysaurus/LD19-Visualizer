# LD19-Visualizer
Quick program to visualize the LD19 Lidar point cloud. 

I created this to visualize the point cloud from my LD19 Lidar on my Mac.
As far as I know, there isn't a native tool to do this.

The UART path is hard coded to `/dev/tty.usbserial-0001` which is correct for macOS, but can be changed for other *nix like operating systems.

Usage: `python ld19_visualizer.py`
