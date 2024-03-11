# AnyTimeScan

This project allows you to use any mobile device to scan objects and generate STL files of the scanned objects. It leverages computer vision techniques and OpenCV for object detection and masking, providing a simple way to capture objects and convert them into 3D models.

## Features

- **Object Detection:** Utilizes OpenCV for object detection in real-time.
- **Mask Generation:** Creates masks for detected objects, highlighting them in the captured image.
- **STL Export:** Converts the masked objects into STL files suitable for 3D printing.

## Prerequisites

- Python 3.x
- OpenCV
- NumPy
- Open3D
- Glob

## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/yourusername/mobile-object-scanner.git
