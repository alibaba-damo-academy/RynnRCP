import os
import sys
import gzip
import io
import cv2
import yaml
import argparse
import threading
import time
import logging
import numpy as np
from datetime import datetime
from lcm import LCM

# Add project root to Python path to access common.lcm.lcmMotion
lcm_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../common/lcm'))
sys.path.insert(0, lcm_root)

from lcmSensor.image_data import image_data
from lcmSensor.camera_desc import camera_desc
from lcmSensor.camera_list_desc import camera_list_desc
from lcmSensor.req_camera_image import req_camera_image
from lcmSensor.camera_image_response import camera_image_response

def parse_arguments():
    """Parse command line arguments for camera configuration and logging."""
    parser = argparse.ArgumentParser(description='Camera Node (LCM)')
    parser.add_argument('--camera_config', type=str, required=True,
                        help='Path to camera configuration file')
    parser.add_argument('--log_config', type=str, required=True,
                        help='Path to logging configuration file')
    return parser.parse_args()

class CameraCapture:
    def __init__(self, camera_config):
        """Initialize camera capture settings and start capturing frames."""
        self.name = camera_config['name']
        self.device = camera_config['device']
        self.width = camera_config['width']
        self.height = camera_config['height']
        self.framerate = camera_config['framerate']
        self.format = camera_config['format']
        self.latest_frame = None
        self.queue_lock = threading.Lock()  # Lock for thread-safe access to the latest frame
        
        self.running = False  # Track if the camera is running
        self.cap = cv2.VideoCapture(self.device)  # Open the camera device
        if not self.cap.isOpened():
            logging.error(f"Cannot open camera: {self.device}")
            os._exit(-1)

        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FPS, self.framerate)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        fourcc = cv2.VideoWriter_fourcc(*self.format)
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)

        self.running = True
        # Start a thread for capturing frames
        self.capture_thread = threading.Thread(target=self.capture_frames)
        self.capture_thread.start()

    def capture_frames(self):
        """Continuously capture frames from the camera in a separate thread."""
        while self.running:
            ret, frame = self.cap.read()  # Read a frame from the camera
            if ret:
                with self.queue_lock:  # Ensure thread-safe access to the frame
                    # Convert the frame from BGR to RGB
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                    # Compress the frame using gzip
                    data_buffer = io.BytesIO()
                    np.save(data_buffer, frame_rgb)  # Save the frame to the buffer
                    data_buffer.seek(0)
                    compressed_data = gzip.compress(data_buffer.getvalue(), compresslevel=1)
                    self.latest_frame = compressed_data  # Store the compressed frame in latest_frame
            else:
                logging.error(f"Cannot read camera: {self.name}. Stopping capture.")
                self.running = False  # Stop capturing if unable to read the frame

    def get_latest_frame(self):
        """Return the latest captured frame."""
        with self.queue_lock:
            return self.latest_frame

    def stop(self):
        """Stop capturing frames and release the camera."""
        self.running = False
        self.capture_thread.join()  # Wait for the capture thread to finish
        self.cap.release()  # Release the camera device

    def get_status(self):
        """Return the camera's operational status."""
        return 0 if self.running else 1  # 0 = OK, 1 = Error

    def __exit__(self):
        """Clean up when the context manager exits."""
        self.stop()

def handle_request(channel, data):
    """Handle incoming requests for camera images and respond with the latest image."""
    msg = req_camera_image.decode(data)  # Decode the incoming request message

    response = camera_image_response()
    response.camera_count = msg.camera_count
    response.seq = msg.seq
    response.images = []

    png_filenames = []  # List to store filenames of saved images

    for i in range(msg.camera_count):
        camera_name = str(msg.camera_names[i])  # Get the name of the requested camera
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        logging.info(f"[{timestamp}]: Received request for {camera_name}, starting to capture image....")
        camera_capture = camera_map.get(camera_name)  # Access the camera capture object by name
        if camera_capture:
            latest_frame = camera_capture.get_latest_frame()  # Get the latest image
            if latest_frame:
                logging.info(f"[{timestamp}]:   Image shape: [{camera_capture.width},{camera_capture.height},3] size: {len(latest_frame)/1024} KB")
                image_info = image_data()  # Create image data response
                image_info.camera_id = msg.camera_ids[i]
                image_info.camera_name = msg.camera_names[i]
                image_info.width = camera_capture.width
                image_info.height = camera_capture.height
                image_info.channels = 3
                image_info.image_size = len(latest_frame)
                image_info.image_bytes = latest_frame  # Attach the compressed image bytes

                # Store the filename for saving the image later
                png_filename = os.path.join(log_dir, f"{camera_name}_{timestamp}.png")
                png_filenames.append((png_filename, latest_frame))  # Append the tuple of filename and latest_frame
                response.images.append(image_info)  # Add image info to the response
            else:
                logging.error(f"No image captured: {camera_name}")
                os._exit(-1)
        else:
            logging.error(f"Camera not found: {camera_name}, check config file")
            os._exit(-1)

    lc.publish("image_response", response.encode())  # Publish the response message
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    logging.info(f"Published image_response at {timestamp}")

    # Save the frames as PNGs after publishing the response
    for png_filename, frame in png_filenames:
        decompressed_data = gzip.decompress(frame)
        image_data_array = np.load(io.BytesIO(decompressed_data))
        cv2.imwrite(png_filename, cv2.cvtColor(image_data_array, cv2.COLOR_RGB2BGR)) 
        logging.info(f"Saved published image to: {png_filename}")  # Log the saved filename

def lcm_message_handler():
    """Continuously handle incoming LCM messages."""
    while True:
        lc.handle()

def publish_camera_list(cameras):
    """Periodically publish the list of available cameras."""
    global seq_cnt
    while True:
        msg = camera_list_desc()  # Create a message describing the list of cameras
        msg.seq = seq_cnt
        msg.stamp = int(time.time() * 1000000)
        msg.n = len(cameras)
        msg.cameras = []

        for camera in cameras:
            camera_item = camera_desc()
            camera_item.name = camera["name"]
            camera_item.id = int(camera["device"].replace("/dev/video", ""))
            camera_item.product = camera["brand"]
            camera_item.format = camera["format"]
            camera_item.width = camera["width"]
            camera_item.height = camera["height"]
            camera_item.fps = camera["framerate"]
            camera_item.status = camera_map[camera["name"]].get_status()  # Get actual camera status
            msg.cameras.append(camera_item)  # Add camera item to the message list

        lc.publish("camera_desc", msg.encode())  # Publish the camera description message
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S.%f")
        logging.info(f"[{timestamp}]: Published camera_desc at {seq_cnt}")
        seq_cnt += 1
        time.sleep(3)  # Wait for 3 seconds before publishing again

def setup_logging(log_dir, log_file, log_console_level, log_file_level):
    """Setup the logging configuration for file and console outputs."""
    if not os.path.exists(log_dir):
        os.makedirs(log_dir, exist_ok=True)  # Create log directory if it doesn't exist

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S.%f")
    log_file_with_timestamp = log_file + "_" + timestamp + ".log"
    log_path = os.path.join(log_dir, log_file_with_timestamp)
    
    level_mapping = {
        0: logging.INFO,    # 0: INFO
        1: logging.WARNING, # 1: WARNING
        2: logging.ERROR,   # 2: ERROR
        }

    # Clear previous handlers
    for handler in logging.root.handlers[:]:
        logging.root.removeHandler(handler)

    # Create and configure file handler for debug info
    file_handler = logging.FileHandler(log_path)
    file_handler.setLevel(level_mapping.get(log_file_level, logging.INFO))  # File handler outputs all levels of logs

    # Create and configure console handler for warnings and errors
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level_mapping.get(log_console_level, logging.WARNING))  # Console outputs warnings and above
    
    # Set log format
    formatter = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)

    # Add handlers to the root logger
    logging.getLogger().setLevel(logging.INFO)  # Set root logger level to INFO to allow all logs
    logging.getLogger().addHandler(file_handler)
    logging.getLogger().addHandler(console_handler)

    logging.warning(f"Camera log file: {log_path}")

def load_config(config_path):
    """Load configuration settings from the given YAML file."""
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)  # Safely load the YAML config
    return config

def main():
    """Main entry point for the Camera Node application."""
    args = parse_arguments()  # Parse command line arguments
    global log_dir 

    log_config = load_config(args.log_config)  # Load logging configuration
    log_file = log_config["camera_node_log_name"]
    log_dir = os.path.join(os.path.expanduser(log_config["log_dir"]), log_file)
    log_console_level = log_config["stderr_threshold"]
    log_file_level = log_config["min_log_level"]

    setup_logging(log_dir, log_file, log_console_level, log_file_level)  # Setup logging

    cameras_config = load_config(args.camera_config)  # Load camera configuration
    cameras = cameras_config['cameras'] 
    camera_captures = []
    global camera_map
    camera_map = {}

    for camera in cameras:
        camera_capture = CameraCapture(camera)  # Initialize each camera capture
        camera_captures.append(camera_capture)
        camera_map[camera['name']] = camera_capture  # Map camera name to its capture object

    logging.info("Starting to capture images... (Press Ctrl+C to stop)")

    global lc
    lc = LCM()  # Initialize LCM (Lightweight Communications and Marshalling)
    subscription = lc.subscribe("image_request", handle_request)  # Subscribe to image requests
    lcm_thread = threading.Thread(target=lcm_message_handler, daemon=True)  # Thread to handle LCM messages
    lcm_thread.start()

    global seq_cnt
    seq_cnt = 0

    camera_list_thread = threading.Thread(
        target=publish_camera_list, args=(cameras,), daemon=True)  # Thread to publish camera list
    camera_list_thread.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        logging.info("Stopping camera node...")
    except Exception as e:
        logging.error(f"Unexpected error: {e}")
    finally:
        lc.unsubscribe(subscription)
        os._exit(-1)

if __name__ == "__main__":
    main()
