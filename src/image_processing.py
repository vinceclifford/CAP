import pyrealsense2 as rs
import numpy as np
import cv2

pipe = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

pipe.start(config)

while True:
    frame = pipe.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    cv2.imshow('Color Image', color_image)
    cv2.imshow('Depth Image', depth_image)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pipe.stop()