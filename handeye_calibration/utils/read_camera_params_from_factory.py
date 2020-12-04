import pyrealsense2 as rs

FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
FRAME_RATE = 30

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and stream everything
cfg = rs.config()
cfg.enable_stream(rs.stream.color, FRAME_WIDTH, FRAME_HEIGHT, rs.format.rgb8, FRAME_RATE)
# Start streaming
pipe.start(cfg)

try:
    # Retreive the stream and intrinsic properties for both cameras
    profiles = pipe.get_active_profile()
    stream = profiles.get_stream(rs.stream.color).as_video_stream_profile()
    intrinsic = stream.get_intrinsics()
    print(intrinsic)
finally:
    pipe.stop()
