import pyrealsense2 as rs

# Initialize pipeline
pipeline = rs.pipeline()
config = rs.config()

# List connected devices
ctx = rs.context()
if len(ctx.devices) == 0:
    raise RuntimeError("No RealSense device connected.")

print("Devices connected:", [dev.get_info(rs.camera_info.name) for dev in ctx.devices])

# Configure streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start pipeline
pipeline.start(config)
