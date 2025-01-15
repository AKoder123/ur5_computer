import pyrealsense2 as rs
import open3d as o3d
import numpy as np

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

try:
    while True:
        # Wait for frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue
        
        ############### Filters
        decimation = rs.decimation_filter()
        depth_frame = decimation.process(depth_frame)

        spatial = rs.spatial_filter()
        depth_frame = spatial.process(depth_frame)

        temporal = rs.temporal_filter()
        depth_frame = temporal.process(depth_frame)

        # hole_filling = rs.hole_filling_filter()
        # depth_frame = hole_filling.process(depth_frame)
        ##################3
        
        
        # Convert frames to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Create point cloud
        pc = rs.pointcloud()
        pc.map_to(color_frame)
        points = pc.calculate(depth_frame)

        # Convert RealSense point cloud to Open3D format
        verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
        colors = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
        colors = np.clip(colors, 0, 1)  # Ensure texture coordinates are valid

        # Create Open3D PointCloud object
        point_cloud = o3d.geometry.PointCloud()
        point_cloud.points = o3d.utility.Vector3dVector(verts)
        
        # Optional: Colorize the point cloud
        if color_frame:
            color = np.asanyarray(color_frame.get_data())
            color = color[:, :, ::-1]  # Convert BGR to RGB
            height, width, _ = color.shape

            # Map texture coordinates to pixel coordinates
            color_indices = (colors * [width - 1, height - 1]).astype(np.int32)
            valid_mask = (color_indices[:, 0] >= 0) & (color_indices[:, 0] < width) & \
                         (color_indices[:, 1] >= 0) & (color_indices[:, 1] < height)
            
            # Apply colors
            colors_rgb = np.zeros_like(verts)
            colors_rgb[valid_mask] = color[color_indices[valid_mask, 1], color_indices[valid_mask, 0], :]
            colors_rgb = colors_rgb / 255.0  # Normalize to [0, 1]
            point_cloud.colors = o3d.utility.Vector3dVector(colors_rgb)

        # Visualize point cloud
        o3d.visualization.draw_geometries([point_cloud])

finally:
    pipeline.stop()
