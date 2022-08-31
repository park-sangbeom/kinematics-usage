import pyrealsense2 as rs
import numpy as np
import cv2
import time 


img_shape = (128, 96)
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline_profile = pipe.start(cfg)
device = pipeline_profile.get_device()
depth_sensor = device.query_sensors()[0]
emitter = depth_sensor.get_option(rs.option.emitter_enabled)
print("emitter = ", emitter)
set_emitter = 1
depth_sensor.set_option(rs.option.emitter_enabled, set_emitter)
emitter1 = depth_sensor.get_option(rs.option.emitter_enabled)
print("new emitter = ", emitter1)
# Declare filters
dec_filter = rs.decimation_filter()   # Decimation - reduces depth frame density
spat_filter = rs.spatial_filter()          # Spatial    - edge-preserving spatial smoothing
temp_filter = rs.temporal_filter()    # Temporal   - reduces temporal noise
hole_filter = rs.hole_filling_filter()
threshold_filter = rs.threshold_filter()

frames = pipe.wait_for_frames()
depth_frame = frames.get_depth_frame()
filtered = dec_filter.process(depth_frame)
filtered = spat_filter.process(filtered)
filtered = temp_filter.process(filtered)
filtered = hole_filter.process(filtered)
# filtered = threshold_filter.process(filtered)

data = np.asanyarray(filtered.get_data())
# resize = cv2.resize(data, , interpolation=cv2.INTER_CUBIC)
# resize = cv2.resize(data, )
# data = cv2.resize(data, img_shape, interpolation=cv2.INTER_CUBIC).astype('float32')
# print(np.min(data), np.max(data))
# cv_image_norm = cv2.normalize(data, data, 0, 1, cv2.NORM_MINMAX)
# cv2.imshow('Image', cv_image_norm)
# k = cv2.waitKey(0)
# if k == 27:
#     cv2.destroyAllWindows()

# print(np.average(data))
np.savez('0831_test1', pointcloud = data)