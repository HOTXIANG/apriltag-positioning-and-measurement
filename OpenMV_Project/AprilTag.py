import sensor
import image
import time
import math
import lcd

# 镜头的物理焦距 (mm)
LENS_FOCAL_LENGTH_MM = 3.6

# 传感器的物理尺寸 (mm)
SENSOR_WIDTH_MM = 5.37
SENSOR_HEIGHT_MM = 4.04

# 分辨率设置
RESOLUTION_MODE = "QVGA" # 默认为QVGA

if RESOLUTION_MODE == "VGA":
    IMAGE_WIDTH = 640
    IMAGE_HEIGHT = 480
    sensor_frame_size = sensor.VGA
elif RESOLUTION_MODE == "QVGA":
    IMAGE_WIDTH = 320
    IMAGE_HEIGHT = 240
    sensor_frame_size = sensor.QVGA
elif RESOLUTION_MODE == "QQVGA":
    IMAGE_WIDTH = 160
    IMAGE_HEIGHT = 120
    sensor_frame_size = sensor.QQVGA


# 计算相机焦距参数
# f_x 是x的像素为单位的焦距
f_x = (LENS_FOCAL_LENGTH_MM / SENSOR_WIDTH_MM) * IMAGE_WIDTH
# f_y 是y的像素为单位的焦距
f_y = (LENS_FOCAL_LENGTH_MM / SENSOR_HEIGHT_MM) * IMAGE_HEIGHT
# c_x 是图像的x中心位置
c_x = IMAGE_WIDTH / 2
# c_y 是图像的y中心位置
c_y = IMAGE_HEIGHT / 2

# 物品信息库
TAG_DATABASE = {
    # 世界坐标系原点Tag
    3: {
        "name": "Origin",
        "size_mm": 20.0
    },
    # 物品Tag 1
    0: {
        "name": "Pink",
        "size_mm": 10.0,
        "color_threshold": [(30, 77, -1, 57, 11, 59)]
    },
    # 物品Tag 2
    1: {
        "name": "Green",
        "size_mm": 20.0,
        "color_threshold": [(10, 32, -22, -5, -5, 19)]
    },
}

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor_frame_size)
sensor.skip_frames(time = 2000)
sensor.set_auto_gain(True)
sensor.set_auto_whitebal(True)
sensor.set_hmirror(True)
sensor.set_vflip(True)
clock = time.clock()

lcd.init()
lcd.clear()
lcd.set_backlight(100)

while(True):
    clock.tick()
    img = sensor.snapshot()
    all_tags = img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y, families=image.TAG36H11)

    origin_pose_mm = None
    item_data_list = []

    # 遍历所有Tag，分类并计算其在相机坐标系中的位置
    for tag in all_tags:
        tag_id = tag.id()
        if tag_id in TAG_DATABASE:
            tag_info = TAG_DATABASE[tag_id]
            tag_size = tag_info["size_mm"]
            pose_mm = {
                "x": tag.x_translation() * tag_size,
                "y": tag.y_translation() * tag_size,
                "z": tag.z_translation() * tag_size
            }

            if tag_id == 3: # 原点
                origin_pose_mm = pose_mm
                img.draw_cross(tag.cx(), tag.cy(), color=(0, 255, 0), size=10)
            else: # 物品
                item_data_list.append({"pose": pose_mm, "info": tag_info, "tag_obj": tag})

    # 对每个被识别的物品进行完整的计算和绘制
    y_offset = 5
    for item in item_data_list:
        item_pose = item["pose"]
        item_info = item["info"]
        item_tag = item["tag_obj"]
        item_name = item_info["name"]

        # 计算到相机的距离
        cam_distance_mm = math.sqrt(item_pose["x"]**2 + item_pose["y"]**2 + item_pose["z"]**2)

        # 计算物品物理尺寸
        item_width_mm, item_height_mm = 0, 0
        blobs = img.find_blobs(item_info["color_threshold"], pixels_threshold=100, area_threshold=100, merge=True)
        if blobs:
            for blob in blobs:
                blob_r = blob.rect()
                tag_cx = item_tag.cx()
                tag_cy = item_tag.cy()
                if (tag_cx >= blob_r[0] and tag_cx < (blob_r[0] + blob_r[2]) and
                    tag_cy >= blob_r[1] and tag_cy < (blob_r[1] + blob_r[3])):
                    img.draw_rectangle(blob.rect(), color=(0, 255, 0), thickness=2)
                    item_width_mm = (blob.w() * cam_distance_mm) / f_x / 2
                    item_height_mm = (blob.h() * cam_distance_mm) / f_y / 2
                    break

        # 计算相对于原点的位置
        if origin_pose_mm:
            relative_x = item_pose["x"] - origin_pose_mm["x"]
            relative_y = item_pose["y"] - origin_pose_mm["y"]
            relative_z = item_pose["z"] - origin_pose_mm["z"]
            pos_str = f"P:X{relative_x:.1f} Y{relative_y:.1f} Z{relative_z:.1f}"
        else:
            pos_str = "Origin not found"

        # 绘制所有信息到屏幕
        img.draw_string(5, y_offset, f"[{item_name}]", color=(255,255,0), scale=1)
        img.draw_string(10, y_offset + 20, f"Dist: {cam_distance_mm:.0f}mm", color=(255,0,255), scale=1)
        img.draw_string(10, y_offset + 40, f"Size: {item_width_mm:.0f}x{item_height_mm:.0f}mm", color=(255,0,255), scale=1)
        img.draw_string(10, y_offset + 60, f"Pos: {pos_str}", color=(255,0,255), scale=1)
        y_offset += 90

    lcd.display(img)

    print(f"FPS: {clock.fps():.2f}")
