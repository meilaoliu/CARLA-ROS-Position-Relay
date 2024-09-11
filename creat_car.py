import pyproj

# 地图原点 (北京的某个点)
origin_lat = 40.1510363262265
origin_lon = 116.260639075136
origin_alt = 33.146

# GNSS 数据 (示例数据)
gnss_lat = 40.15294524
gnss_lon = 116.26303372
gnss_alt = 33.146  # 示例海拔高度

# 使用 pyproj 进行转换
transformer = pyproj.Transformer.from_crs("epsg:4326", "epsg:32650", always_xy=True)

# 将原点和 GNSS 坐标都转换为 UTM 坐标
origin_x, origin_y = transformer.transform(origin_lon, origin_lat)
gnss_x, gnss_y = transformer.transform(gnss_lon, gnss_lat)

# 计算相对原点的位移
relative_x = gnss_x - origin_x
relative_y = gnss_y - origin_y
relative_z = gnss_alt - origin_alt

print(f"相对位置: X: {relative_x}, Y: {relative_y}, Z: {relative_z}")
