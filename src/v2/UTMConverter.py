import utm
import numpy as np

##### Setting #####
ref_GPS = [35.0695, 128.579]

GPSData = list(map(float, input().split(",")))
print(GPSData)
biasX, biasY, _, _ = utm.from_latlon(ref_GPS[0],ref_GPS[1])
WayPoint = np.reshape(GPSData, (-1, 2))
WP = []
for i, j in WayPoint:
    x, y, _, _ = utm.from_latlon(i, j)
    x -= biasX
    y -= biasY
    WP.append(x)
    WP.append(y)

print(WP)
        
# [35.06958, 128.578882, 35.069663, 128.57882, 35.069673, 128.578854, 35.069683, 128.578886]
