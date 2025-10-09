from hal.products.mats import SDCSRoadMap
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os, cv2
# 

roadmap = SDCSRoadMap(leftHandTraffic=False)
    # print(dir(roadmap))

img = mpimg.imread("D:\\Quanser_PJD\\Resource\\src\\libraries\\resources\\images\\sdcs_cityscape.png")

H, W = img.shape[:2]

# Compute pixel corner coordinates
x = np.array([0, W])
y = -np.array([0, H])

print(x)
print(-y)

scale = np.array([-0.002035, 0.002035])  # meters per pixel
offset = np.array([1134,-2363])          # pixel coords of world origin
rotation_deg = 180

# Build rotation matrix
theta = np.deg2rad(rotation_deg)
R = np.array([
    [np.cos(theta), -np.sin(theta)],
    [np.sin(theta),  np.cos(theta)]
])

# Convert image corners to world coordinates
def pixel_to_world(px, py):
    dp = np.array([px, py])
    # pixel - offset
    dp = dp - offset[:, None]
    # rotate
    dp = R @ dp
    # scale
    dp = dp * scale[:, None]
    return dp

# Get image corners in world coordinates
x_corners, y_corners = pixel_to_world(x, y)
y_corners += 1680*0.002035
# Set extent = [xmin, xmax, ymin, ymax] in world units
extent = [x_corners[0], x_corners[1], y_corners[0], y_corners[1]]

plt.imshow(img, extent=extent, origin='upper')  # set image position in world space
scale = np.array([-0.002035, 0.002035])  # scale_x, scale_y
offset = np.array([1125, 2365])          # offset_x, offset_y
rotation_deg = 0

    # Plot all nodes
for i in range(len(roadmap.nodes)):   # Or len(roadmap.nodes)
    pose = roadmap.get_node_pose(i).squeeze()
    plt.plot(pose[0], pose[1], 'bo')  # blue dot
    plt.text(pose[0], pose[1], str(i), fontsize=9, c = "y")

    # Optionally: draw connections (if you know them or can access .edges or .adjacency)
    # Example: if roadmap.edges = [(0,1), (1,2)]
    # for a, b in roadmap.edges:
    #     pose_a = roadmap.get_node_pose(a).squeeze()
    #     pose_b = roadmap.get_node_pose(b).squeeze()
    #     plt.plot([pose_a[0], pose_b[0]], [pose_a[1], pose_b[1]], 'k--')

plt.axis('equal')
plt.title('SDCS Road Map Nodes')
plt.xlabel('X')
plt.ylabel('Y')
plt.xlim([-2.5,3])
plt.ylim([-1.5,5])
plt.show()