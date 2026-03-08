#!/usr/bin/env python3
"""
3D 虛擬空間模擬器 (3D Space Simulator)

利用 matplotlib 的 3D 繪圖功能，將相機固定在原點，
並即時根據 AprilTag 的 3D Pose (平移與旋轉) 繪製出標籤在虛擬空間中的真實相對坐標。
"""

import cv2
import numpy as np
import os
import time
import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

import config
from apriltag_detector import AprilTagDetector

# 強制將畫面輸出到本機端螢幕 (解決 X11 顯示問題)
if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0"

# 初始化偵測器
try:
    if not os.path.exists(config.CAMERA_PARAMS_FILE):
        print("❌ 找不到 `camera_params.npz`，請先執行 `python3 calibrate_camera.py` 進行相機校正。")
        exit(1)
        
    detector = AprilTagDetector()
except Exception as e:
    print(f"初始化偵測器失敗: {e}")
    exit(1)

# 初始化 matplotlib 圖表
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')
plt.title("3D Space Simulator (Camera at Origin)")

# 固定座標軸範圍 (大約 1.5 公尺)
AXIS_LIM = 1.0
ax.set_xlim([-AXIS_LIM, AXIS_LIM])
ax.set_ylim([-AXIS_LIM, AXIS_LIM])
ax.set_zlim([0, AXIS_LIM * 2]) # Z 軸是深度，所以從 0 開始到較遠處

ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (Depth) (m)')

# 相機朝向是 Z 軸正方向
# 在 matplotlib 中，通常我們習慣 Z 軸朝上，但這裡我們保持相機座標系：
# X 軸：向右
# Y 軸：向下
# Z 軸：向前 (深度)
# 為了視覺上直覺，我們稍微反轉 Y 軸的顯示
ax.invert_yaxis()

def get_quiver_data(tvec, rvec, size=0.1):
    """根據旋轉矩陣和平移向量，計算 3D 座標軸 (Quiver) 的起點和方向"""
    # 將旋轉向量轉為旋轉矩陣 3x3
    if rvec.shape == (3, 3):
        R = rvec
    else:
        R, _ = cv2.Rodrigues(rvec)
        
    # 座標軸單位向量
    axes = np.eye(3) * size
    
    # 將單位向量依據 R 旋轉
    rotated_axes = R.dot(axes)
    
    # 提取起點 (x, y, z)
    x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]
    
    # 提取方向 (u, v, w) 給 X, Y, Z 軸
    u, v, w = rotated_axes[0, :], rotated_axes[1, :], rotated_axes[2, :]
    
    return (x, y, z), (u, v, w)

def update(frame_count):
    """matplotlib 動畫的更新回呼函數"""
    ax.cla() # 清除上一幀
    
    # 重新設定座標軸限制與標籤
    ax.set_xlim([-AXIS_LIM, AXIS_LIM])
    ax.set_ylim([-AXIS_LIM, AXIS_LIM])
    ax.set_zlim([0, AXIS_LIM * 2])
    ax.invert_yaxis()
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Depth Z (m)')
    ax.set_title("3D Space Simulator\nCamera at Origin (0,0,0)")
    
    # 繪製相機機身 (在原點)
    ax.scatter(0, 0, 0, color='black', s=100, marker='^', label='Camera')
    # 相機的視野方向 (Z軸)
    ax.quiver(0, 0, 0, 0, 0, 0.2, color='k', linestyle='dashed', arrow_length_ratio=0.1)

    tags, frame = detector.detect()
    
    if not tags:
        ax.text2D(0.05, 0.95, "No Tag Detected", transform=ax.transAxes, color='red')
        return

    for tag in tags:
        if tag.pose_t is not None and tag.pose_R is not None:
            tvec = tag.pose_t
            R = tag.pose_R

            x, y, z = tvec[0][0], tvec[1][0], tvec[2][0]
            
            # 畫出標籤的中心點
            ax.scatter(x, y, z, color='orange', s=50, label=f'Tag {tag.tag_id}')
            
            # 取得 3D 座標軸所需的數據
            (px, py, pz), (u, v, w) = get_quiver_data(tvec, R, size=0.15)
            
            # 畫出標籤的 X 軸 (紅色)
            ax.quiver(px, py, pz, u[0], v[0], w[0], color='r', arrow_length_ratio=0.1)
            # 畫出標籤的 Y 軸 (綠色)
            ax.quiver(px, py, pz, u[1], v[1], w[1], color='g', arrow_length_ratio=0.1)
            # 畫出標籤的 Z 軸 (藍色 - 標籤法線方向)
            ax.quiver(px, py, pz, u[2], v[2], w[2], color='b', arrow_length_ratio=0.1)

            # 在點旁邊加上數值標籤
            ax.text(x, y, z - 0.1, f" ID:{tag.tag_id}\n Z:{z:.2f}m", color='blue')

    ax.legend(loc='lower left')

print("啟動 matplotlib 動畫迴圈... 請在彈出的視窗中觀看。")
# 使用 FuncAnimation 定期呼叫 update 函數
ani = FuncAnimation(fig, update, interval=100, cache_frame_data=False)

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    print("釋放資源中...")
    detector.release()
    print("結束。")
