#!/usr/import/env python3
"""
3D 姿態可視化演示 (3D Pose AR Visualization)

讀取相機與相機內參，即時偵測 AprilTag 並透過 OpenCV 的 drawFrameAxes 
將標籤的 3D 座標軸 (X=紅, Y=綠, Z=藍) 以及一個虛擬立體方塊渲染到畫面上。
"""

import cv2
import numpy as np
import os
import time

import config
from apriltag_detector import AprilTagDetector

# 強制將畫面輸出到本機端螢幕
if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0"

def draw_cube(img, imgpts):
    """
    在影像上繪製一個 3D 立方體
    :param img: 原始影像
    :param imgpts: 投影後的 8 個端點 2D 座標
    """
    imgpts = np.int32(imgpts).reshape(-1, 2)
    # 畫前面 (綠色)
    cv2.drawContours(img, [imgpts[:4]], -1, (0, 255, 0), 2)
    # 畫支柱 (藍色)
    for i, j in zip(range(4), range(4, 8)):
        cv2.line(img, tuple(imgpts[i]), tuple(imgpts[j]), (255, 0, 0), 2)
    # 畫後面 (紅色)
    cv2.drawContours(img, [imgpts[4:]], -1, (0, 0, 255), 2)
    return img

def main():
    print("=======================================")
    print("      3D 姿態與擴增實境 (AR) 演示")
    print("=======================================")

    # 檢查是否已校正
    if not os.path.exists(config.CAMERA_PARAMS_FILE):
        print("❌ 找不到 `camera_params.npz`，請先執行 `python3 calibrate_camera.py` 進行相機校正。")
        return

    data = np.load(config.CAMERA_PARAMS_FILE)
    camera_matrix = data['mtx']
    dist_coeffs = data['dist']

    detector = AprilTagDetector()
    
    print("準備就緒！請將 AprilTag 放在鏡頭前。")
    print("按下 [q] 鍵或 [Esc] 鍵結束演示。")

    # 定義一個虛擬立體方塊的 8 個頂點 (以剛體中心為原點，符合 config 定義)
    s = config.CUBE_SIZE / 2.0
    cube_points = np.float32([
        [-s, s, -s], [s, s, -s], [s, -s, -s], [-s, -s, -s],          # 前面 (FRONT, Z=-s)
        [-s, s,  s], [s, s,  s], [s, -s,  s], [-s, -s,  s],          # 後面 (BACK, Z=s)
    ])
    # 這裡為求畫圖簡單，直接連線前四個跟後四個點 (底跟柱)。需要修正 draw_cube 的接線法。

    try:
        while True:
            # 優先使用 Bundle 聯合偵測，畫面會更加穩定
            bundle, tags, frame = detector.get_bundle_target()
            
            if frame is None:
                continue
                
            # 若有 Bundle，則畫出這個大方塊的姿態
            if bundle is not None and getattr(bundle, 'pose_t', None) is not None:
                rvec, _ = cv2.Rodrigues(bundle.pose_R)
                tvec = bundle.pose_t
                
                # 1. 畫出 3D 座標軸 (長度為方塊邊長)
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, config.CUBE_SIZE)
                
                # 2. 將大方塊的 8 個頂點投影到 2D 畫面上
                imgpts, jac = cv2.projectPoints(cube_points, rvec, tvec, camera_matrix, dist_coeffs)
                frame = draw_cube(frame, imgpts)
                
                # 3. 標註文字參數
                dist_z = tvec[2][0]
                # 嘗試將中心投影
                cen_pts, _ = cv2.projectPoints(np.float32([[0,0,0]]), rvec, tvec, camera_matrix, dist_coeffs)
                cx, cy = int(cen_pts[0][0][0]), int(cen_pts[0][0][1])
                cv2.putText(frame, f"BUNDLE Z: {dist_z:.3f}m", (cx - 60, cy + 60), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                            
            # 仍可畫出原始的小 tag 以便除錯
            for tag in tags:
                if getattr(tag, 'pose_t', None) is not None:
                    # 畫小 tag 座標軸
                    rvec, _ = cv2.Rodrigues(tag.pose_R)
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tag.pose_t, config.TAG_SIZE / 2)

            cv2.imshow('3D Pose AR Simulator', frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
                
    except KeyboardInterrupt:
        pass
    finally:
        detector.release()
        cv2.destroyAllWindows()
        print("演示結束。")

if __name__ == "__main__":
    main()
