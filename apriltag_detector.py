"""
AprilTag 追蹤小車 — AprilTag 偵測模組

使用 USB 攝影機擷取影像，透過 pupil-apriltags 偵測 AprilTag，
回傳 tag 的 ID、中心座標、角點、面積等資訊。
"""

import os
import cv2
import numpy as np
from pupil_apriltags import Detector

import config


class TagInfo:
    """封裝單一 AprilTag 偵測結果。"""

    def __init__(self, detection, frame_w: int, frame_h: int):
        self.tag_id: int = detection.tag_id
        self.center = detection.center            # (cx, cy)
        self.corners = detection.corners           # 4 個角點
        self.decision_margin = detection.decision_margin

        # 3D Pose 資訊 (若有啟用估計)
        self.pose_R = getattr(detection, 'pose_R', None) # 3x3 旋轉矩陣
        self.pose_t = getattr(detection, 'pose_t', None) # 3x1 平移向量 (距離、座標)

        # 計算面積（用角點做四邊形面積）
        pts = detection.corners
        self.area = 0.5 * abs(
            (pts[0][0] * pts[1][1] - pts[1][0] * pts[0][1])
            + (pts[1][0] * pts[2][1] - pts[2][0] * pts[1][1])
            + (pts[2][0] * pts[3][1] - pts[3][0] * pts[2][1])
            + (pts[3][0] * pts[0][1] - pts[0][0] * pts[3][1])
        )

        # 水平偏移量：-1.0（最左）~ +1.0（最右），0 = 正中央
        self.offset_x = (self.center[0] / frame_w - 0.5) * 2.0

        # 面積佔比（用於距離估算：面積越大 → 越近）
        self.area_ratio = self.area / (frame_w * frame_h)

    def __repr__(self):
        return (
            f"TagInfo(id={self.tag_id}, "
            f"center=({self.center[0]:.0f}, {self.center[1]:.0f}), "
            f"offset_x={self.offset_x:+.2f}, "
            f"area_ratio={self.area_ratio:.4f})"
        )


class BundleInfo:
    """封裝標籤束 (Tag Bundle) 的多點聯合姿態估計結果。"""
    def __init__(self, rvec, tvec, frame_w: int, frame_h: int):
        self.pose_R, _ = cv2.Rodrigues(rvec)
        self.pose_t = tvec
        
        # 為了相容原本的 TagInfo 介面，提供 offset_x (雖然是 3D，但也可提供 2D 投影估計)
        # 簡單投影中心點 (0,0,0) 回 2D 畫面計算 offset_x
        # 這裡簡化處理：直接用這 16 個點的 3D 解算，其 x 軸平移量代表左右偏移
        # 水平偏移量估算，這裡不精確等同像素偏移，因此在 tracker 內主要依賴 pose_t
        self.offset_x = 0.0 # 後續在 tracker.py 我們會直接使用 pose_t[0][0] 來做誤差
        self.area_ratio = 1.0 # Bundle 模式主要依賴 3D 距離，area 不重要
        self.tag_id = "BUNDLE"


class AprilTagDetector:
    """AprilTag 偵測器：擷取影像並偵測 tag。"""

    def __init__(self):
        # ---- 初始化攝影機 ----
        self.cap = cv2.VideoCapture(config.CAMERA_INDEX, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, config.CAMERA_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 限制緩衝區為 1，確保取得最新影像，避免延遲導致震盪

        if not self.cap.isOpened():
            raise RuntimeError(
                f"無法開啟攝影機（裝置索引 {config.CAMERA_INDEX}）。"
                " 請檢查 USB 攝影機是否已連接。"
            )

        # 讀取實際解析度（可能與設定不同）
        self.frame_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # ---- 嘗試讀取相機內參 ----
        self.camera_params = None
        self.camera_matrix = None
        self.dist_coeffs = None
        
        if os.path.exists(config.CAMERA_PARAMS_FILE):
            try:
                data = np.load(config.CAMERA_PARAMS_FILE)
                mtx = data['mtx']
                self.dist_coeffs = data['dist']
                self.camera_matrix = mtx
                # pupil-apriltags 需要的格式：[fx, fy, cx, cy]
                self.camera_params = [mtx[0,0], mtx[1,1], mtx[0,2], mtx[1,2]]
                print(f"[AprilTagDetector] 成功載入相機內參：{self.camera_params}")
            except Exception as e:
                print(f"[AprilTagDetector] 無法讀取相機內參 {config.CAMERA_PARAMS_FILE}: {e}")

        # ---- 預先建構 Bundle 的 3D Object Points ----
        # 以方塊中心為原點 (0,0,0)
        # Z軸向前 (指離相機)，X軸向右，Y軸向下。
        # FRONT 面 朝向 -Z (面向相機)。
        # pupiltags corners: bottom-left, bottom-right, top-right, top-left
        self.bundle_obj_points = {}
        s = config.CUBE_SIZE / 2.0
        ts = config.TAG_SIZE / 2.0
        
        # 定義 4 個面上的 Tag 角點 (以中心為基準)
        # p1: 左下, p2: 右下, p3: 右上, p4: 左上 (這是從 Tag 正面看過去的 local 座標)
        # Tag Local: X右, Y上(錯了，一般pupil Y 是下), Z向內
        # 確保順序對應: bottom-left(-ts, ts), bottom-right(ts, ts), top-right(ts, -ts), top-left(-ts, -ts)
        
        # FRONT (面朝 -Z, Tag X與世界X同向, Tag Y與世界Y同向)
        self.bundle_obj_points[config.BUNDLE_TAGS.get("FRONT", 0)] = np.array([
            [-ts,  ts, -s],  # bottom-left
            [ ts,  ts, -s],  # bottom-right
            [ ts, -ts, -s],  # top-right
            [-ts, -ts, -s]   # top-left
        ], dtype=np.float32)

        # BACK (面朝 +Z, 從後面看 X要反轉才能看正 Tag)
        self.bundle_obj_points[config.BUNDLE_TAGS.get("BACK", 2)] = np.array([
            [ ts,  ts,  s],  # bottom-left
            [-ts,  ts,  s],  # bottom-right
            [-ts, -ts,  s],  # top-right
            [ ts, -ts,  s]   # top-left
        ], dtype=np.float32)

        # RIGHT (面朝 +X)
        self.bundle_obj_points[config.BUNDLE_TAGS.get("RIGHT", 1)] = np.array([
            [ s,  ts,  ts],  # bottom-left
            [ s,  ts, -ts],  # bottom-right
            [ s, -ts, -ts],  # top-right
            [ s, -ts,  ts]   # top-left
        ], dtype=np.float32)

        # LEFT (面朝 -X)
        self.bundle_obj_points[config.BUNDLE_TAGS.get("LEFT", 3)] = np.array([
            [-s,  ts, -ts],  # bottom-left
            [-s,  ts,  ts],  # bottom-right
            [-s, -ts,  ts],  # top-right
            [-s, -ts, -ts]   # top-left
        ], dtype=np.float32)

        # ---- 初始化 AprilTag 偵測器 ----
        self.detector = Detector(
            families=config.TAG_FAMILY,
            nthreads=config.TAG_DETECT_THREADS,
            quad_decimate=1.0,      # 設為 1.0 不降低解析度，大幅提升偵測清晰度與距離
            quad_sigma=0.0,
            decode_sharpening=0.25,
        )

        print(
            f"[AprilTagDetector] 初始化完成 "
            f"({self.frame_w}x{self.frame_h} @ {config.CAMERA_FPS}fps, "
            f"family={config.TAG_FAMILY})"
        )

    def detect(self) -> tuple:
        """
        擷取一幀影像並偵測 AprilTag。

        :return: (tags, frame)
            tags  — TagInfo 物件列表
            frame — 原始 BGR 影像（可供預覽使用）
        """
        ret, frame = self.cap.read()
        if not ret:
            return [], None

        # 轉為灰階供偵測
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.camera_params is not None:
            detections = self.detector.detect(
                gray, 
                estimate_tag_pose=True, 
                camera_params=self.camera_params, 
                tag_size=config.TAG_SIZE
            )
        else:
            detections = self.detector.detect(gray)

        tags = [
            TagInfo(d, self.frame_w, self.frame_h)
            for d in detections
        ]

        return tags, frame

    def get_bundle_target(self) -> tuple:
        """
        取得 Bundle (方塊剛體) 的聯合 3D Pose。
        使用 solvePnPRansac 解決果凍效應與晃動問題。
        
        :return: (bundle_target, all_tags_info, frame)
        """
        tags, frame = self.detect()
        
        if not tags or self.camera_matrix is None:
            return None, tags, frame
            
        obj_pts_list = []
        img_pts_list = []
        
        # 收集所有屬於 Bundle 的 tag 角點
        for tag in tags:
            if tag.tag_id in self.bundle_obj_points:
                obj_pts_list.append(self.bundle_obj_points[tag.tag_id])
                img_pts_list.append(tag.corners)
                
        if not obj_pts_list:
            return None, tags, frame
            
        # 攤平為 N x 3 與 N x 2 陣列
        all_obj_pts = np.vstack(obj_pts_list)
        all_img_pts = np.vstack(img_pts_list).astype(np.float32)
        
        # 至少要有 4 個點 (1 個 tag) 才能解 PnP，越多越穩定
        if len(all_img_pts) >= 4:
            # 使用 RANSAC 剔除被果凍效應扭曲的離群點
            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                all_obj_pts, 
                all_img_pts, 
                self.camera_matrix, 
                self.dist_coeffs,
                reprojectionError=8.0, # 容許 8 pixels 的投影誤差 (因應果凍效應)
                iterationsCount=100
            )
            
            if success:
                bundle = BundleInfo(rvec, tvec, self.frame_w, self.frame_h)
                return bundle, tags, frame

        return None, tags, frame

    def get_target(self, tag_id: int = None) -> tuple:
        """
        取得目標 tag。

        :param tag_id: 指定 tag ID。None = 回傳面積最大的 tag。
        :return: (target_tag, all_tags, frame)
        """
        tags, frame = self.detect()

        if not tags:
            return None, tags, frame

        if tag_id is not None:
            # 尋找指定 ID
            for tag in tags:
                if tag.tag_id == tag_id:
                    return tag, tags, frame
            return None, tags, frame
        else:
            # 回傳面積最大的 tag
            largest = max(tags, key=lambda t: t.area)
            return largest, tags, frame

    @staticmethod
    def draw_tags(frame, tags: list):
        """
        在影像上標註偵測到的 tag（用於預覽）。

        :param frame: BGR 影像
        :param tags: TagInfo 列表
        :return: 標註後的影像
        """
        if frame is None:
            return frame

        for tag in tags:
            # 畫出四邊形外框
            pts = tag.corners.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

            # 標註 tag ID
            cx, cy = int(tag.center[0]), int(tag.center[1])
            cv2.putText(
                frame, f"ID:{tag.tag_id}",
                (cx - 20, cy - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )

            # 標註中心點
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # 標註偏移量
            cv2.putText(
                frame, f"off:{tag.offset_x:+.2f}",
                (cx - 30, cy + 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1,
            )
            
            # 若有 Pose，顯示 Z 軸距離與角度
            if tag.pose_t is not None:
                dist_z = tag.pose_t[2][0]
                dist_x = tag.pose_t[0][0]
                cv2.putText(
                    frame, f"Z: {dist_z:.2f}m",
                    (cx - 30, cy + 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 255), 2,
                )

        return frame

    def release(self):
        """釋放攝影機資源。"""
        if self.cap.isOpened():
            self.cap.release()
        print("[AprilTagDetector] 攝影機已釋放")
