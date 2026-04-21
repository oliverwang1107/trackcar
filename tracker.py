"""
AprilTag 追蹤小車 — 追蹤控制模組

整合 AprilTag 偵測與馬達控制，使用 PID 控制器
實現平滑的水平追蹤（轉向）與距離維持（速度）。
"""

import time
import numpy as np
import config


class PIDController:
    """通用 PID 控制器。"""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -100, output_max: float = 100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self._prev_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()
        self._first_run = True

    def compute(self, error: float) -> float:
        """
        計算 PID 輸出。

        :param error: 當前誤差值
        :return: 控制輸出
        """
        now = time.time()
        dt = now - self._last_time
        if dt <= 0:
            dt = 0.001  # 防止除以零
        self._last_time = now

        if self._first_run:
            self._prev_error = error
            self._first_run = False

        # P
        p_out = self.kp * error

        # I（帶積分飽和防護）
        self._integral += error * dt
        self._integral = max(-50, min(50, self._integral))  # anti-windup
        i_out = self.ki * self._integral

        # D
        derivative = (error - self._prev_error) / dt
        d_out = self.kd * derivative
        self._prev_error = error

        # 總輸出
        output = p_out + i_out + d_out
        return max(self.output_min, min(self.output_max, output))

    def reset(self):
        """重設 PID 內部狀態。"""
        self._prev_error = 0.0
        self._integral = 0.0
        self._last_time = time.time()
        self._first_run = True


class LinearKalmanFilter:
    """
    線性卡爾曼濾波器，用於追蹤方塊的 X (橫向) 與 Z (深度) 座標及其速度。
    狀態向量: [x, z, vx, vz]^T
    """
    def __init__(self):
        import numpy as np
        # 狀態向量 [x, z, vx, vz]
        self.x = np.zeros((4, 1), dtype=np.float32)
        
        # 狀態協方差矩陣 P
        self.P = np.eye(4, dtype=np.float32) * 1.0
        
        # 觀測矩陣 H (我們只能觀測到位置 x, z)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        
        # 過程噪聲協方差 Q
        self.Q = np.array([
            [config.KF_Q_POSITION, 0, 0, 0],
            [0, config.KF_Q_POSITION, 0, 0],
            [0, 0, config.KF_Q_VELOCITY, 0],
            [0, 0, 0, config.KF_Q_VELOCITY]
        ], dtype=np.float32)
        
        # 觀測噪聲協方差 R
        self.R = np.eye(2, dtype=np.float32) * config.KF_R_MEASUREMENT
        
        self.last_time = time.time()
        self.initialized = False

    def reset(self, x, z):
        """重新初始化濾波器位置"""
        self.x[0, 0] = x
        self.x[1, 0] = z
        self.x[2, 0] = 0.0
        self.x[3, 0] = 0.0
        self.P = np.eye(4, dtype=np.float32) * 1.0
        self.last_time = time.time()
        self.initialized = True

    def predict(self):
        """根據時間差預測下一幀位置 (Constant Velocity Model)"""
        import numpy as np
        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        
        if dt <= 0:
            dt = 0.001
            
        # 狀態轉移矩陣 F
        F = np.array([
            [1, 0, dt,  0],
            [0, 1,  0, dt],
            [0, 0,  1,  0],
            [0, 0,  0,  1]
        ], dtype=np.float32)
        
        self.x = np.dot(F, self.x)
        self.P = np.dot(np.dot(F, self.P), F.T) + self.Q
        
        return self.x[0, 0], self.x[1, 0]

    def update(self, z_x, z_z):
        """使用相機觀測到的位置更新濾波器"""
        import numpy as np
        if not self.initialized:
            self.reset(z_x, z_z)
            return z_x, z_z
            
        Z = np.array([[z_x], [z_z]], dtype=np.float32)
        
        # y = Z - H*x (觀測殘差)
        y = Z - np.dot(self.H, self.x)
        
        # S = H*P*H^T + R
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        
        # K = P*H^T*S^-1 (卡爾曼增益)
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        
        # 後驗狀態更新
        self.x = self.x + np.dot(K, y)
        
        # 後驗協方差更新
        I = np.eye(4, dtype=np.float32)
        self.P = np.dot(I - np.dot(K, self.H), self.P)
        
        return self.x[0, 0], self.x[1, 0]

class Tracker:
    """
    AprilTag 追蹤控制器。

    根據偵測到的 tag 位置，使用兩個 PID 控制器：
    - 轉向 PID：修正水平偏移 → 差速轉彎
    - 速度 PID：根據 tag 面積控制前進速度 → 維持目標距離
    """

    def __init__(self, detector, motor_controller):
        self.detector = detector
        self.motors = motor_controller

        # 轉向 PID（輸入 = 水平偏移，輸出 = 左右速度差）
        self.steering_pid = PIDController(
            kp=config.STEERING_PID["Kp"],
            ki=config.STEERING_PID["Ki"],
            kd=config.STEERING_PID["Kd"],
            output_min=-config.MAX_SPEED,
            output_max=config.MAX_SPEED,
        )

        # 速度 PID（輸入 = 面積差，輸出 = 基礎速度修正）
        self.speed_pid = PIDController(
            kp=config.SPEED_PID["Kp"],
            ki=config.SPEED_PID["Ki"],
            kd=config.SPEED_PID["Kd"],
            output_min=-config.MAX_SPEED,
            output_max=config.MAX_SPEED,
        )

        self._lost_count = 0
        self._running = False
        
        # 狀態估計 (State Estimation) 變數
        self._last_time = time.time()
        self.est_dist = -1.0
        self.est_offset_x = 0.0
        
        # 加入卡爾曼濾波器
        self.kf = LinearKalmanFilter()
        
        self._current_left_speed = 0.0
        self._current_right_speed = 0.0

        print("[Tracker] 初始化完成")

    def _compute_motor_speeds(self, tag) -> tuple:
        """
        根據 tag 資訊計算左右馬達速度。
        優先選用 3D 位姿資訊，若無則降級為 2D 影像資訊。

        :param tag: TagInfo 或 BundleInfo 物件
        :return: (left_speed, right_speed)
        """
        # ---- 1. 取得距離與轉向誤差 ----
        use_3d = getattr(tag, 'pose_t', None) is not None

        if use_3d:
            # 距離誤差：當前 Z 軸距離 - 目標距離 (公尺)
            current_dist = tag.pose_t[2][0]
            dist_error = current_dist - config.TARGET_DISTANCE
            
            # 轉向誤差：橫向位移 (X 軸公尺)
            # tag_x > 0 表示 tag 在相機右側 -> 需要右轉 (steering > 0)
            steering_error = tag.pose_t[0][0]
            
            # Bundle 模式的 pose 已經是非常穩定的剛體中心，直接使用其 X, Z 即可
        else:
            # 降級方案：保留原版邏輯 (理論上 Bundle 模式不會觸發這個分支，除非失敗)
            dist_error = (config.TARGET_AREA_RATIO - tag.area_ratio) * 10.0
            steering_error = tag.offset_x * 0.15

        # ---- 2. 轉向死區處理 ----
        if abs(steering_error) < config.STEERING_DEADZONE:
            steering_error = 0.0
            self.steering_pid.reset()  # 進入死區時重設積分，防止積分纏繞 (Integral Windup)

        # ---- 3. PID 計算 ----
        speed_adjust = self.speed_pid.compute(dist_error)
        steering = self.steering_pid.compute(steering_error)

        # 基礎前進速度：完全由 PID 決定（誤差大則快，到達則停）
        # 若需要緩慢滑行，可加入 config.BASE_SPEED 的比例
        base_speed = speed_adjust

        # ---- 3. 差速混和 ----
        # 由於您遇到「看到 tag 卻轉走」的情況，這通常代表硬體上的左右馬達接線相反，
        # 或者攝影機是上下顛倒安裝的。
        # 這裡我們將轉向邏輯反轉，讓車子往反方向（正確的方向）追蹤！
        left_speed = base_speed - steering
        right_speed = base_speed + steering

        # ---- 4. 限幅與馬達補償 ----
        def finalize_speed(val):
            # 限制在最大速度內
            val = max(-config.MAX_SPEED, min(config.MAX_SPEED, val))
            # 處理死區：若速度太低馬達轉不動，則給予最小啟動速度 (除非目標是 0)
            if 0.5 < abs(val) < config.MIN_SPEED:
                return config.MIN_SPEED if val > 0 else -config.MIN_SPEED
            if abs(val) <= 0.5:
                return 0
            return val

        return finalize_speed(left_speed), finalize_speed(right_speed)

    def _search_mode(self):
        """當找不到 tag 時，使用走停走停 (Stop-and-Look) 方式搜尋，避免旋轉時的動態模糊。"""
        # 每 0.4 秒為一週期：前 0.2 秒旋轉，後 0.2 秒完全靜止讓相機對焦與擷取清晰影像
        t = time.time() % 0.4
        if t < 0.2:
            if self.est_offset_x < 0:
                self.motors.turn_left(config.SEARCH_SPEED)
            else:
                self.motors.turn_right(config.SEARCH_SPEED)
        else:
            self.motors.stop()

    def _get_action_name(self, left: float, right: float) -> str:
        """將馬達速度轉換為人類可讀的動作名稱。"""
        threshold = 5
        if abs(left) < threshold and abs(right) < threshold:
            return "STOPPED"
        
        diff = left - right
        avg = (left + right) / 2
        
        if abs(diff) > 20:
            if diff > 0: return "TURNING RIGHT"
            else: return "TURNING LEFT"
            
        if avg > threshold: return "MOVING FORWARD"
        if avg < -threshold: return "MOVING BACKWARD"
        
        return "ADJUSTING"

    def update(self, preview: bool = False) -> bool:
        """
        執行一次追蹤循環。

        :param preview: 是否顯示預覽視窗
        :return: True = 繼續運行，False = 使用者按 q 結束
        """
        # 使用 Bundle 偵測器取得剛體資料
        target, all_tags, frame = self.detector.get_bundle_target()

        # 如果 Bundle 找不到，退回找單一 Tag (備援方案)
        if target is None:
            target, all_tags, frame = self.detector.get_target(config.TARGET_TAG_ID)

        # ---- 1. 里程計預測與卡爾曼濾波 Prediction ----
        pred_x, pred_z = self.kf.predict()
        
        action = "UNKNOWN"

        if target is not None and getattr(target, 'pose_t', None) is not None:
            # ---- 2. 感測器融合 (Observation Update) ----
            obs_x = target.pose_t[0][0]
            obs_z = target.pose_t[2][0]
            
            # 使用 KF 進行測量更新
            upd_x, upd_z = self.kf.update(obs_x, obs_z)
            
            self.est_offset_x = upd_x
            self.est_dist = upd_z

            if self._lost_count > 0:
                self.speed_pid.reset()
                self.steering_pid.reset()
                self._current_left_speed = 0.0
                self._current_right_speed = 0.0

            self._lost_count = 0
            action = "TRACKING"
        else:
            # ---- 3. 目標遺失 (三個階段：預測 -> 後退 -> 旋轉搜尋) ----
            self._lost_count += 1
            
            if not self.kf.initialized:
                # 從未啟動過 KF，直接進入搜尋
                self._lost_count = config.LOST_FRAME_THRESHOLD

            if self._lost_count < config.PREDICT_THRESHOLD:
                # 階段 1: 短暫遺失 → 交給卡爾曼濾波器盲推 (Blind Tracking)
                action = "PREDICTING"
                if pred_z > 5.0 or pred_z < 0.0:
                    self._lost_count = config.PREDICT_THRESHOLD # 異常跳變，跳至下一階段
                else:
                    self.est_offset_x = pred_x
                    self.est_dist = pred_z
            
            # 注意：這裡用 if 而不是 elif，是因為上面的預測階段可能會因為異常而直接跳到下一階段
            if config.PREDICT_THRESHOLD <= self._lost_count < config.LOST_FRAME_THRESHOLD:
                # 階段 2: 後退尋找 (增加視野以增加捕捉概率)
                action = "BACKING_UP"
                self._current_left_speed = -config.BACKUP_SPEED
                self._current_right_speed = -config.BACKUP_SPEED
                self.motors.set_motors(self._current_left_speed, self._current_right_speed)
                
            elif self._lost_count >= config.LOST_FRAME_THRESHOLD:
                # 階段 3: 連續遺失太多幀，進入原地搜尋模式
                action = "SEARCHING"
                if self.est_offset_x < 0:
                    self._current_left_speed = -config.SEARCH_SPEED
                    self._current_right_speed = config.SEARCH_SPEED
                else:
                    self._current_left_speed = config.SEARCH_SPEED
                    self._current_right_speed = -config.SEARCH_SPEED
                self._search_mode()

        # ---- 4. 馬達控制計算 ----
        # 只要不是在搜尋模式，我們就根據「估計的狀態」來進行 PID 控制
        if action in ("TRACKING", "PREDICTING"):
            class EstimatedTarget:
                pass
            est_target = EstimatedTarget()
            est_target.offset_x = self.est_offset_x
            # 簡單反推面積：假設距離 0.25 時面積是 0.08
            est_target.area_ratio = 0.005 / (self.est_dist**2 + 0.0001)
            
            import numpy as np
            # 重建 3D pose_t
            est_target.pose_t = np.array([[self.est_offset_x * 0.15], [0.0], [self.est_dist]])
            
            target_left, target_right = self._compute_motor_speeds(est_target)
            
            # 馬達輸出平滑化 (Rate Limiting)
            max_delta = 35.0
            if target_left > self._current_left_speed + max_delta:
                self._current_left_speed += max_delta
            elif target_left < self._current_left_speed - max_delta:
                self._current_left_speed -= max_delta
            else:
                self._current_left_speed = target_left
                
            if target_right > self._current_right_speed + max_delta:
                self._current_right_speed += max_delta
            elif target_right < self._current_right_speed - max_delta:
                self._current_right_speed -= max_delta
            else:
                self._current_right_speed = target_right
                
            self.motors.set_motors(self._current_left_speed, self._current_right_speed)
            
            if action == "TRACKING":
                action = self._get_action_name(self._current_left_speed, self._current_right_speed)
                
            if int(time.time() * 10) % 2 == 0:
                print(f"[{action}] Est Dist: {self.est_dist:.2f}m, Est Off: {self.est_offset_x:+.2f}, L: {self._current_left_speed:.0f}, R: {self._current_right_speed:.0f}")

        # ---- 預覽 ----
        if preview and frame is not None:
            frame = self.detector.draw_tags(frame, all_tags)

            # 顯示估計點 (用圓圈表示)
            if self.est_dist > 0:
                # 將 est_offset_x (-1.0 ~ 1.0) 轉回像素坐標
                center_x = int((self.est_offset_x + 1.0) * (config.CAMERA_WIDTH / 2.0))
                center_y = int(config.CAMERA_HEIGHT / 2.0)
                
                import cv2
                color_est = (255, 0, 255) if action == "PREDICTING" else (0, 255, 0)
                cv2.circle(frame, (center_x, center_y), 8, color_est, -1)
                cv2.putText(frame, "EST", (center_x + 10, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_est, 2)

            # 顯示狀態文字
            if action == "PREDICTING":
                status_list = [
                    f"PREDICTING ({self._lost_count})",
                    f"EST_DIST: {self.est_dist:.3f} m",
                    f"EST_OFF: {self.est_offset_x:+.2f}"
                ]
                color = (255, 0, 255) # 紫色
            elif action == "BACKING_UP":
                status_list = [
                    "BACKING UP (RECOVERY)",
                    f"LOST: {self._lost_count} frames"
                ]
                color = (0, 165, 255) # 橘色
            elif action == "SEARCHING":
                status_list = ["SEARCHING..."]
                color = (0, 0, 255) # 紅色
            else:
                status_list = [
                    f"ACTION: {action}",
                    f"EST_DIST: {self.est_dist:.3f} m",
                    f"EST_OFF: {self.est_offset_x:+.2f}"
                ]
                color = (0, 255, 0) # 綠色

            import cv2
            for i, text in enumerate(status_list):
                cv2.putText(
                    frame, text, (10, 35 + i*30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 3, # 黑色外框增加對比
                )
                cv2.putText(
                    frame, text, (10, 35 + i*30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2,
                )
            cv2.imshow("AprilTag Tracker", frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                return False

        return True

    def run(self, preview: bool = False):
        """
        主追蹤迴圈。

        :param preview: 是否顯示預覽視窗
        """
        self._running = True
        print("[Tracker] 開始追蹤...")
        print("         按 Ctrl+C 停止" + ("（或在預覽視窗按 q）" if preview else ""))

        try:
            while self._running:
                if not self.update(preview=preview):
                    break
        except KeyboardInterrupt:
            print("\n[Tracker] 收到 Ctrl+C，停止追蹤。")
        finally:
            self.stop()

    def stop(self):
        """停止追蹤並清理資源。"""
        self._running = False
        self.motors.stop()

        import cv2
        cv2.destroyAllWindows()

        print("[Tracker] 已停止")
