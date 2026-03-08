"""
AprilTag 追蹤小車 — 追蹤控制模組

整合 AprilTag 偵測與馬達控制，使用 PID 控制器
實現平滑的水平追蹤（轉向）與距離維持（速度）。
"""

import time
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

        print("[Tracker] 初始化完成")

    def _compute_motor_speeds(self, tag) -> tuple:
        """
        根據 tag 資訊計算左右馬達速度。

        :param tag: TagInfo 物件
        :return: (left_speed, right_speed)
        """
        # ---- 轉向控制 ----
        # offset_x > 0 → tag 在右邊 → 需要右轉 → 左輪加速、右輪減速
        steering = self.steering_pid.compute(tag.offset_x)

        # ---- 速度控制 ----
        # area_ratio < TARGET → 太遠 → 前進
        # area_ratio > TARGET → 太近 → 後退
        area_error = config.TARGET_AREA_RATIO - tag.area_ratio
        speed_adjust = self.speed_pid.compute(area_error)
        base_speed = config.BASE_SPEED + speed_adjust

        # 確保基礎速度在合理範圍
        base_speed = max(-config.MAX_SPEED, min(config.MAX_SPEED, base_speed))

        # ---- 混合：差速轉向 ----
        left_speed = base_speed + steering
        right_speed = base_speed - steering

        # 限幅
        left_speed = max(-config.MAX_SPEED, min(config.MAX_SPEED, left_speed))
        right_speed = max(-config.MAX_SPEED, min(config.MAX_SPEED, right_speed))

        return left_speed, right_speed

    def _search_mode(self):
        """當找不到 tag 時，原地慢速旋轉搜尋。"""
        self.motors.turn_right(config.SEARCH_SPEED)

    def update(self, preview: bool = False) -> bool:
        """
        執行一次追蹤循環。

        :param preview: 是否顯示預覽視窗
        :return: True = 繼續運行，False = 使用者按 q 結束
        """
        target, all_tags, frame = self.detector.get_target(
            tag_id=config.TARGET_TAG_ID
        )

        if target is not None:
            # ---- 偵測到目標 ----
            self._lost_count = 0
            left_speed, right_speed = self._compute_motor_speeds(target)
            self.motors.set_motors(left_speed, right_speed)
        else:
            # ---- 目標遺失 ----
            self._lost_count += 1

            if self._lost_count >= config.LOST_FRAME_THRESHOLD:
                # 連續遺失太多幀 → 進入搜尋模式
                self._search_mode()
            else:
                # 短暫遺失 → 減速滑行
                self.motors.stop()

        # ---- 預覽 ----
        if preview and frame is not None:
            frame = self.detector.draw_tags(frame, all_tags)

            # 顯示狀態文字
            if target:
                status = (
                    f"Tracking ID:{target.tag_id} "
                    f"off:{target.offset_x:+.2f} "
                    f"area:{target.area_ratio:.4f}"
                )
                color = (0, 255, 0)
            elif self._lost_count >= config.LOST_FRAME_THRESHOLD:
                status = "SEARCHING..."
                color = (0, 165, 255)
            else:
                status = f"LOST ({self._lost_count}/{config.LOST_FRAME_THRESHOLD})"
                color = (0, 0, 255)

            import cv2
            cv2.putText(
                frame, status, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2,
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
