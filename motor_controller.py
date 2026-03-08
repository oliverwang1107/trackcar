"""
AprilTag 追蹤小車 — 馬達控制模組

透過 TB6612FNG 驅動模組控制兩顆 TT 減速馬達。
支援前進、後退、轉彎、差速控制。
"""

import RPi.GPIO as GPIO
import config


class MotorController:
    """TB6612FNG 雙馬達控制器。"""

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # ---- 設定所有控制腳位為輸出 ----
        self._setup_pin(config.MOTOR_A["IN1"])
        self._setup_pin(config.MOTOR_A["IN2"])
        self._setup_pin(config.MOTOR_B["IN1"])
        self._setup_pin(config.MOTOR_B["IN2"])
        self._setup_pin(config.STBY_PIN)

        # ---- 設定 PWM 腳位 ----
        GPIO.setup(config.MOTOR_A["PWM"], GPIO.OUT)
        GPIO.setup(config.MOTOR_B["PWM"], GPIO.OUT)
        self._pwm_a = GPIO.PWM(config.MOTOR_A["PWM"], config.PWM_FREQUENCY)
        self._pwm_b = GPIO.PWM(config.MOTOR_B["PWM"], config.PWM_FREQUENCY)
        self._pwm_a.start(0)
        self._pwm_b.start(0)

        # ---- 啟用 TB6612（STBY 拉高） ----
        GPIO.output(config.STBY_PIN, GPIO.HIGH)

        print("[MotorController] 初始化完成")

    # ------------------------------------------------------------------
    #  內部工具
    # ------------------------------------------------------------------

    @staticmethod
    def _setup_pin(pin: int):
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

    def _set_motor(self, in1: int, in2: int, pwm, speed: float):
        """
        控制單顆馬達。

        :param speed: -100 ~ 100（負數 = 反轉）
        """
        speed = max(-100, min(100, speed))

        if speed > 0:
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
        elif speed < 0:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
        else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)

        pwm.ChangeDutyCycle(abs(speed))

    # ------------------------------------------------------------------
    #  公開 API
    # ------------------------------------------------------------------

    def set_motors(self, left_speed: float, right_speed: float):
        """
        分別設定左右馬達速度。

        :param left_speed:  -100 ~ 100
        :param right_speed: -100 ~ 100
        """
        self._set_motor(
            config.MOTOR_A["IN1"], config.MOTOR_A["IN2"],
            self._pwm_a, left_speed,
        )
        self._set_motor(
            config.MOTOR_B["IN1"], config.MOTOR_B["IN2"],
            self._pwm_b, right_speed,
        )

    def forward(self, speed: float = None):
        """兩輪同速前進。"""
        speed = speed if speed is not None else config.BASE_SPEED
        self.set_motors(speed, speed)

    def backward(self, speed: float = None):
        """兩輪同速後退。"""
        speed = speed if speed is not None else config.BASE_SPEED
        self.set_motors(-speed, -speed)

    def turn_left(self, speed: float = None):
        """原地左轉（左輪反轉、右輪正轉）。"""
        speed = speed if speed is not None else config.BASE_SPEED
        self.set_motors(-speed, speed)

    def turn_right(self, speed: float = None):
        """原地右轉（左輪正轉、右輪反轉）。"""
        speed = speed if speed is not None else config.BASE_SPEED
        self.set_motors(speed, -speed)

    def stop(self):
        """停止所有馬達。"""
        self.set_motors(0, 0)

    def standby(self, enable: bool = True):
        """進入 / 離開待機模式。"""
        GPIO.output(config.STBY_PIN, GPIO.LOW if enable else GPIO.HIGH)

    def cleanup(self):
        """安全釋放 GPIO 資源。"""
        self.stop()
        self._pwm_a.stop()
        self._pwm_b.stop()
        GPIO.cleanup()
        print("[MotorController] GPIO 已釋放")
