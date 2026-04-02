"""
AprilTag 追蹤小車 — 馬達控制模組

透過 TB6612FNG 驅動模組控制兩顆 TT 減速馬達。
支援前進、後退、轉彎、差速控制。
改用 gpiozero 以支援 Raspberry Pi 5 與最新的 Bookworm 系統。
"""

from gpiozero import PWMOutputDevice, DigitalOutputDevice
import config

class MotorController:
    """TB6612FNG 雙馬達控制器。"""

    def __init__(self):
        # 設定並啟用 STBY 腳位
        self._stby = DigitalOutputDevice(config.STBY_PIN, initial_value=True)

        # 初始化馬達 A (左輪)
        self._in1_a = DigitalOutputDevice(config.MOTOR_A["IN1"], initial_value=False)
        self._in2_a = DigitalOutputDevice(config.MOTOR_A["IN2"], initial_value=False)
        self._pwm_a = PWMOutputDevice(config.MOTOR_A["PWM"], frequency=config.PWM_FREQUENCY, initial_value=0)

        # 初始化馬達 B (右輪)
        self._in1_b = DigitalOutputDevice(config.MOTOR_B["IN1"], initial_value=False)
        self._in2_b = DigitalOutputDevice(config.MOTOR_B["IN2"], initial_value=False)
        self._pwm_b = PWMOutputDevice(config.MOTOR_B["PWM"], frequency=config.PWM_FREQUENCY, initial_value=0)

        print("[MotorController] 初始化完成 (使用 gpiozero)")

    # ------------------------------------------------------------------
    #  內部工具
    # ------------------------------------------------------------------

    def _set_motor(self, in1, in2, pwm, speed: float):
        """
        控制單顆馬達。
        """
        speed_val = max(-100, min(100, speed)) / 100.0

        if speed_val > 0:
            in1.on()
            in2.off()
        elif speed_val < 0:
            in1.off()
            in2.on()
        else:
            in1.off()
            in2.off()

        pwm.value = abs(speed_val)

    # ------------------------------------------------------------------
    #  公開 API
    # ------------------------------------------------------------------

    def set_motors(self, left_speed: float, right_speed: float):
        """
        分別設定左右馬達速度。

        :param left_speed:  -100 ~ 100
        :param right_speed: -100 ~ 100
        """
        self._set_motor(self._in1_a, self._in2_a, self._pwm_a, left_speed)
        self._set_motor(self._in1_b, self._in2_b, self._pwm_b, right_speed)

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
        if enable:
            self._stby.off()
        else:
            self._stby.on()

    def cleanup(self):
        """安全釋放 GPIO 資源。"""
        self.stop()
        self._pwm_a.close()
        self._pwm_b.close()
        self._in1_a.close()
        self._in2_a.close()
        self._in1_b.close()
        self._in2_b.close()
        self._stby.close()
        print("[MotorController] GPIO 已釋放")

