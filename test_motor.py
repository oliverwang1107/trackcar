import time
from motor_controller import MotorController

def test_motors():
    print("初始化馬達控制器...")
    mc = MotorController()

    try:
        print("馬達前進 2 秒 (50% 速度)...")
        mc.forward(50)
        time.sleep(2)
        
        print("馬達停止 1 秒...")
        mc.stop()
        time.sleep(1)
        
        print("馬達後退 2 秒 (50% 速度)...")
        mc.backward(50)
        time.sleep(2)

        print("左轉 1 秒...")
        mc.turn_left(50)
        time.sleep(1)

        print("右轉 1 秒...")
        mc.turn_right(50)
        time.sleep(1)

        print("馬達停止...")
        mc.stop()

    except KeyboardInterrupt:
        print("\n使用者中斷測試")
    finally:
        mc.cleanup()
        print("測試結束")

if __name__ == "__main__":
    test_motors()
