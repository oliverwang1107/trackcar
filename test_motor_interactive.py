import time
import sys
from motor_controller import MotorController

def interactive_test():
    print("--- 🚀 馬達持續測試程式 (供三用電表測量) ---")
    print("正在初始化馬達控制器...")
    
    try:
        mc = MotorController()
    except Exception as e:
        print(f"❌ 初始化失敗: {e}")
        return

    print("\n[控制指令]")
    print("  f : 兩輪前進 (100%)")
    print("  b : 兩輪後退 (100%)")
    print("  l : 原地左旋轉 (100%)")
    print("  r : 原地右旋轉 (100%)")
    print("  s : 停止 (Stop)")
    print("  q : 退出程式 (Quit)")
    print("-" * 40)

    try:
        while True:
            # 使用 input() 獲取指令。注意：在某些環境下 stdin 可能不正常。
            # 如果是透過 antigravity 執行，我會手動輸入。
            cmd = input("請輸入指令 > ").strip().lower()
            
            if cmd == 'f':
                print(">>> 設定：前進 (100% 速度)")
                mc.forward(100)
            elif cmd == 'b':
                print(">>> 設定：後退 (100% 速度)")
                mc.backward(100)
            elif cmd == 'l':
                print(">>> 設定：原地左旋轉")
                mc.turn_left(100)
            elif cmd == 'r':
                print(">>> 設定：原地右旋轉")
                mc.turn_right(100)
            elif cmd == 's':
                print(">>> 執行：停止")
                mc.stop()
            elif cmd == 'q':
                print("準備退出...")
                break
            elif cmd == '':
                continue
            else:
                print(f"未知指令 '{cmd}'，請重新輸入。")
            
            print("  (狀態已鎖定，可使用電表測量電壓。輸入 's' 停止。)\n")

    except KeyboardInterrupt:
        print("\n使用者中斷")
    finally:
        mc.cleanup()
        print("--- 🏁 測試結束，GPIO 已釋放 ---")

if __name__ == "__main__":
    # 強制不使用緩衝區
    sys.stdout.reconfigure(line_buffering=True)
    interactive_test()
