#!/usr/bin/env python3
"""
AprilTag 追蹤小車 — 主程式入口

使用方式：
    python3 main.py                 # 基本追蹤模式
    python3 main.py --preview       # 開啟即時預覽視窗
    python3 main.py --tag-id 3      # 只追蹤 ID=3 的 tag
    python3 main.py --speed 50      # 設定基礎速度為 50
    python3 main.py --test-motors   # 測試馬達（不啟動攝影機）
"""

import argparse
import sys
import time

import config


def test_motors():
    """馬達功能測試：依序測試各種運動方向。"""
    from motor_controller import MotorController

    mc = MotorController()
    test_speed = 40
    duration = 1.5

    tests = [
        ("前進", mc.forward),
        ("後退", mc.backward),
        ("左轉", mc.turn_left),
        ("右轉", mc.turn_right),
    ]

    try:
        for name, action in tests:
            print(f"  測試：{name} (速度={test_speed}, 持續={duration}s)")
            action(test_speed)
            time.sleep(duration)
            mc.stop()
            time.sleep(0.5)

        print("\n  差速測試：左 60 / 右 30")
        mc.set_motors(60, 30)
        time.sleep(duration)
        mc.stop()

        print("\n✅ 馬達測試完成！")
    except KeyboardInterrupt:
        print("\n⚠️ 測試中斷。")
    finally:
        mc.cleanup()


def test_camera():
    """攝影機 AprilTag 偵測測試（不啟動馬達）。"""
    import cv2
    from apriltag_detector import AprilTagDetector

    detector = AprilTagDetector()
    print("[測試] 攝影機 AprilTag 偵測測試。按 q 結束。")

    try:
        while True:
            tags, frame = detector.detect()
            if frame is not None:
                frame = detector.draw_tags(frame, tags)

                if tags:
                    for tag in tags:
                        print(f"  偵測到：{tag}")

                cv2.imshow("AprilTag Test", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    except KeyboardInterrupt:
        print("\n[測試] 結束。")
    finally:
        detector.release()
        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(
        description="AprilTag 追蹤小車",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--preview", action="store_true",
        help="開啟即時預覽視窗（需要連接螢幕或 VNC）",
    )
    parser.add_argument(
        "--tag-id", type=int, default=None,
        help="只追蹤指定 ID 的 AprilTag（預設：追蹤最大的 tag）",
    )
    parser.add_argument(
        "--speed", type=int, default=None,
        help=f"基礎前進速度 0–100（預設：{config.BASE_SPEED}）",
    )
    parser.add_argument(
        "--test-motors", action="store_true",
        help="執行馬達功能測試（不啟動攝影機）",
    )
    parser.add_argument(
        "--test-camera", action="store_true",
        help="執行攝影機偵測測試（不啟動馬達）",
    )
    args = parser.parse_args()

    # ---- 馬達測試模式 ----
    if args.test_motors:
        print("=" * 40)
        print("  馬達功能測試")
        print("=" * 40)
        test_motors()
        return

    # ---- 攝影機測試模式 ----
    if args.test_camera:
        print("=" * 40)
        print("  攝影機 AprilTag 偵測測試")
        print("=" * 40)
        test_camera()
        return

    # ---- 覆蓋設定 ----
    if args.tag_id is not None:
        config.TARGET_TAG_ID = args.tag_id
        print(f"[設定] 追蹤 Tag ID: {args.tag_id}")

    if args.speed is not None:
        config.BASE_SPEED = max(0, min(100, args.speed))
        print(f"[設定] 基礎速度: {config.BASE_SPEED}")

    # ---- 初始化模組 ----
    print("=" * 40)
    print("  AprilTag 追蹤小車")
    print("=" * 40)

    from motor_controller import MotorController
    from apriltag_detector import AprilTagDetector
    from tracker import Tracker

    motor_ctrl = None
    detector = None

    try:
        motor_ctrl = MotorController()
        detector = AprilTagDetector()
        tracker = Tracker(detector, motor_ctrl)

        print()
        print("系統就緒！開始追蹤...")
        print()

        tracker.run(preview=args.preview)

    except Exception as e:
        print(f"\n❌ 錯誤：{e}", file=sys.stderr)
        import traceback
        traceback.print_exc()

    finally:
        print("\n正在清理資源...")
        if detector:
            detector.release()
        if motor_ctrl:
            motor_ctrl.cleanup()
        print("程式結束。")


if __name__ == "__main__":
    main()
