import os
import cv2
import numpy as np
import argparse
import config

# 強制將畫面輸出到本機端螢幕
if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0"

def main():
    parser = argparse.ArgumentParser(description="相機內參校正工具 (Camera Calibration Tool - Visual)")
    parser.add_argument("--rows", type=int, default=6, help="棋盤格內角點行數 (預設: 6)")
    parser.add_argument("--cols", type=int, default=9, help="棋盤格內角點列數 (預設: 9)")
    parser.add_argument("--size", type=float, default=0.015, help="棋盤格單格邊長(公尺) (預設: 0.015 即 15mm)")
    parser.add_argument("--output", default="camera_params.npz", help="校正結果輸出檔名")
    parser.add_argument("--auto", action="store_true", help="開啟自動連拍模式 (偵測到棋盤格即快速擷取)")
    parser.add_argument("--count", type=int, default=100, help="自動模式下的擷取目標張數 (預設: 100)")
    args = parser.parse_args()

    CHECKERBOARD = (args.cols, args.rows)
    # 終止條件
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # 建立 3D 空間點座標 (0,0,0), (1,0,0), (2,0,0) ...
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp = objp * args.size

    objpoints = [] # 真實世界 3D 點
    imgpoints = [] # 影像中 2D 點

    cap = cv2.VideoCapture(config.CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)

    if not cap.isOpened():
        print("無法開啟攝影機，請檢查 config.CAMERA_INDEX。")
        return

    import time

    print("=== 可視化相機校正工具 ===")
    print(f"尋找 {args.cols}x{args.rows} 棋盤格 (單格 {args.size}m)...")
    print("已強制設定 DISPLAY=:0 將視窗輸出到 Raspberry Pi 外接螢幕 (或 VNC)。")
    if args.auto:
        print(f"【高速連拍模式】開啟！當偵測到棋盤格時，會每 0.3 秒自動擷取，共拍 {args.count} 張。")
        print("操作說明：請將棋盤格放在鏡頭前，四處移動（畫面的綠色軌跡代表「已拍過的位置」）。")
        print("提早結束請按 [Enter] 或 [q]。")
    else:
        print("【手動擷取模式】")
        print("操作說明：")
        print("  - [空白鍵] 或 [c]: 若有偵測到棋盤格，則擷取當前畫面加入校正集")
        print("  - [Enter] 或 [q]: 結束擷取並開始計算相機參數")
        print("  (如需自動擷取，可加 --auto 參數)")
    print("==========================")

    captured_count = 0
    last_capture_time = time.time()
    
    # 用來記錄已經拍過的棋盤格中心點，畫出「走過的軌跡」
    history_centers = []

    while True:
        ret, frame = cap.read()
        if not ret:
            print("無法讀取攝影機畫面")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 尋找棋盤格
        ret_chess, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, 
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        display_frame = frame.copy()

        if ret_chess:
            # 畫出角點
            cv2.drawChessboardCorners(display_frame, CHECKERBOARD, corners, ret_chess)
            if args.auto:
                cv2.putText(display_frame, "Auto Capture active! Keep moving slowly.", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(display_frame, "Checkerboard Detected! Press SPACE.", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, "No checkerboard detected.", 
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.putText(display_frame, f"Captured: {captured_count}" + (f" / {args.count}" if args.auto else ""), 
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # 畫出走過的軌跡（歷史中心點）
        for pt in history_centers:
            cv2.circle(display_frame, pt, 5, (255, 0, 0), -1)

        cv2.imshow('Camera Calibration', display_frame)
        key = cv2.waitKey(1) & 0xFF

        # 自動連拍邏輯 (每 0.3 秒擷取一次)
        current_time = time.time()
        if args.auto and ret_chess and (current_time - last_capture_time > 0.3):
            # 亞像素級角點優化
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners2)
            captured_count += 1
            
            # 計算這次棋盤格的中心點並加入軌跡
            center_x = int(np.mean(corners2[:, 0, 0]))
            center_y = int(np.mean(corners2[:, 0, 1]))
            history_centers.append((center_x, center_y))

            print(f"[{captured_count}/{args.count}] 自動擷取影像")
            
            # 視覺回饋（閃爍綠框）
            cv2.rectangle(display_frame, (0,0), (display_frame.shape[1], display_frame.shape[0]), (0,255,0), 6)
            cv2.imshow('Camera Calibration', display_frame)
            cv2.waitKey(200)
            
            last_capture_time = time.time()
            if captured_count >= args.count:
                print("達到目標張數，開始計算...")
                break

        # 手動控制邏輯
        if key == ord(' ') or key == ord('c'):
            if not args.auto:
                if ret_chess:
                    # 亞像素級角點優化
                    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                    objpoints.append(objp)
                    imgpoints.append(corners2)
                    captured_count += 1
                    print(f"已擷取 {captured_count} 張影像")
                    
                    # 視覺回饋（閃爍）
                    cv2.rectangle(display_frame, (0,0), (display_frame.shape[1], display_frame.shape[0]), (255,255,255), -1)
                    cv2.imshow('Camera Calibration', display_frame)
                    cv2.waitKey(100)
                else:
                    print("未偵測到完整的棋盤格，無法擷取。")
        elif key == 13 or key == ord('q') or key == 27: # Enter, q, or Esc
            break

    cv2.destroyAllWindows()
    cap.release()

    if captured_count < 15:
        print(f"\n擷取的影像為 ({captured_count} 張)。建議至少需要 15 張以上才能取得較好的校正結果。")
        print("仍會為您嘗試計算，但精確度可能較差。")
        return

    print("開始計算相機參數，請稍候...")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    if ret:
        print("\n✅ 校正成功！")
        print("相機矩陣 (Camera Matrix):")
        print(mtx)
        print("失真係數 (Distortion Coefficients):")
        print(dist)
        
        np.savez(args.output, mtx=mtx, dist=dist)
        print(f"\n結果已儲存至 {args.output}")
        
        fx, fy = mtx[0, 0], mtx[1, 1]
        cx, cy = mtx[0, 2], mtx[1, 2]
        print(f"\n這表示您的相機參數為：\n[fx, fy, cx, cy] = [{fx:.2f}, {fy:.2f}, {cx:.2f}, {cy:.2f}]")
        print("之後可以將此參數填入 config.py 中以啟用 3D Pose 估計。")
    else:
        print("❌ 校正失敗。")

if __name__ == "__main__":
    main()
