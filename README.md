# 🚗 AprilTag 追蹤小車：完整操作指南

這是一個基於 **Raspberry Pi 5** 的自動追蹤小車專案。本文件將詳細說明專案中各個程式的功能與操作方式。

---

## 📂 程式目錄索引

| 類別 | 檔案名稱 | 功能說明 |
| :--- | :--- | :--- |
| **核心** | `main.py` | 專案主程式：執行自動追蹤功能 |
| **測試** | `test_motor.py` | 自動化馬達功能測試（循環移動） |
| **測試** | `test_motor_interactive.py` | 互動式馬達測試（手動按鍵控制，適合量測電壓） |
| **診斷** | `diag_tb6612.py` | 底層硬體診斷（排除軟體抽象，直接測試 GPIO） |
| **教學/進階** | `calibrate_camera.py` | 相機內參校正工具（需準備棋盤格零件） |
| **視覺化** | `visualize_3d.py` | 3D 姿態 AR 演示（標籤上渲染 3D 座標軸與立方體） |
| **視覺化** | `simulate_space.py` | 3D 空間模擬器（使用 matplotlib 繪製即時相對位置） |

---

## 🚀 1. 核心程式 (Core)

### `main.py` - 自動追蹤主程式
這是小車日常運行的主要程式。它會啟動攝影機，尋找 AprilTag，並驅動馬達跟隨。

*   **基本啟動：**
    ```bash
    python3 main.py
    ```
    *(備註：若有連接螢幕或使用 VNC，**預設會自動開啟預覽視窗**。)*
*   **關閉預覽：**
    ```bash
    python3 main.py --no-preview
    ```
*   **測試模式：**
    *   `--test-motors`: 只測試馬達方向是否正確。
    *   `--test-camera`: 只打開攝影機測試偵測效果。

---

## 🔧 2. 硬體測試與診斷 (Hardware & Debug)

### `test_motor.py` - 自動測試
讓小車依序執行「前進、後退、左轉、右轉、停止」。用於快速確認馬達線路是否正確。
```bash
python3 test_motor.py
```

### `test_motor_interactive.py` - 手動互動測試
如果你懷疑馬達力道不足或沒反應，可以使用此程式手動給予訊號。
*   **指令：**
    *   `f`: 前進 / `b`: 後退 / `l`: 左轉 / `r`: 右轉
    *   `s`: 停止 / `q`: 退出
```bash
python3 test_motor_interactive.py
```

### `diag_tb6612.py` - 底層硬體診斷
當 `MotorController` 無法運作時，使用此腳本。它不依賴任何專案邏輯，直接操作 GPIO 腳位。如果連這個都沒有反應，代表硬體連接或驅動板供電有問題。
```bash
python3 diag_tb6612.py
```

---

## 📷 3. 相機校正 (Calibration)

為了讓小車能精準知道標籤的距離與角度（3D 姿態），需要進行相機校正。

### `calibrate_camera.py`
1.  準備一張 A4 列印的棋盤格 (Checkerboard)。
2.  執行程式：
    ```bash
    python3 calibrate_camera.py --auto
    ```
3.  在鏡頭前變換角度移動棋盤格，直到拍滿 100 張影像。
4.  程式會自動產生 `camera_params.npz`。

---

## 🌈 4. AR 與 3D 可視化 (Advanced)

*這些功能需要在完成相機校正後（取得 `camera_params.npz`）才能運行。*

### `visualize_3d.py` - AR 增廣實境
在攝影機畫面中，直接在 AprilTag 上方繪製 3D 座標軸與一個紅色的虛擬立方體。
```bash
python3 visualize_3d.py
```

### `simulate_space.py` - 虛擬空間模擬
打開一個 matplotlib 3D 圖表，以相機為中心 (0,0,0)，觀察標籤在三維空間中移動的軌跡與姿態。
```bash
python3 simulate_space.py
```

---

## ⚙️ 設定檔說明 (`config.py`)

如果你更換了腳位或攝影機，請修改此檔案：
*   `GPIO_PINS`: 修改馬達訊號腳位。
*   `CAMERA_INDEX`: 預設為 `0`。
*   `DISTANCE_THRESHOLD`: 小車與標籤的目標維持距離。
*   `STEERING_PID`: 調整小車轉彎的靈敏度。

---

## 🛠 安裝與準備
1.  執行 `sudo ./install.sh` 安裝依賴。
2.  務必進入虛擬環境：`source venv/bin/activate`。
3.  下載 AprilTag 標籤 (tag36h11 家族)。

