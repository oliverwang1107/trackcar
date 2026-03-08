# 🚗 AprilTag 追蹤小車

基於 **Raspberry Pi 5** 的 AprilTag 自動追蹤小車。使用 USB 攝影機偵測 AprilTag 標籤，透過 PID 控制器實現平滑的跟隨行為。

## 系統架構

```
USB 攝影機 → AprilTag 偵測 → PID 追蹤控制器 → TB6612FNG → 雙 TT 馬達
```

## 硬體需求

| 零件 | 規格 |
|:---|:---|
| Raspberry Pi 5 | 4GB / 8GB |
| USB 攝影機 | 720p 30fps（廣角佳） |
| TB6612FNG 驅動模組 | 雙頻道 1.2A |
| TT 減速馬達 + 輪子 | 1:48 × 2 |
| 2WD 小車底盤 | 含萬向輪 |
| 18650 鋰電池 | 3.7V × 2（串聯 7.4V） |
| RPi5 行動電源 | 5V 5A PD |

📎 詳細接線請參考 [docs/wiring_guide.md](docs/wiring_guide.md)

## 快速開始

### 1. 安裝

```bash
# 在 Raspberry Pi 5 上執行
git clone <repo-url> trackcar
cd trackcar
chmod +x install.sh
sudo ./install.sh

# 啟用虛擬環境
source venv/bin/activate
```

### 2. 列印 AprilTag

到 [AprilTag 產生器](https://chev.me/arucogen/) 列印 `tag36h11` 系列的標籤，建議邊長 8cm 以上。

### 3. 測試硬體

```bash
# 測試馬達
python3 main.py --test-motors

# 測試攝影機 + AprilTag 偵測
python3 main.py --test-camera
```

### 4. 開始追蹤

```bash
# 基本模式
python3 main.py

# 開啟預覽視窗（需螢幕或 VNC）
python3 main.py --preview

# 只追蹤特定 tag
python3 main.py --preview --tag-id 3

# 調整基礎速度
python3 main.py --speed 50
```

## 參數調整

所有參數集中在 `config.py`，主要調整項目：

| 參數 | 說明 | 預設值 |
|:---|:---|:---:|
| `BASE_SPEED` | 基礎前進速度 | 40 |
| `STEERING_PID["Kp"]` | 轉向靈敏度 | 0.5 |
| `TARGET_AREA_RATIO` | 目標距離（面積比） | 0.08 |
| `LOST_FRAME_THRESHOLD` | 遺失幀門檻 | 15 |
| `SEARCH_SPEED` | 搜尋旋轉速度 | 30 |

## 專案結構

```
trackcar/
├── main.py                # 主程式入口
├── config.py              # 集中配置
├── motor_controller.py    # 馬達控制模組
├── apriltag_detector.py   # AprilTag 偵測模組
├── tracker.py             # PID 追蹤控制
├── requirements.txt       # Python 依賴
├── install.sh             # 安裝腳本
├── docs/
│   └── wiring_guide.md    # 接線指南
└── README.md
```

## 故障排除

| 問題 | 檢查項目 |
|:---|:---|
| 馬達不轉 | 1. 電池是否有電？ 2. STBY 是否接 GPIO24？ 3. 共地是否連接？ |
| 馬達方向相反 | 交換該馬達的輸出接線（AO1/AO2 或 BO1/BO2） |
| 攝影機無法開啟 | 執行 `ls /dev/video*` 確認裝置，調整 `config.py` 的 `CAMERA_INDEX` |
| 偵測不到 tag | 1. 確認 tag 已列印且為 `tag36h11` 系列 2. 確認光線充足 3. tag 距離不要太遠 |
| 追蹤不穩定 | 調低 `STEERING_PID["Kp"]`，調高 `STEERING_PID["Kd"]` |

## 授權

MIT License
