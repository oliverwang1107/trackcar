# 🚗 AprilTag 追蹤小車：從零開始教學

這是一個基於 **Raspberry Pi 5** 的自動追蹤小車專案。它能透過攝影機「看見」指定的標籤（AprilTag），並自動跟著它移動。

本教學專為**完全沒有經驗**的使用者設計，請按照以下步驟逐步操作。

---

## 🛠 硬體準備清單

在使用軟體之前，你需要準備好以下硬體：

1.  **Raspberry Pi 5** (建議 4GB 以上)
2.  **USB 攝影機** (一般的電腦 WebCam 即可)
3.  **TB6612FNG 馬達驅動板**
4.  **TT 減速馬達** x2 + 輪子
5.  **小車底盤**
6.  **電池**：建議使用兩顆 18650 鋰電池 (約 7.4V) 幫馬達供電
7.  **行動電源**：建議 5V 5A (預留給 RPi 5 使用)

> [!TIP]
> 詳細的接線圖請參考：[docs/wiring_guide.md](docs/wiring_guide.md)

---

## 🏁 第一步：準備 Raspberry Pi 系統

如果你還沒設定好你的樹莓派，請按照以下步驟：

1.  在電腦安裝 [Raspberry Pi Imager](https://www.raspberrypi.com/software/)。
2.  選擇作業系統：`Raspberry Pi OS (64-bit)`。
3.  **重要建議**：點擊右下角齒輪（設定），開啟：
    *   **開啟 SSH**（這樣你才能遠端連線）。
    *   **設定帳號密碼**（預設通常建議使用者名稱為 `pi`）。
    *   **設定 Wi-Fi**（填入你手機或家裡的 Wi-Fi 名稱與密碼）。
4.  點擊「寫入」並等待完成。

---

## 🔗 第二步：連線到你的樹莓派 (RPI)

將記憶卡插回樹莓派並開機，等待約 1-2 分鐘讓它開機並連上 Wi-Fi。

1.  **找到 IP 地址**：
    *   你可以檢查無線路由器的後台。
    *   或者嘗試在終端機輸入：`ping raspberrypi.local`。
2.  **連線 (SSH)**：
    *   在 Windows 搜尋 `PowerShell` 或 Mac 打開 `終端機 (Terminal)`。
    *   輸入指令：`ssh pi@<你的IP地址>` (例如 `ssh pi@192.168.1.10` 或 `ssh pi@raspberrypi.local`)。
    *   出現輸入密碼提示，輸入你剛才設定的密碼（輸入時不會顯示字是正常的）。

---

## 📥 第三步：下載程式碼 (Git Clone)

連線成功後，你會看到綠色的文字提示。現在請輸入以下指令來下載這個專案：

```bash
# 下載專案
git clone https://github.com/oliverwang1107/trackcar.git

# 進入資料夾
cd trackcar
```

---

## 🛠 第四步：一鍵安裝環境

我們準備了一個自動安裝腳本，會幫你裝好所有需要的軟體與 Python 工具。

```bash
# 賦予執行權限
chmod +x install.sh

# 執行安裝 (會花幾分鐘，請耐心等待)
sudo ./install.sh

# 啟動 Python 虛擬環境 (每次重開機連線後都要做這一步)
source venv/bin/activate
```

---

## 🧪 第五步：硬體功能測試

在正式讓小車亂跑前，請務必先做以下測試：

1.  **馬達測試**（請把小車架空，避免它衝出去）：
    ```bash
    python3 main.py --test-motors
    ```
    *小車會依序嘗試前進、後退、左轉、右轉。如果方向反了，請交換馬達的接線位置。*

2.  **攝影機測試**：
    ```bash
    # 如果你有接螢幕或使用 VNC，可以看到畫面：
    python3 main.py --test-camera
    ```
    *如果沒接螢幕，程式偵測到標籤時也會在畫面上顯示文字資訊。*

---

## 🚀 第六步：啟動追蹤！

現在你可以拿著一張 **AprilTag (家族編號 tag36h11)** 放在攝影機前面。

```bash
# 基本啟動
python3 main.py

# 如果想要一邊看畫面一邊跑 (需螢幕/VNC)
python3 main.py --preview
```

### 如何取得 AprilTag 標籤？
請到 [這個網站](https://chev.me/arucogen/)：
*   Dictionary 選擇 `tag36h11`
*   列印出來（建議邊長 8 公分以上比較好偵測）

---

## ❓ 常見問題 Q&A

**Q: 馬達完全不會動？**
1. 檢查電池是否有電。
2. 檢查 `TB6612FNG` 驅動板上的 `STBY` 腳位是否有接到樹莓派的 GPIO 24。

**Q: 攝影機畫面全黑或是報錯？**
請輸入 `ls /dev/video*` 看看有沒有設備。通常預設是 0，如果不對，可以在 `config.py` 改 `CAMERA_INDEX`。

**Q: 小車轉向反應太慢或太激進？**
打開 `config.py` 調整 `STEERING_PID` 裡面的 `Kp` 值。數字越大反應越快，但太大會開始左右狂抖。

---

## 📜 授權
MIT License
