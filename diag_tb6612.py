#!/usr/bin/env python3
"""
TB6612FNG 低階診斷腳本 (Low-level Hardware Diagnostic)

目的：排除 MotorController 類別的抽象干擾，直接使用 gpiozero 操作腳位。
用於檢查：電源、STBY、GND、腳位映射。
"""

import time
import sys
from gpiozero import DigitalOutputDevice, PWMOutputDevice

# 根據 config.py 的配置
STBY_PIN = 24
AIN1 = 17
AIN2 = 27
PWMA = 12
BIN1 = 22
BIN2 = 23
PWMB = 13

def run_diag():
    print("--- 🚀 開始 TB6612FNG 硬體診斷 ---")
    
    try:
        # 1. 初始化 STBY
        print(f"1. 啟用 STBY (GPIO {STBY_PIN})...")
        stby = DigitalOutputDevice(STBY_PIN, initial_value=True)
        time.sleep(0.5)

        # 2. 測試左馬達 (A)
        print(f"2. 測試左馬達 (A) [IN1:{AIN1}, IN2:{AIN2}, PWM:{PWMA}]")
        in1_a = DigitalOutputDevice(AIN1, initial_value=False)
        in2_a = DigitalOutputDevice(AIN2, initial_value=False)
        pwm_a = PWMOutputDevice(PWMA, initial_value=0)
        
        print("   - 設定方向 (Forward)...")
        in1_a.on()
        in2_a.off()
        print("   - 設定 PWM (100% Speed)...")
        pwm_a.value = 1.0 # 全速
        
        print("   >>> 觀察左馬達是否有轉動？ (持續 3 秒)...")
        time.sleep(3)
        
        print("   - 停止左馬達...")
        pwm_a.value = 0
        in1_a.off()
        in2_a.off()
        
        time.sleep(1)

        # 3. 測試右馬達 (B)
        print(f"3. 測試右馬達 (B) [IN1:{BIN1}, IN2:{BIN2}, PWM:{PWMB}]")
        in1_b = DigitalOutputDevice(BIN1, initial_value=False)
        in2_b = DigitalOutputDevice(BIN2, initial_value=False)
        pwm_b = PWMOutputDevice(PWMB, initial_value=0)
        
        print("   - 設定方向 (Forward)...")
        in1_b.on()
        in2_b.off()
        print("   - 設定 PWM (100% Speed)...")
        pwm_b.value = 1.0 # 全速
        
        print("   >>> 觀察右馬達是否有轉動？ (持續 3 秒)...")
        time.sleep(3)
        
        print("   - 停止右馬達...")
        pwm_b.value = 0
        in1_b.off()
        in2_b.off()

    except Exception as e:
        print(f"\n❌ 診斷過程發生錯誤: {e}")
    finally:
        print("\n4. 釋放資源並結束...")
        # 確保結束時 STBY 也是釋放的，防止硬體持續輸出
        if 'stby' in locals(): stby.close()
        if 'in1_a' in locals(): in1_a.close()
        if 'in2_a' in locals(): in2_a.close()
        if 'pwm_a' in locals(): pwm_a.close()
        if 'in1_b' in locals(): in1_b.close()
        if 'in2_b' in locals(): in2_b.close()
        if 'pwm_b' in locals(): pwm_b.close()
        print("--- 🏁 診斷結束 ---")

if __name__ == "__main__":
    run_diag()
