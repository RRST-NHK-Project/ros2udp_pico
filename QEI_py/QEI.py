from machine import Pin
import time

# GPIOピンの設定
pin_a = Pin(14, Pin.IN, Pin.PULL_UP)  # A相
pin_b = Pin(15, Pin.IN, Pin.PULL_UP)  # B相

# 初期値
position = 0  # エンコーダーの位置カウント
PPR = 1024  # エンコーダーのパルス数（1回転あたりのパルス数）
angle_per_pulse = 360 / PPR  # 1パルスごとの角度

# 前回のA相の状態
last_a = pin_a.value()

# 割り込みハンドラ
def update_position(pin):
    global position, last_a
    current_a = pin_a.value()
    current_b = pin_b.value()

    # エンコーダーの回転方向を判定
    if current_a != last_a:  # 状態が変化した場合
        if current_a == current_b:
            position += 1  # 時計回り
        else:
            position -= 1  # 反時計回り
        last_a = current_a

# 割り込みの設定
pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=update_position)

# メインループ
try:
    while True:
        # 現在の角度を計算（正規化なし）
        angle = position * angle_per_pulse
        print(f"Position: {position}, Angle: {angle:.2f}")
        time.sleep(0.1)  # 100ms待機
except KeyboardInterrupt:
    print("Exit")