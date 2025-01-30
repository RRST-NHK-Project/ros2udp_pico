import machine
from machine import Pin
from machine import PWM
from utime import sleep
import network
import usocket

SSID = "GL-SFT1200-288-2G-LNX"
PASSWORD = "lnxmaster"
SRC_IP = "192.168.8.219"
SRC_PORT = 5000
SUBNET_MASK = "255.255.255.0"
GATEWAY = "192.168.8.1"
DNS = "192.168.8.1"

MD1P = PWM(machine.Pin(0, machine.Pin.OUT))
MD2P = PWM(machine.Pin(1, machine.Pin.OUT))
MD3P = PWM(machine.Pin(2, machine.Pin.OUT))
MD4P = PWM(machine.Pin(3, machine.Pin.OUT))
MD5P = PWM(machine.Pin(22, machine.Pin.OUT))
MD6P = PWM(machine.Pin(26, machine.Pin.OUT))

MD1D = Pin(6, Pin.OUT)
MD2D = Pin(7, Pin.OUT)
MD3D = Pin(8, Pin.OUT)
MD4D = Pin(9, Pin.OUT)
MD5D = Pin(27, Pin.OUT)
MD6D = Pin(28, Pin.OUT)

# サーボ
SERVO1 = PWM(machine.Pin(10, machine.Pin.OUT))
SERVO2 = PWM(machine.Pin(11, machine.Pin.OUT))
SERVO3 = PWM(machine.Pin(12, machine.Pin.OUT))
SERVO4 = PWM(machine.Pin(13, machine.Pin.OUT))

"""
# 電磁弁
SV1 = Pin(10, Pin.OUT)
SV2 = Pin(11, Pin.OUT)
SV3 = Pin(12, Pin.OUT)
SV4 = Pin(13, Pin.OUT) 
"""

# エンコーダー
ENC1_A = Pin(14, Pin.IN, Pin.PULL_UP)  # A相
ENC1_B = Pin(15, Pin.IN, Pin.PULL_UP)  # B相
ENC2_A = Pin(16, Pin.IN, Pin.PULL_UP)  # A相
ENC2_B = Pin(17, Pin.IN, Pin.PULL_UP)  # B相
ENC3_A = Pin(18, Pin.IN, Pin.PULL_UP)  # A相
ENC3_B = Pin(19, Pin.IN, Pin.PULL_UP)  # B相
ENC4_A = Pin(20, Pin.IN, Pin.PULL_UP)  # A相
ENC4_B = Pin(21, Pin.IN, Pin.PULL_UP)  # B相


MD1P.freq(2000)  # 20kHz
MD2P.freq(2000)  # 20kHz
MD3P.freq(2000)  # 20kHz
MD4P.freq(2000)  # 20kHz
MD5P.freq(2000)  # 20kHz
MD6P.freq(2000)  # 20kHz

SERVO1.freq(300)  # 300Hz
SERVO2.freq(300)  # 300Hz
SERVO3.freq(300)  # 300Hz
SERVO4.freq(300)  # 300Hz

data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

led = Pin("LED", Pin.OUT)  # LED

wlan = network.WLAN(network.STA_IF)  # wlan作成（無線LAN）
wlan.active(True)  # 起動
wlan.ifconfig((SRC_IP, SUBNET_MASK, GATEWAY, DNS))
wlan.connect(SSID, PASSWORD)  # アクセスポイントに接続
while not wlan.isconnected():  # 接続待ち
    print(".", end="")
    led.toggle()
    sleep(0.1)
print("\nConnected.\nIP:" + wlan.ifconfig()[0])

sock = usocket.socket(usocket.AF_INET, usocket.SOCK_DGRAM)  # ソケット作成
sock.bind((SRC_IP, SRC_PORT))  # バインド

try:
    while True:
        # データを受信
        buffer, addr = sock.recvfrom(128)
        buffer = buffer.decode("utf-8")
        str_data = buffer.split(",")
        data = list(map(int, str_data))
        led.toggle()
        
        print(data)

        servo_duty = int(
            (data[7] * (2500 - 500) / 270 + 500) * 65535 / 2500
        )  # 500〜2500に変換

        SERVO1.duty_u16(servo_duty)
        SERVO2.duty_u16(servo_duty)
        SERVO3.duty_u16(servo_duty)
        SERVO4.duty_u16(servo_duty)

        """
        SV1.value(data[11])
        SV2.value(data[12])
        SV3.value(data[13])
        SV4.value(data[14])
        """
        mdp = [0, 0, 0, 0, 0, 0, 0]
        mdd = [0, 0, 0, 0, 0, 0, 0]
        
        mdd = [1 if data[i] >= 0 else 0 for i in range(1, 8)]
        mdp = [min(1, abs(data[i]) / 100) for i in range(1, 8)]
        
        MD1P.duty_u16(int(mdp[1] * 65535))
        MD2P.duty_u16(int(mdp[2] * 65535))
        MD3P.duty_u16(int(mdp[3] * 65535))
        MD4P.duty_u16(int(mdp[4] * 65535))
        MD5P.duty_u16(int(mdp[5] * 65535))
        MD6P.duty_u16(int(mdp[6] * 65535))

        MD1D.value(mdd[1])
        MD2D.value(mdd[2])
        MD3D.value(mdd[3])
        MD4D.value(mdd[4])
        MD5D.value(mdd[5])
        MD6D.value(mdd[6])

except KeyboardInterrupt:
    print("Server stopped")
finally:
    sock.close()
