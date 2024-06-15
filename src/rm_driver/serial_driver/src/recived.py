import serial

# 创建一个串口对象
ser = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)

while True:
    # 读取一行数据
    line = ser.readline()
    # 如果接收到数据，打印出来
    if line:
        print("Received bytes: ", end="")
        for byte in line:
            print(f"{byte:02x} ", end="")
        print()