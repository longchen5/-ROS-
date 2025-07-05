import serial
import time

try:
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate=115200,
        timeout=1
    )
    print(f"成功打开串口: {ser.name}")
    
    # 测试发送
    ser.write(b'AT\r\n')  # 发送测试命令
    print("已发送: AT")
    
    # 等待并读取响应
    time.sleep(0.5)
    response = ser.read_all()
    if response:
        print(f"收到响应: {response.decode()}")
    else:
        print("未收到响应")
    
    ser.close()

except serial.SerialException as e:
    print(f"串口错误: {e}")
except Exception as e:
    print(f"其他错误: {e}")
