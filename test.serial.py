import serial
import time


class SerialReceiver:
    def receive_list_data(self, serial_port):
        try:
            # 打开串口
            ser = serial.Serial(serial_port, 9600)
            time.sleep(0.5)  # 稍作延迟，确保数据已发送完毕

            # 从串口读取数据
            data_str = ser.read(ser.in_waiting).decode('utf-8')  # 读取串口缓冲区中的数据
            if data_str:
                print(f"Received: {data_str}")
                # 将接收到的字符串按逗号分割成列表
                data_list = data_str.strip().split(',')
                # 将每个字符串转换回整数或浮点数（根据数据类型需求）
                data_list = [float(item) if '.' in item else int(item) for item in data_list]
                print(f"Processed Data: {data_list}")
            else:
                print("No data received.")
            
            # 关闭串口
            ser.close()
        except serial.SerialException as e:
            print(f"串口错误: {e}")

# 使用示例
receiver = SerialReceiver()
receiver.receive_list_data("/dev/ttyUSB0")  # 根据实际串口号修改

