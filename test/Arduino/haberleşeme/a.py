import json
from time import sleep
from pySerialTransfer import pySerialTransfer as txfer
from dataclasses import dataclass


# Motor parametreleri için yapı
@dataclass
class Motor:
    enable_pin: int
    dir_pin: int
    pwm_pin: int
    step_period: int
    maxRadius: int
    maxFrequency: int
    lastStepTime: int
    stepInterval: int


# MPU sensörü için yapı
@dataclass
class Mpu:
    accel: dict
    gyro: dict
    mag: dict
    temp: float
    pressure: float


# GPS verileri için yapı
@dataclass
class Gps:
    lat: float
    lon: float
    alt: float
    speed: float
    sat: int


# Ana veri yapısı
@dataclass
class SensorData:
    mpu: Mpu
    # gps: Gps
    time: str
    motor: dict


if __name__ == "__main__":
    try:
        link = txfer.SerialTransfer("/dev/ttyUSB0")  # Bağlantı portunu belirtin
        link.open()
        sleep(5)

        while True:
            if link.available():  # Eğer veri varsa
                recSize = 0

                # Gelen JSON verisini byte dizisi olarak al
                json_bytes = link.rx_obj(obj_type='b', start_pos=recSize)  # 'b' byte dizisini alır
                # recSize += len(json_bytes)

                # Gelen byte dizisini string'e dönüştür
                json_str = bytes(json_bytes).decode('utf-8')

                print(f"Received JSON string: {json_str}")  # JSON string'ini yazdır

                # JSON string'ini Python JSON objesine dönüştür
                try:
                    json_data = json.loads(json_str)
                    print("Decoded JSON data:", json_data)  # JSON objesini yazdır

                    # JSON verisini SensorData yapısına dönüştür
                    mpu_data = Mpu(**json_data['mpu'])
                    # gps_data = Gps(**json_data['gps'])
                    motor_data = {key: Motor(**val) for key, val in json_data['motor'].items()}

                    # SensorData'yi oluştur
                    sensor_data = SensorData(
                        mpu=mpu_data,
                        # gps=gps_data,
                        time=json_data['time'],
                        motor=motor_data
                    )

                    print("Parsed Sensor Data:", sensor_data)

                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")

            elif link.status <= 0:  # Status kontrolü düzeltildi
                if link.status == txfer.Status.CRC_ERROR:
                    print("ERROR: CRC_ERROR")
                elif link.status == txfer.Status.PAYLOAD_ERROR:
                    print("ERROR: PAYLOAD_ERROR")
                elif link.status == txfer.Status.STOP_BYTE_ERROR:
                    print("ERROR: STOP_BYTE_ERROR")
                else:
                    print(f"ERROR: {link.status}")

    except KeyboardInterrupt:
        link.close()
