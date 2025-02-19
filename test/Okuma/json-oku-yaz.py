import json
import serial
import time

# Seri portu açmaya çalış
try:
    ser = serial.Serial('/dev/ttyUSB2', 115200, timeout=1)
    print("Seri port bağlantısı başarılı!")
except serial.SerialException:
    print("Seri port açılırken hata oluştu!")
    exit()

# Sürekli veri okuma ve kayıt işlemi
while True:
    try:
        # JSON dosyasını oku (hata önleme)
        try:
            with open("veri.json", "r", encoding="utf-8") as f:
                veri = json.load(f)
        except (json.JSONDecodeError, FileNotFoundError):
            veri = {}  # JSON bozuksa veya yoksa boş veri başlat

        # Seri porttan veri oku
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').strip()
            print(f"Read from serial: {data}")

            # Eğer gelen veri "veri" ise JSON'u seri porta yaz
            if data == "veri":
                json_data = json.dumps(veri)
                ser.write(json_data.encode('utf-8'))
                print(f"Write to serial: {json_data}")

            # Gelen veriyi log.json'a ekleyerek kaydet
            with open("log.json", "a", encoding="utf-8") as log_file:
                json.dump({"received": data, "timestamp": time.time()}, log_file)
                log_file.write("\n")  # Satır sonu ekleyerek düzenli kayıt yap

        # Dosyayı sürekli yeniden yazmak yerine değişiklik varsa güncelle
        time.sleep(0.1)  # CPU kullanımını azaltmak için küçük bekleme süresi

    except KeyboardInterrupt:
        print("\nProgram sonlandırıldı.")
        break
    except Exception as e:
        print(f"Hata oluştu: {e}")
