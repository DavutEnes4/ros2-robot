import socket
import time

# Bağlantı kur
tello_ip = '192.168.10.1'
tello_port = 8889
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 9000))

# Komut moduna geç
sock.sendto('command'.encode('utf-8'), (tello_ip, tello_port))
print("Komut gönderildi, yanıt bekleniyor...")
response, ip = sock.recvfrom(100)
print(f"Yanıt: {response.decode('utf-8')}")

# WiFi ağınıza bağlan (ap komutu)
#
# wifi_cmd = 'ap raspi raspi1234'
wifi_cmd = 'takeoff'
sock.sendto(wifi_cmd.encode('utf-8'), (tello_ip, tello_port))
print("WiFi komutu gönderildi, yanıt bekleniyor...")

try:
    response, ip = sock.recvfrom(100)
    print(f"Yanıt: {response.decode('utf-8')}")
except:
    print("Yanıt alınamadı, drone yeniden başlıyor olabilir")