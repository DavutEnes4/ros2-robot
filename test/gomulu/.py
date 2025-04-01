import pygame
import sys
import os
import urllib.request
import math
import random

# Dosya indirme fonksiyonu: Dosya mevcut değilse URL’den indirir.
def download_asset(url, filename):
    if not os.path.exists(filename):
        print(f"{filename} indiriliyor...")
        urllib.request.urlretrieve(url, filename)

# Varlıklar (örnek URL’ler; lisans durumlarını kontrol edin!)
assets = {
    "background": {
        "url": "https://opengameart.org/sites/default/files/forest_bg_highres.png",
        "filename": "background.png"
    },
    "player": {
        "url": "https://opengameart.org/sites/default/files/warrior_highres.png",
        "filename": "player.png"
    },
    "enemy": {
        "url": "https://opengameart.org/sites/default/files/evil_knight_highres.png",
        "filename": "enemy.png"
    },
    "attack_sound": {
        "url": "https://freesound.org/people/joshuaempyre/sounds/398086/download/398086__joshuaempyre__sword.wav",
        "filename": "attack.wav"
    }
}

# Varlıkları indir.
for key, asset in assets.items():
    download_asset(asset["url"], asset["filename"])

pygame.init()

WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Vampire Survivors Tarzı - Pygame")

# Görsellerin yüklenmesi
try:
    background_img = pygame.image.load("background.png")
    background_img = pygame.transform.scale(background_img, (WIDTH, HEIGHT))
except Exception as e:
    print("Arka plan görseli yüklenemedi:", e)
    background_img = pygame.Surface((WIDTH, HEIGHT))
    background_img.fill((50, 50, 50))

try:
    player_img = pygame.image.load("player.png")
    player_img = pygame.transform.scale(player_img, (70, 90))
except Exception as e:
    print("Oyuncu görseli yüklenemedi:", e)
    player_img = pygame.Surface((70, 90))
    player_img.fill((0, 255, 0))

try:
    enemy_img = pygame.image.load("enemy.png")
    enemy_img = pygame.transform.scale(enemy_img, (50, 50))
except Exception as e:
    print("Düşman görseli yüklenemedi:", e)
    enemy_img = pygame.Surface((50, 50))
    enemy_img.fill((255, 0, 0))

try:
    attack_sound = pygame.mixer.Sound("attack.wav")
except Exception as e:
    print("Saldırı sesi yüklenemedi:", e)
    attack_sound = None

clock = pygame.time.Clock()
font = pygame.font.SysFont("Arial", 20)

# Oyuncu parametreleri
player_rect = player_img.get_rect(center=(WIDTH // 2, HEIGHT // 2))
player_health = 100
player_damage = 10
attack_radius = 100  # Otomatik saldırı alanı
auto_attack_interval = 500  # milisaniye cinsinden saldırı aralığı
last_attack_time = 0

score = 0

# Düşman sınıfı: Her düşman oyuncuya doğru hareket eder.
class Enemy:
    def __init__(self, image, x, y):
        self.image = image
        self.rect = image.get_rect(center=(x, y))
        self.health = 30
        self.speed = random.uniform(1.5, 2.5)
    def update(self, target_rect):
        dx = target_rect.centerx - self.rect.centerx
        dy = target_rect.centery - self.rect.centery
        dist = math.hypot(dx, dy)
        if dist != 0:
            dx /= dist
            dy /= dist
        self.rect.x += dx * self.speed
        self.rect.y += dy * self.speed

enemies = []

# Düşman oluşturma ayarları
enemy_spawn_interval = 1000  # Her 1 saniyede bir düşman oluştur
last_spawn_time = pygame.time.get_ticks()

# Rastgele kenardan düşman oluşturma fonksiyonu
def spawn_enemy():
    side = random.choice(["top", "bottom", "left", "right"])
    if side == "top":
        x = random.randint(0, WIDTH)
        y = 0
    elif side == "bottom":
        x = random.randint(0, WIDTH)
        y = HEIGHT
    elif side == "left":
        x = 0
        y = random.randint(0, HEIGHT)
    else:
        x = WIDTH
        y = random.randint(0, HEIGHT)
    enemies.append(Enemy(enemy_img, x, y))

# Sağlık barı çizme fonksiyonu (oyuncu için)
def draw_health_bar(surface, x, y, health, max_health):
    bar_length = 200
    bar_height = 20
    fill = (health / max_health) * bar_length
    outline_rect = pygame.Rect(x, y, bar_length, bar_height)
    fill_rect = pygame.Rect(x, y, fill, bar_height)
    pygame.draw.rect(surface, (255, 0, 0), fill_rect)
    pygame.draw.rect(surface, (255, 255, 255), outline_rect, 2)

running = True
while running:
    dt = clock.tick(60)
    current_time = pygame.time.get_ticks()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Oyuncu hareketi (ok tuşları)
    keys = pygame.key.get_pressed()
    player_speed = 5
    if keys[pygame.K_LEFT]:
        player_rect.x -= player_speed
    if keys[pygame.K_RIGHT]:
        player_rect.x += player_speed
    if keys[pygame.K_UP]:
        player_rect.y -= player_speed
    if keys[pygame.K_DOWN]:
        player_rect.y += player_speed

    # Otomatik saldırı: Belirli aralıklarla oyuncunun etrafındaki düşmanlara hasar verilir.
    if current_time - last_attack_time >= auto_attack_interval:
        if attack_sound:
            attack_sound.play()
        for enemy in enemies:
            enemy_center = enemy.rect.center
            player_center = player_rect.center
            dist = math.hypot(enemy_center[0] - player_center[0], enemy_center[1] - player_center[1])
            if dist <= attack_radius:
                enemy.health -= player_damage
        last_attack_time = current_time

    # Düşmanları güncelle: Oyuncuya doğru hareket ve çarpışma kontrolü.
    for enemy in enemies[:]:
        enemy.update(player_rect)
        # Eğer düşman oyuncuya değerse, oyuncuya hasar ver ve düşmanı listeden çıkar.
        if enemy.rect.colliderect(player_rect):
            player_health -= 5
            enemies.remove(enemy)
        # Düşmanın sağlığı sıfırlandıysa, düşmanı kaldır ve skora ekle.
        elif enemy.health <= 0:
            enemies.remove(enemy)
            score += 10

    # Belirli aralıklarla yeni düşman oluştur.
    if current_time - last_spawn_time >= enemy_spawn_interval:
        spawn_enemy()
        last_spawn_time = current_time

    # Ekran çizimleri
    screen.blit(background_img, (0, 0))
    screen.blit(player_img, player_rect)
    for enemy in enemies:
        screen.blit(enemy.image, enemy.rect)
    # Saldırı efektini görsel olarak göstermek için oyuncu etrafında kısa süreli bir daire çizilir.
    if current_time - last_attack_time < 100:
        pygame.draw.circle(screen, (255, 255, 0), player_rect.center, attack_radius, 2)
    
    # Oyuncu sağlık barı ve skorun çizilmesi
    draw_health_bar(screen, 20, 20, player_health, 100)
    score_text = font.render(f"Score: {score}", True, (255, 255, 255))
    screen.blit(score_text, (20, 50))
    
    pygame.display.flip()

    # Oyuncu sağlık kontrolü: Sağlık 0’ın altına düşerse oyun biter.
    if player_health <= 0:
        running = False
        screen.fill((0, 0, 0))
        game_over_text = font.render("Oyun Bitti! Yenildiniz.", True, (255, 0, 0))
        screen.blit(game_over_text, (WIDTH // 2 - game_over_text.get_width() // 2, HEIGHT // 2))
        pygame.display.flip()
        pygame.time.delay(2000)

pygame.quit()
sys.exit()
