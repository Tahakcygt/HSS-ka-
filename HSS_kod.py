Python 3.12.10 (tags/v3.12.10:0cc8128, Apr  8 2025, 12:21:36) [MSC v.1943 64 bit (AMD64)] on win32
Enter "help" below or click "Help" above for more information.
>>> import json
... import math
... from dronekit import LocationGlobalRelative
... 
... # ---------------------------------------------------------
... # 1. YARDIMCI FONKSİYONLAR (Geometri & Dönüşüm)
... # ---------------------------------------------------------
... 
... def get_dist(p1, p2): 
...     return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
... 
... def normalize_angle(angle):
...     while angle > math.pi: angle -= 2 * math.pi
...     while angle < -math.pi: angle += 2 * math.pi
...     return angle
... 
... # Enlem/Boylam farkını Metreye (North, East) çevirir
... def get_relative_meters(home_loc, target_lat, target_lon):
...     earth_radius = 6378137.0 # Dünya yarıçapı (metre)
...     dLat = math.radians(target_lat - home_loc.lat)
...     dLon = math.radians(target_lon - home_loc.lon)
...     
...     north = dLat * earth_radius
...     east = dLon * earth_radius * math.cos(math.radians(home_loc.lat))
...     return (east, north) # (x, y) formatında döndürür
... 
... # Metre (North, East) bilgisini tekrar Küresel Koordinata çevirir (Komut göndermek için)
... def get_global_location(home_loc, north, east):
...     earth_radius = 6378137.0
...     dLat = north / earth_radius
...     dLon = east / (earth_radius * math.cos(math.radians(home_loc.lat)))
...     
...     new_lat = home_loc.lat + math.degrees(dLat)
...     new_lon = home_loc.lon + math.degrees(dLon)
...     return LocationGlobalRelative(new_lat, new_lon, home_loc.alt)
... 
# ---------------------------------------------------------
# 2. KIRMIZI ALAN SINIFI (METRE CİNSİNDEN ÇALIŞIR)
# ---------------------------------------------------------
class RedZoneMetric:
    def __init__(self, x_meter, y_meter, radius_meter):
        self.c = (x_meter, y_meter) # Merkezin konumu (Drone Home'a göre metre)
        self.r = radius_meter       # Yarıçap (metre)
    
    # İçinde miyiz?
    def ins(self, x, y): 
        return get_dist((x, y), self.c) < self.r
    
    # Yol Kapalı mı? (10m Paylı - Hata Payı İçin)
    # 35m blocking check (Geniş kontrol) ile çağıracağız.
    def is_blocking_path(self, hx, hy, px, py, margin_metres):
        if self.ins(hx, hy): return False # Zaten içindeysek dışarı atma mantığı çalışsın
        
        p1 = (hx, hy)
        p2 = (px, py)
        dx, dy = p2[0]-p1[0], p2[1]-p1[1]
        l2 = dx*dx + dy*dy
        
        if l2 == 0: return False
        
        # Noktanın doğruya izdüşümü (Projection)
        t = max(0, min(1, ((self.c[0]-p1[0])*dx + (self.c[1]-p1[1])*dy)/l2))
        proj_x, proj_y = p1[0] + t*dx, p1[1] + t*dy
        
        # Engel + Margin kadar yakından geçiyorsa BLOKE sayılır
        return get_dist((proj_x, proj_y), self.c) < (self.r + margin_metres)

# ---------------------------------------------------------
# 3. ANA KAÇIŞ MANTIĞI (SİZİN İSTEDİĞİNİZ KOD)
# ---------------------------------------------------------

def calculate_avoidance_target(vehicle, prey_loc, json_data):
    """
    Bu fonksiyon:
     JSON verisini işler.
     Tüm konumları metreye çevirir.
     Kaçış algoritmasını çalıştırır.
    Gidilmesi gereken hedef koordinatı (LocationGlobalRelative) döndürür.
    """
    
    if not vehicle.home_location:
        print("!! HATA: Home konumu yok, hesaplama yapılamıyor.")
        return None

    # --- A) VERİLERİ HAZIRLA (METRE CİNSİNDEN) ---
    
    # 1. Avcı (Bizim) Konum (Local NED -> x, y)
    hunter_x = vehicle.location.local_frame.east
    hunter_y = vehicle.location.local_frame.north
    
    # 2. Hedef (Av) Konum (Lat/Lon -> x, y)
    prey_xy = get_relative_meters(vehicle.home_location, prey_loc.lat, prey_loc.lon)
    prey_x, prey_y = prey_xy[0], prey_xy[1]
    
    # Gelecek Tahmini (Basitçe hedefin şu anki konumu + hız vektörü eklenebilir)
    # Şimdilik direkt hedefe bakıyoruz (Daha güvenli)
    target_point = (prey_x, prey_y)

    # 3. Kırmızı Alanları JSON'dan Oku ve Oluştur
    red_zones = []
    try:
        data = json.loads(json_data) # JSON string ise parse et
        # Veri yapısı örneği: {"hss_koordinatlari": [{"enlem": x, "boylam": y, "yaricap": r}, ...]}
        hss_list = data.get("hss_koordinatlari", []) 
        
        for hss in hss_list:
            # Lat/Lon -> Metre (x, y)
            pos = get_relative_meters(vehicle.home_location, hss["enlem"], hss["boylam"])
            # RedZoneMetric nesnesi oluştur (Metre cinsinden)
            red_zones.append(RedZoneMetric(pos[0], pos[1], hss["yaricap"]))
            
    except Exception as e:
        print(f"JSON Hatası: {e}")
        return None

    # --- B) ALGORİTMA BAŞLIYOR ---

    # 1. Yardımcı Fonksiyonlar (İç içe tanımlı veya dışarıda olabilir)
    def is_safe_point(px, py, ignore_rz=None):
        for rz in red_zones:
            if rz == ignore_rz: continue
            # x2.0 güvenlik alanı (Diğer engele girmemek için)
            if get_dist((px, py), rz.c) < (rz.r * 2.0):
                return False
        return True

    def is_path_safe(sx, sy, ex, ey, ignore_rz=None):
        for rz in red_zones:
            if rz == ignore_rz: continue
            # Gidiş yolu temizliği (20m pay)
            if rz.is_blocking_path(sx, sy, ex, ey, margin_metres=20):
                return False
        return True

    # 2. Durum Tespiti
    inside_rz = None
    blocking_rz = None
    
    # İçinde miyiz?
    for rz in red_zones:
        if rz.ins(hunter_x, hunter_y): 
            inside_rz = rz
            break
            
    # Yol kapalı mı? (35m "Şişman" Kontrol + Görüş Hattı)
    if not inside_rz:
        for rz in red_zones:
            if rz.is_blocking_path(hunter_x, hunter_y, target_point[0], target_point[1], margin_metres=35):
                blocking_rz = rz
                break
    
    # --- C) KARAR MEKANİZMASI ---
    
    final_x, final_y = 0, 0
    mode = "INTERCEPT"

    # SENARYO 1: İÇİNDEYSEK KAÇ
    if inside_rz:
        mode = "ESCAPE"
        dx = hunter_x - inside_rz.c[0]
        dy = hunter_y - inside_rz.c[1]
        m = math.sqrt(dx**2 + dy**2) or 1
        # Merkezden dışarı doğru 100m ilerle
        final_x = hunter_x + (dx/m) * 100
        final_y = hunter_y + (dy/m) * 100

    # SENARYO 2: YOL KAPALIYSA DOĞRU DÖNÜŞÜ BUL
    elif blocking_rz:
        mode = "AVOID"
        dx = hunter_x - blocking_rz.c[0]
        dy = hunter_y - blocking_rz.c[1]
        angle_from_obs = math.atan2(dy, dx)
        
        # Yörünge: Yarıçap x 2.5
        safe_r = blocking_rz.r * 2.5
        offset = 1.0 # Radyan
        
        # Aday 1 (Sağ / CCW)
        a1 = angle_from_obs + offset
        t1_x = blocking_rz.c[0] + safe_r * math.cos(a1)
        t1_y = blocking_rz.c[1] + safe_r * math.sin(a1)
        
        # Aday 2 (Sol / CW)
        a2 = angle_from_obs - offset
        t2_x = blocking_rz.c[0] + safe_r * math.cos(a2)
        t2_y = blocking_rz.c[1] + safe_r * math.sin(a2)
        
        # GÜVENLİK KONTROLLERİ (Nokta + Yol)
        t1_safe = is_safe_point(t1_x, t1_y, ignore_rz=blocking_rz) and \
                  is_path_safe(hunter_x, hunter_y, t1_x, t1_y, red_zones, ignore_rz=blocking_rz)
                  
        t2_safe = is_safe_point(t2_x, t2_y, ignore_rz=blocking_rz) and \
                  is_path_safe(hunter_x, hunter_y, t2_x, t2_y, red_zones, ignore_rz=blocking_rz)
        
        # Mesafe Kontrolü
        d1 = get_dist((t1_x, t1_y), target_point)
        d2 = get_dist((t2_x, t2_y), target_point)
        
        # SEÇİM
        if t1_safe and not t2_safe:
            final_x, final_y = t1_x, t1_y
        elif t2_safe and not t1_safe:
            final_x, final_y = t2_x, t2_y
        else:
            # İkisi de güvenliyse (veya ikisi de riskliyse) yakına git
            if d1 < d2: final_x, final_y = t1_x, t1_y
            else: final_x, final_y = t2_x, t2_y

    # SENARYO 3: YOL TEMİZSE TAKİP
    else:
        mode = "INTERCEPT"
        final_x, final_y = target_point

    print(f">> MOD: {mode} | Hedef: {final_x:.1f}, {final_y:.1f}")
    
    # --- D) SONUCU DRONE KOORDİNATINA ÇEVİR ---
    # Hesapladığımız (North, East) metreleri tekrar (Lat, Lon) yapıp döndürür.
    return get_global_location(vehicle.home_location, final_y, final_x)

