import json
import math
from dronekit import LocationGlobalRelative

# ==============================================================================
# 1. YARDIMCI MATEMATİK FONKSİYONLARI
# ==============================================================================

def get_dist(p1, p2): 
    """İki nokta (x,y) arasındaki mesafeyi hesaplar."""
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def normalize_angle(angle):
    """Açıyı -pi ile +pi arasına sıkıştırır."""
    while angle > math.pi: angle -= 2 * math.pi
    while angle < -math.pi: angle += 2 * math.pi
    return angle

def get_relative_meters(home_loc, target_lat, target_lon):
    """
    Coğrafi konumu (Lat/Lon), Home noktasına göre Metre (North/East) cinsine çevirir.
    Dönüş: (x_east, y_north)
    """
    earth_radius = 6378137.0 
    dLat = math.radians(target_lat - home_loc.lat)
    dLon = math.radians(target_lon - home_loc.lon)
    
    north = dLat * earth_radius
    east = dLon * earth_radius * math.cos(math.radians(home_loc.lat))
    return (east, north)

def get_global_location(home_loc, north, east):
    """
    Metre (North/East) bilgisini tekrar Coğrafi Konuma (Lat/Lon) çevirir.
    Drone'a komut göndermek için bu gereklidir.
    """
    earth_radius = 6378137.0
    dLat = north / earth_radius
    dLon = east / (earth_radius * math.cos(math.radians(home_loc.lat)))
    
    new_lat = home_loc.lat + math.degrees(dLat)
    new_lon = home_loc.lon + math.degrees(dLon)
    return LocationGlobalRelative(new_lat, new_lon, home_loc.alt)

# ==============================================================================
# 2. KIRMIZI ALAN SINIFI (METRE BAZLI)
# ==============================================================================

class RedZoneMetric:
    def __init__(self, x_meter, y_meter, radius_meter):
        self.c = (x_meter, y_meter)
        self.r = radius_meter
    
    def ins(self, x, y): 
        """Nokta bu alanın içinde mi?"""
        return get_dist((x, y), self.c) < self.r
    
    def is_blocking_path(self, hx, hy, px, py, margin_metres):
        """
        Yol bu engelin (Yarıçap + margin_metres) kadar yakınından geçiyor mu?
        """
        if self.ins(hx, hy): return False 
        
        p1 = (hx, hy)
        p2 = (px, py)
        dx, dy = p2[0]-p1[0], p2[1]-p1[1]
        l2 = dx*dx + dy*dy
        
        if l2 == 0: return False
        
        t = max(0, min(1, ((self.c[0]-p1[0])*dx + (self.c[1]-p1[1])*dy)/l2))
        proj_x, proj_y = p1[0] + t*dx, p1[1] + t*dy
        
        return get_dist((proj_x, proj_y), self.c) < (self.r + margin_metres)

# ==============================================================================
# 3. OTONOM KAÇIŞ ALGORİTMASI (ANA FONKSİYON)
# ==============================================================================

def calculate_avoidance_target(vehicle, prey_loc, json_data):
    """
    Bu fonksiyon ana kodunuzdan çağrılacak.
    Girdi: 
      - vehicle: Dronekit vehicle objesi
      - prey_loc: Hedefin LocationGlobalRelative konumu
      - json_data: HSS verilerini içeren string
    Çıktı: 
      - LocationGlobalRelative (Gidilecek güvenli nokta)
    """
    
    if not vehicle.home_location:
        print("!! UYARI: Home konumu ayarlı değil, hesaplama yapılamıyor.")
        return None

    # --- A) KONUMLARI METREYE ÇEVİR ---
    # Bizim (Hunter) Konumumuz
    hunter_x = vehicle.location.local_frame.east
    hunter_y = vehicle.location.local_frame.north
    
    # Hedef (Prey) Konumu
    prey_xy = get_relative_meters(vehicle.home_location, prey_loc.lat, prey_loc.lon)
    prey_x, prey_y = prey_xy[0], prey_xy[1]
    
    # Hedef Noktası (Gelecek tahmini eklenebilir, şimdilik direkt hedef)
    target_point = (prey_x, prey_y)

    # --- B) KIRMIZI ALANLARI OLUŞTUR ---
    red_zones = []
    try:
        if isinstance(json_data, str):
            data = json.loads(json_data)
        else:
            data = json_data
            
        hss_list = data.get("hss_koordinatlari", [])
        
        for hss in hss_list:
            pos = get_relative_meters(vehicle.home_location, hss["enlem"], hss["boylam"])
            red_zones.append(RedZoneMetric(pos[0], pos[1], hss["yaricap"]))
            
    except Exception as e:
        print(f"!! JSON AYRIŞTIRMA HATASI: {e}")
        return None

    # --- C) GÜVENLİK KONTROLÜ FONKSİYONLARI ---
    
    def is_safe_point(px, py, ignore_rz=None):
        """Gidilecek nokta başka bir engelin x2.0 yakınına düşüyor mu?"""
        for rz in red_zones:
            if rz == ignore_rz: continue
            if get_dist((px, py), rz.c) < (rz.r * 2.0):
                return False
        return True

    def is_path_safe(sx, sy, ex, ey, ignore_rz=None):
        """Oraya giden yol temiz mi? (20m payla kontrol)"""
        for rz in red_zones:
            if rz == ignore_rz: continue
            if rz.is_blocking_path(sx, sy, ex, ey, margin_metres=20):
                return False
        return True

    # --- D) DURUM ANALİZİ VE KARAR ---
    
    final_x, final_y = target_point
    
    # 1. Acil Durum Kontrolü (İçinde miyiz?)
    inside_rz = None
    for rz in red_zones:
        if rz.ins(hunter_x, hunter_y): 
            inside_rz = rz
            break
            
    # 2. Yol Kontrolü (35 Metre Geniş Payla)
    blocking_rz = None
    if not inside_rz:
        for rz in red_zones:
            if rz.is_blocking_path(hunter_x, hunter_y, target_point[0], target_point[1], margin_metres=35):
                blocking_rz = rz
                break
    
    # --- E) HAREKET PLANI ---

    # SENARYO 1: İÇİNDEYSEK KAÇ
    if inside_rz:
        print(">> DURUM: ACİL KAÇIŞ (ALAN İÇİ)")
        dx = hunter_x - inside_rz.c[0]
        dy = hunter_y - inside_rz.c[1]
        m = math.sqrt(dx**2 + dy**2) or 1
        # Merkezden dışarı doğru 150m git
        final_x = hunter_x + (dx/m) * 150
        final_y = hunter_y + (dy/m) * 150

    # SENARYO 2: YOL KAPALIYSA DOĞRU TARAFTAN DÖN
    elif blocking_rz:
        print(">> DURUM: ENGELDEN KAÇINMA (MANEVRA)")
        dx = hunter_x - blocking_rz.c[0]
        dy = hunter_y - blocking_rz.c[1]
        angle_from_obs = math.atan2(dy, dx)
        
        # Yörünge: Yarıçap x 2.5 (Geniş Dönüş)
        safe_r = blocking_rz.r * 2.5
        offset = 1.0 # Radyan (Yaklaşık 60 derece)
        
        # Aday 1 (Sağ / CCW)
        a1 = angle_from_obs + offset
        t1_x = blocking_rz.c[0] + safe_r * math.cos(a1)
        t1_y = blocking_rz.c[1] + safe_r * math.sin(a1)
        
        # Aday 2 (Sol / CW)
        a2 = angle_from_obs - offset
        t2_x = blocking_rz.c[0] + safe_r * math.cos(a2)
        t2_y = blocking_rz.c[1] + safe_r * math.sin(a2)
        
        # Güvenlik Taraması (Nokta + Yol)
        t1_safe = is_safe_point(t1_x, t1_y, ignore_rz=blocking_rz) and \
                  is_path_safe(hunter_x, hunter_y, t1_x, t1_y, ignore_rz=blocking_rz)
                  
        t2_safe = is_safe_point(t2_x, t2_y, ignore_rz=blocking_rz) and \
                  is_path_safe(hunter_x, hunter_y, t2_x, t2_y, ignore_rz=blocking_rz)
        
        # Mesafe Kontrolü
        d1 = get_dist((t1_x, t1_y), target_point)
        d2 = get_dist((t2_x, t2_y), target_point)
        
        # KARAR VERME
        if t1_safe and not t2_safe:
            final_x, final_y = t1_x, t1_y # Sadece T1 güvenli
        elif t2_safe and not t1_safe:
            final_x, final_y = t2_x, t2_y # Sadece T2 güvenli
        else:
            # İkisi de güvenli (veya ikisi de riskli) -> Yakın olana git
            if d1 < d2: final_x, final_y = t1_x, t1_y
            else: final_x, final_y = t2_x, t2_y

    else:
        # SENARYO 3: YOL TEMİZ -> SALDIR
        # print(">> DURUM: TAKİP (YOL AÇIK)") # İstersen yorumu aç
        final_x, final_y = target_point
    
    # --- F) SONUCU TEKRAR ENLEM/BOYLAM YAP VE DÖNDÜR ---
    return get_global_location(vehicle.home_location, final_y, final_x)
    return get_global_location(vehicle.home_location, final_y, final_x)
