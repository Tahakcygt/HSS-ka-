import math
import json

# ---------------------------------------------------------
# --- MATEMATİK VE YARDIMCI FONKSİYONLAR ---
# ---------------------------------------------------------

def get_dist(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def is_blocking_path(drone_pos, target_pos, zone, margin=10):
    """
    Drone ile hedef arasındaki çizgi üzerinde engel var mı kontrol eder.
    """
    zx, zy, zr = zone['x'], zone['y'], zone['r']
    hx, hy = drone_pos
    tx, ty = target_pos

    # Eğer zaten engelin içindeysek 'yol kapalı' demeyiz, kaçış modu devreye girer.
    if get_dist(drone_pos, (zx, zy)) < zr:
        return False

    dx, dy = tx - hx, ty - hy
    l2 = dx*dx + dy*dy

    if l2 == 0: return False

    # Doğru parçası üzerindeki en yakın noktayı  bul
    t = max(0, min(1, ((zx - hx)*dx + (zy - hy)*dy) / l2))
    proj_x = hx + t * dx
    proj_y = hy + t * dy

    dist_to_center = get_dist((proj_x, proj_y), (zx, zy))
    
    # Engel yarıçapı + güvenlik marjından yakınsa yol kapalıdır
    return dist_to_center < (zr + margin)

def is_safe_point(point, red_zones, ignore_idx=None):
    """
    Belirtilen noktanın herhangi bir yasaklı bölge içinde olup olmadığını kontrol eder.
    """
    px, py = point
    for i, zone in enumerate(red_zones):
        if i == ignore_idx: continue
        # 2.2 katsayısı güvenli orbit mesafesidir
        if get_dist((px, py), (zone['x'], zone['y'])) < (zone['r'] * 2.2):
            return False
    return True

def is_path_safe(start, end, red_zones, ignore_idx=None):
    """
    İki nokta arasındaki yolun güvenli olup olmadığını kontrol eder.
    """
    for i, zone in enumerate(red_zones):
        if i == ignore_idx: continue
        if is_blocking_path(start, end, zone, margin=20):
            return False
    return True

# ---------------------------------------------------------
# --- ANA ALGORİTMA (TAHMİN + KAÇIŞ) ---
# ---------------------------------------------------------

def calculate_next_waypoint(json_data):
    """
    Girdi olarak JSON alır.
    Çıktı olarak gitmesi gereken koordinatı (waypoint) ve modu döndürür.
    """
    data = json.loads(json_data)
    
    drone_pos = tuple(data["drone_pos"])
    
    # --- YENİ KISIM: HEDEF TAHMİNİ (INTERCEPT PREDICTION) ---
    raw_target_pos = data["target_pos"]     # Hedefin şu anki konumu
    target_vel = data.get("target_vel", [0, 0]) # Hedef hızı [vx, vy]
    
    # 3.0 saniye sonrasını hedefliyoruz (Önleme Rotası)
    PREDICTION_TIME = 3.0 
    
    pred_x = raw_target_pos[0] + (target_vel[0] * PREDICTION_TIME)
    pred_y = raw_target_pos[1] + (target_vel[1] * PREDICTION_TIME)
    
    final_target = (pred_x, pred_y)
    # --------------------------------------------------------

    red_zones = data["red_zones"]
    
    # 1. DURUM: Zaten bir Kırmızı Alanın İçinde miyiz? (ACİL KAÇIŞ)
    inside_zone = None
    inside_idx = -1
    
    for i, zone in enumerate(red_zones):
        if get_dist(drone_pos, (zone['x'], zone['y'])) < zone['r']:
            inside_zone = zone
            inside_idx = i
            break
            
    if inside_zone:
        # Merkezden dışarı doğru vektör hesapla ve uzaklaş
        dx = drone_pos[0] - inside_zone['x']
        dy = drone_pos[1] - inside_zone['y']
        m = math.sqrt(dx**2 + dy**2) or 1
        escape_x = drone_pos[0] + (dx / m) * 200 # 200 metre ileri kaç
        escape_y = drone_pos[1] + (dy / m) * 200
        
        return {
            "mode": "ESCAPE",
            "waypoint": [escape_x, escape_y],
            "reason": "Inside Zone"
        }

    # 2. DURUM: Yol Üzerinde Engel Var mı? (ORBIT MANEVRASI)
    # DİKKAT: 'final_target' (tahmin edilen 3sn sonraki nokta) için kontrol yapıyoruz.
    blocking_zone = None
    blocking_idx = -1
    
    for i, zone in enumerate(red_zones):
        if is_blocking_path(drone_pos, final_target, zone, margin=35):
            blocking_zone = zone
            blocking_idx = i
            break
            
    if blocking_zone:
        # Engelin etrafından dolanmak için teğet noktaları hesapla
        dx = drone_pos[0] - blocking_zone['x']
        dy = drone_pos[1] - blocking_zone['y']
        angle_from_obs = math.atan2(dy, dx)
        
        safe_r = blocking_zone['r'] * 2.5 # Genişlik katsayısı
        offset = 1.0 # Açı ofseti (radyan)
        
        # Sağ Nokta (CCW)
        a1 = angle_from_obs + offset
        t1_x = blocking_zone['x'] + safe_r * math.cos(a1)
        t1_y = blocking_zone['y'] + safe_r * math.sin(a1)
        
        # Sol Nokta (CW)
        a2 = angle_from_obs - offset
        t2_x = blocking_zone['x'] + safe_r * math.cos(a2)
        t2_y = blocking_zone['y'] + safe_r * math.sin(a2)
        
        # Güvenlik kontrolleri
        t1_safe = is_safe_point((t1_x, t1_y), red_zones, ignore_idx=blocking_idx)
        t2_safe = is_safe_point((t2_x, t2_y), red_zones, ignore_idx=blocking_idx)
        
        if t1_safe: t1_safe = is_path_safe(drone_pos, (t1_x, t1_y), red_zones, ignore_idx=blocking_idx)
        if t2_safe: t2_safe = is_path_safe(drone_pos, (t2_x, t2_y), red_zones, ignore_idx=blocking_idx)
            
        chosen_point = None
        d1 = get_dist((t1_x, t1_y), final_target)
        d2 = get_dist((t2_x, t2_y), final_target)
        
        # En kısa ve güvenli yolu seç
        if t1_safe and not t2_safe: chosen_point = (t1_x, t1_y)
        elif t2_safe and not t1_safe: chosen_point = (t2_x, t2_y)
        else: chosen_point = (t1_x, t1_y) if d1 < d2 else (t2_x, t2_y)
            
        return {
            "mode": "AVOID",
            "waypoint": [chosen_point[0], chosen_point[1]],
            "reason": "Obstacle on Predicted Path"
        }

    # 3. DURUM: Yol Temiz (Tahmin Edilen Noktaya Önleme Yap)
    return {
        "mode": "INTERCEPT",
        "waypoint": [final_target[0], final_target[1]], 
        "reason": "Path Clear to Predicted Point"
    }
