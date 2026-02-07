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

    # Eğer zaten engelin içindeysek kaçış modu devreye girer.
    if get_dist(drone_pos, (zx, zy)) < zr:
        return False

    dx, dy = tx - hx, ty - hy
    l2 = dx*dx + dy*dy

    if l2 == 0: return False

    # Doğru parçası üzerindeki en yakın noktayı bul
    t = max(0, min(1, ((zx - hx)*dx + (zy - hy)*dy) / l2))
    proj_x = hx + t * dx
    proj_y = hy + t * dy

    dist_to_center = get_dist((proj_x, proj_y), (zx, zy))
    
    # Engel yarıçapı + güvenlik marjından yakınsa yol kapalıdır
    return dist_to_center < (zr + margin)

def is_safe_point(point, red_zones, ignore_idx=None):
    px, py = point
    for i, zone in enumerate(red_zones):
        if i == ignore_idx: continue
        if get_dist((px, py), (zone['x'], zone['y'])) < (zone['r'] * 2.2):
            return False
    return True

def is_path_safe(start, end, red_zones, ignore_idx=None):
    for i, zone in enumerate(red_zones):
        if i == ignore_idx: continue
        if is_blocking_path(start, end, zone, margin=20):
            return False
    return True

# ---------------------------------------------------------
# --- ANA ALGORİTMA ---
# ---------------------------------------------------------

def calculate_next_waypoint(json_data):
    """
    Girdi: JSON verisi
    Çıktı: Waypoint (dict)
    """
    data = json.loads(json_data)
    
    drone_pos = tuple(data["drone_pos"])
    
    # --- 1. HEDEF VERİLERİ ---
    raw_target_pos = data["target_pos"]             # Hedefin Konumu
    target_vel = data.get("target_vel", [0, 0])
    
    # [YENİ] Drone Hız Verisi (Burnunun nereye baktığını anlamak için lazım)
    # Eğer json'da gelmezse varsayılan 0 alır ve bu özellik pasif kalır.
    drone_vel = data.get("drone_vel", [0, 0]) 

    # --- 2. TAHMİN (Sadece Hesaplama İçin) ---
    PREDICTION_TIME = 3.0 
    pred_x = raw_target_pos[0] + (target_vel[0] * PREDICTION_TIME)
    pred_y = raw_target_pos[1] + (target_vel[1] * PREDICTION_TIME)
    
    predicted_target = (pred_x, pred_y)             # HESAPLAMA REFERANSI (Sanal)
    # --------------------------------------------------------

    red_zones = data["red_zones"]
    
    # --- 3. ACİL KAÇIŞ KONTROLÜ (INSIDE) ---
    inside_zone = None
    inside_idx = -1
    
    for i, zone in enumerate(red_zones):
        if get_dist(drone_pos, (zone['x'], zone['y'])) < zone['r']:
            inside_zone = zone
            inside_idx = i
            break
            
    if inside_zone:
        dx = drone_pos[0] - inside_zone['x']
        dy = drone_pos[1] - inside_zone['y']
        m = math.sqrt(dx**2 + dy**2) or 1
        escape_x = drone_pos[0] + (dx / m) * 200
        escape_y = drone_pos[1] + (dy / m) * 200
        
        return {
            "mode": "ESCAPE",
            "waypoint": [escape_x, escape_y],
            "reason": "Inside Zone"
        }

   
    # Drone hareket halindeyse (hız > 1 m/s), gittiği yönde engel var mı bakar.
    drone_speed = math.sqrt(drone_vel[0]**2 + drone_vel[1]**2)
    
    if drone_speed > 1.0:
        LOOK_AHEAD_TIME = 1.3 # Ne kadar uzağa bakacak (saniye cinsinden)
        look_ahead_x = drone_pos[0] + (drone_vel[0] * LOOK_AHEAD_TIME)
        look_ahead_y = drone_pos[1] + (drone_vel[1] * LOOK_AHEAD_TIME)
        
        for zone in red_zones:
            # Hedef çizgisine değil, kendi gidiş hattımıza (Look Ahead) bakıyoruz
            if is_blocking_path(drone_pos, (look_ahead_x, look_ahead_y), zone, margin=20):
                # Eğer önümüz kapalıysa: Engel merkezinden dron'a doğru bir vektör çıkar (İTME)
                dx = drone_pos[0] - zone['x']
                dy = drone_pos[1] - zone['y']
                m = math.sqrt(dx**2 + dy**2) or 1
                
                # Engelden 500m uzağa doğru bir nokta koy (Repulsion Point)
                repulsion_x = drone_pos[0] + (dx / m) * 500
                repulsion_y = drone_pos[1] + (dy / m) * 500
                
                return {
                    "mode": "REPULSION",
                    "waypoint": [repulsion_x, repulsion_y],
                    "reason": "Forward Collision Imminent (Repulsion)"
                }
    # =========================================================================


    # --- 4. ENGEL KONTROLÜ VE ORBIT (NORMAL ROTA) ---
    blocking_zone = None
    blocking_idx = -1
    
    for i, zone in enumerate(red_zones):
        # DİKKAT: Engel kontrolünü 'raw_target_pos' (Şu anki hedef) ile yapıyoruz
        if is_blocking_path(drone_pos, raw_target_pos, zone, margin=35):
            blocking_zone = zone
            blocking_idx = i
            break
            
    if blocking_zone:
        dx = drone_pos[0] - blocking_zone['x']
        dy = drone_pos[1] - blocking_zone['y']
        angle_from_obs = math.atan2(dy, dx)
        
        safe_r = blocking_zone['r'] * 2.5 
        offset = 1.0 
        
        # Sağ (CCW) ve Sol (CW) noktalar
        a1 = angle_from_obs + offset
        t1_x = blocking_zone['x'] + safe_r * math.cos(a1)
        t1_y = blocking_zone['y'] + safe_r * math.sin(a1)
        
        a2 = angle_from_obs - offset
        t2_x = blocking_zone['x'] + safe_r * math.cos(a2)
        t2_y = blocking_zone['y'] + safe_r * math.sin(a2)
        
        # Güvenlik Kontrolleri
        t1_safe = is_safe_point((t1_x, t1_y), red_zones, ignore_idx=blocking_idx)
        t2_safe = is_safe_point((t2_x, t2_y), red_zones, ignore_idx=blocking_idx)
        
        if t1_safe: t1_safe = is_path_safe(drone_pos, (t1_x, t1_y), red_zones, ignore_idx=blocking_idx)
        if t2_safe: t2_safe = is_path_safe(drone_pos, (t2_x, t2_y), red_zones, ignore_idx=blocking_idx)
            
        chosen_point = None
        
        # Hangi tarafın daha kısa olduğuna karar verirken
        # 'predicted_target' (2sn sonraki konum) kullanıyoruz.
        d1 = get_dist((t1_x, t1_y), predicted_target) 
        d2 = get_dist((t2_x, t2_y), predicted_target)
        
        if t1_safe and not t2_safe: chosen_point = (t1_x, t1_y)
        elif t2_safe and not t1_safe: chosen_point = (t2_x, t2_y)
        else: chosen_point = (t1_x, t1_y) if d1 < d2 else (t2_x, t2_y)
            
        return {
            "mode": "AVOID",
            "waypoint": [chosen_point[0], chosen_point[1]],
            "reason": "Obstacle on Current Path (Optimized for Future)"
        }

    # --- 5. YOL TEMİZ ---
    # Engel yoksa doğrudan 'raw_target_pos'a (şu anki konuma) git.
    return {
        "mode": "INTERCEPT",
        "waypoint": [raw_target_pos[0], raw_target_pos[1]], 
        "reason": "Path Clear to Target"
    }


