import pcbnew
import math
import os
import wx

STACKUP_THICKNESS_MM = 1.0 
TRACE_THICKNESS_MM = 0.2 
BOARD_MESH_STEP_MM = 0.5  
TRACE_MESH_STEP_MM = 0.1  

# 完美一维曲率积分表
class CurveTable:
    def __init__(self, zones, min_bound, max_bound):
        self.table = []
        if not zones:
            self.valid = False
            return
        self.valid = True
        
        curr_t = min_bound - 10
        curr_S = curr_t 
        curr_Z = 0.0
        curr_theta = 0.0
        
        self.table.append((curr_t, curr_S, curr_Z, curr_theta))
        
        num_steps = int((max_bound + 10 - curr_t) / 0.1)
        for _ in range(num_steps):
            k = 0.0
            for z in zones:
                if z["eff_min"] <= curr_t <= z["eff_max"]:
                    R = z["radius"]
                    k = (1.0 if z["angle_rad"] > 0 else -1.0) / R
                    break
            
            d_theta = k * 0.1
            mid_theta = curr_theta + d_theta / 2
            
            curr_S += 0.1 * math.cos(mid_theta)
            curr_Z += 0.1 * math.sin(mid_theta)
            curr_theta += d_theta
            curr_t += 0.1
            
            self.table.append((curr_t, curr_S, curr_Z, curr_theta))
            
    def get(self, val):
        if not self.valid: return val, 0, 0
        if val <= self.table[0][0]: 
            t0, s0, z0, th0 = self.table[0]
            dx = val - t0
            return s0 + dx * math.cos(th0), z0 + dx * math.sin(th0), th0
        if val >= self.table[-1][0]: 
            t1, s1, z1, th1 = self.table[-1]
            dx = val - t1
            return s1 + dx * math.cos(th1), z1 + dx * math.sin(th1), th1
        
        lo, hi = 0, len(self.table) - 1
        while lo < hi - 1:
            mid = (lo + hi) // 2
            if self.table[mid][0] <= val: lo = mid
            else: hi = mid
                
        t0, s0, z0, th0 = self.table[lo]
        t1, s1, z1, th1 = self.table[hi]
        
        ratio = (val - t0) / (t1 - t0)
        return s0 + (s1 - s0) * ratio, z0 + (z1 - z0) * ratio, th0 + (th1 - th0) * ratio

class FlexBendSimPlugin(pcbnew.ActionPlugin):
    def defaults(self):
        self.name = "Flex Bend 3D Sim Export"
        self.category = "Flex Electronics"
        self.description = "Export 3D Decoupled Geometry"
        self.show_toolbar_button = True
        self.icon_file_name = os.path.join(os.path.dirname(__file__), "icon.png")

    def Run(self):
        board = pcbnew.GetBoard()
        
        all_tracks = []
        for track in board.GetTracks():
            if track.GetClass() in ('PCB_TRACK', 'PCB_ARC'):
                start, end = track.GetStart(), track.GetEnd()
                all_tracks.append((
                    pcbnew.ToMM(start.x), pcbnew.ToMM(start.y), 
                    pcbnew.ToMM(end.x), pcbnew.ToMM(end.y), 
                    pcbnew.ToMM(track.GetWidth())
                ))
        for drawing in board.GetDrawings():
            if drawing.GetClass() == 'PCB_SHAPE' and drawing.GetShape() == pcbnew.SHAPE_T_SEGMENT:
                start, end = drawing.GetStart(), drawing.GetEnd()
                all_tracks.append((
                    pcbnew.ToMM(start.x), pcbnew.ToMM(start.y), 
                    pcbnew.ToMM(end.x), pcbnew.ToMM(end.y), 
                    pcbnew.ToMM(drawing.GetWidth())
                ))
                
        x_zones = []
        y_zones = []
        
        for drawing in board.GetDrawings():
            if drawing.GetClass() == 'PCB_SHAPE' and drawing.GetShape() in (pcbnew.SHAPE_T_RECT, pcbnew.SHAPE_T_POLY):
                layer_name = board.GetLayerName(drawing.GetLayer())
                if layer_name.startswith("User."):
                    bbox = drawing.GetBoundingBox()
                    w = pcbnew.ToMM(bbox.GetWidth())
                    h = pcbnew.ToMM(bbox.GetHeight())
                    if w < 0.1 or h < 0.1: continue
                    
                    zx_min = pcbnew.ToMM(bbox.GetX())
                    zy_min = pcbnew.ToMM(bbox.GetY())
                    zx_max = zx_min + w
                    zy_max = zy_min + h
                    is_vertical = h > w
                    
                    real_min, real_max = float('inf'), -float('inf')
                    for x1, y1, x2, y2, tw in all_tracks:
                        if max(x1, x2) + tw >= zx_min and min(x1, x2) - tw <= zx_max and max(y1, y2) + tw >= zy_min and min(y1, y2) - tw <= zy_max:
                            if is_vertical:
                                c1 = max(min(x1, x2) - tw/2.0, zx_min)
                                c2 = min(max(x1, x2) + tw/2.0, zx_max)
                            else:
                                c1 = max(min(y1, y2) - tw/2.0, zy_min)
                                c2 = min(max(y1, y2) + tw/2.0, zy_max)
                            if c1 <= c2:
                                real_min = min(real_min, c1)
                                real_max = max(real_max, c2)
                    
                    if real_min == float('inf'): continue
                        
                    if real_max - real_min < 0.5:
                        center = (real_max + real_min) / 2
                        real_min = center - 0.25
                        real_max = center + 0.25
                    
                    zone_data = {
                        "layer": layer_name,
                        "eff_min": real_min,
                        "eff_max": real_max,
                        "length": real_max - real_min,
                        "cross_min": zy_min if is_vertical else zx_min,
                        "cross_max": zy_max if is_vertical else zx_max,
                    }
                    if is_vertical: x_zones.append(zone_data)
                    else: y_zones.append(zone_data)
                    
        if not x_zones and not y_zones:
            board.GetDesignSettings().m_MessagePanelText = "错误: 未找到有效折叠区"
            return
            
        for z in x_zones + y_zones:
            prompt = f"图层 {z['layer']} 弯折角度:"
            dlg = wx.TextEntryDialog(None, prompt, "定义折叠参数", "90")
            if dlg.ShowModal() == wx.ID_OK:
                try:
                    angle_deg = float(dlg.GetValue())
                    angle_deg = max(-180.0, min(180.0, angle_deg))
                    if abs(angle_deg) < 0.1: angle_deg = 0.1
                except ValueError: angle_deg = 90.0
            else: return
            dlg.Destroy()
            z["angle_rad"] = math.radians(angle_deg)
            z["radius"] = z["length"] / abs(z["angle_rad"])

        g_min_x = min([z["eff_min"] for z in x_zones] + [0]) - 20
        g_max_x = max([z["eff_max"] for z in x_zones] + [100]) + 20
        g_min_y = min([z["eff_min"] for z in y_zones] + [0]) - 20
        g_max_y = max([z["eff_max"] for z in y_zones] + [100]) + 20
        
        for x1, y1, x2, y2, _ in all_tracks:
            g_min_x = min(g_min_x, x1, x2) - 10
            g_max_x = max(g_max_x, x1, x2) + 10
            g_min_y = min(g_min_y, y1, y2) - 10
            g_max_y = max(g_max_y, y1, y2) + 10

        x_table = CurveTable(x_zones, g_min_x, g_max_x)
        y_table = CurveTable(y_zones, g_min_y, g_max_y)
        
        # 最核心的坐标映射，所有几何体绝对同源
        def map_point(x, y, z):
            sx, szx, _ = x_table.get(x)
            sy, szy, _ = y_table.get(y)
            return (sx, sy, z + szx + szy)
        
        output_dir = os.path.dirname(board.GetFileName())
        if not output_dir: output_dir = os.getcwd()
        
        stl_path = os.path.join(output_dir, "flex_bend_mesh.stl")
        self._export_final(stl_path, all_tracks, x_zones, y_zones, map_point)
        board.GetDesignSettings().m_MessagePanelText = f"分治合并完美导出成功！"

    def _export_final(self, path, all_tracks, x_zones, y_zones, map_point):
        def write_tri(f, v1, v2, v3):
            ux, uy, uz = v2[0]-v1[0], v2[1]-v1[1], v2[2]-v1[2]
            vx, vy, vz = v3[0]-v1[0], v3[1]-v1[1], v3[2]-v1[2]
            nx, ny, nz = uy*vz - uz*vy, uz*vx - ux*vz, ux*vy - uy*vx
            length = math.sqrt(nx**2 + ny**2 + nz**2)
            if length > 0: nx, ny, nz = nx/length, ny/length, nz/length
            else: nx, ny, nz = 0, 0, 0
            f.write(f"facet normal {nx} {ny} {nz}\nouter loop\n")
            for v in [v1, v2, v3]:
                f.write(f"vertex {v[0]} {v[1]} {v[2]}\n")
            f.write("endloop\nendfacet\n")

        with open(path, 'w') as f:
            f.write("solid FlexPCB\n")
            
            # 1. 渲染 X 轴折叠区底板（只在 eff_min 到 eff_max 内，且使用全局 map_point）
            for zone in x_zones:
                x_steps = max(1, int((zone["eff_max"] - zone["eff_min"]) / BOARD_MESH_STEP_MM))
                y_steps = max(1, int((zone["cross_max"] - zone["cross_min"]) / BOARD_MESH_STEP_MM))
                for i in range(x_steps):
                    xa = zone["eff_min"] + ((zone["eff_max"] - zone["eff_min"]) * i) / x_steps
                    xb = zone["eff_min"] + ((zone["eff_max"] - zone["eff_min"]) * (i + 1)) / x_steps
                    for j in range(y_steps):
                        ya = zone["cross_min"] + ((zone["cross_max"] - zone["cross_min"]) * j) / y_steps
                        yb = zone["cross_min"] + ((zone["cross_max"] - zone["cross_min"]) * (j + 1)) / y_steps
                        
                        p00_t = map_point(xa, ya, 0); p10_t = map_point(xb, ya, 0)
                        p11_t = map_point(xb, yb, 0); p01_t = map_point(xa, yb, 0)
                        p00_b = map_point(xa, ya, -STACKUP_THICKNESS_MM); p10_b = map_point(xb, ya, -STACKUP_THICKNESS_MM)
                        p11_b = map_point(xb, yb, -STACKUP_THICKNESS_MM); p01_b = map_point(xa, yb, -STACKUP_THICKNESS_MM)
                        
                        write_tri(f, p00_t, p10_t, p11_t); write_tri(f, p00_t, p11_t, p01_t)
                        write_tri(f, p01_b, p11_b, p10_b); write_tri(f, p01_b, p10_b, p00_b)
                        if i == 0: write_tri(f, p00_t, p00_b, p01_b); write_tri(f, p00_t, p01_b, p01_t)
                        if i == x_steps - 1: write_tri(f, p10_t, p11_t, p11_b); write_tri(f, p10_t, p11_b, p10_b)
                        if j == 0: write_tri(f, p00_t, p10_t, p10_b); write_tri(f, p00_t, p10_b, p00_b)
                        if j == y_steps - 1: write_tri(f, p01_t, p01_b, p11_b); write_tri(f, p01_t, p11_b, p11_t)

            # 2. 渲染 Y 轴折叠区底板
            for zone in y_zones:
                x_steps = max(1, int((zone["cross_max"] - zone["cross_min"]) / BOARD_MESH_STEP_MM))
                y_steps = max(1, int((zone["eff_max"] - zone["eff_min"]) / BOARD_MESH_STEP_MM))
                for i in range(x_steps):
                    xa = zone["cross_min"] + ((zone["cross_max"] - zone["cross_min"]) * i) / x_steps
                    xb = zone["cross_min"] + ((zone["cross_max"] - zone["cross_min"]) * (i + 1)) / x_steps
                    for j in range(y_steps):
                        ya = zone["eff_min"] + ((zone["eff_max"] - zone["eff_min"]) * j) / y_steps
                        yb = zone["eff_min"] + ((zone["eff_max"] - zone["eff_min"]) * (j + 1)) / y_steps
                        
                        p00_t = map_point(xa, ya, 0); p10_t = map_point(xb, ya, 0)
                        p11_t = map_point(xb, yb, 0); p01_t = map_point(xa, yb, 0)
                        p00_b = map_point(xa, ya, -STACKUP_THICKNESS_MM); p10_b = map_point(xb, ya, -STACKUP_THICKNESS_MM)
                        p11_b = map_point(xb, yb, -STACKUP_THICKNESS_MM); p01_b = map_point(xa, yb, -STACKUP_THICKNESS_MM)
                        
                        write_tri(f, p00_t, p10_t, p11_t); write_tri(f, p00_t, p11_t, p01_t)
                        write_tri(f, p01_b, p11_b, p10_b); write_tri(f, p01_b, p10_b, p00_b)
                        if i == 0: write_tri(f, p00_t, p00_b, p01_b); write_tri(f, p00_t, p01_b, p01_t)
                        if i == x_steps - 1: write_tri(f, p10_t, p11_t, p11_b); write_tri(f, p10_t, p11_b, p10_b)
                        if j == 0: write_tri(f, p00_t, p10_t, p10_b); write_tri(f, p00_t, p10_b, p00_b)
                        if j == y_steps - 1: write_tri(f, p01_t, p01_b, p11_b); write_tri(f, p01_t, p11_b, p11_t)

            # 3. 渲染走线
            for x1, y1, x2, y2, width in all_tracks:
                length = math.hypot(x2 - x1, y2 - y1)
                steps = max(1, int(length / TRACE_MESH_STEP_MM))
                w_mm = max(0.2, width)
                
                for i in range(steps):
                    px0 = x1 + ((x2 - x1) * i) / steps
                    py0 = y1 + ((y2 - y1) * i) / steps
                    px1 = x1 + ((x2 - x1) * (i + 1)) / steps
                    py1 = y1 + ((y2 - y1) * (i + 1)) / steps
                    
                    dx, dy = px1 - px0, py1 - py0
                    L = math.sqrt(dx**2 + dy**2)
                    if L < 1e-9: continue
                    nx, ny = -dy/L * w_mm/2, dx/L * w_mm/2
                    
                    z_top = 0.01 + TRACE_THICKNESS_MM
                    z_bot = 0.01
                    c = [(px0+nx, py0+ny), (px1+nx, py1+ny), (px1-nx, py1-ny), (px0-nx, py0-ny)]
                    
                    p_top = [map_point(cx, cy, z_top) for cx, cy in c]
                    p_bot = [map_point(cx, cy, z_bot) for cx, cy in c]
                    
                    write_tri(f, p_top[0], p_top[1], p_top[2]); write_tri(f, p_top[0], p_top[2], p_top[3])
                    write_tri(f, p_bot[3], p_bot[2], p_bot[1]); write_tri(f, p_bot[3], p_bot[1], p_bot[0])
                    for k in range(4):
                        k_next = (k+1)%4
                        write_tri(f, p_top[k], p_top[k_next], p_bot[k_next]); write_tri(f, p_top[k], p_bot[k_next], p_bot[k])
                        
            f.write("endsolid FlexPCB\n")

FlexBendSimPlugin().register()
