import math
import os
from PIL import Image, ImageDraw

def render_icon(draw_fn, size, supersample=4):
    dim = size * supersample
    im = Image.new('L', (dim, dim), 0)
    d = ImageDraw.Draw(im)
    draw_fn(d, dim)
    return im.resize((size, size), Image.Resampling.LANCZOS)

def draw_control(d, dim):
    c = dim / 2
    r = dim * 0.44
    w = int(dim * 0.08)
    d.ellipse([c - dim*0.14, c - dim*0.14, c + dim*0.14, c + dim*0.14], outline=255, width=w)
    d.line([(c, c - dim*0.18), (c, c - r)], fill=255, width=w)
    d.line([(c - dim*0.13, c - r + dim*0.13), (c, c - r), (c + dim*0.13, c - r + dim*0.13)], fill=255, width=w)
    d.line([(c, c + dim*0.18), (c, c + r)], fill=255, width=w)
    d.line([(c - dim*0.13, c + r - dim*0.13), (c, c + r), (c + dim*0.13, c + r - dim*0.13)], fill=255, width=w)
    d.line([(c - dim*0.18, c), (c - r, c)], fill=255, width=w)
    d.line([(c - r + dim*0.13, c - dim*0.13), (c - r, c), (c - r + dim*0.13, c + dim*0.13)], fill=255, width=w)
    d.line([(c + dim*0.18, c), (c + r, c)], fill=255, width=w)
    d.line([(c + r - dim*0.13, c - dim*0.13), (c + r, c), (c + r - dim*0.13, c + dim*0.13)], fill=255, width=w)

def draw_files(d, dim):
    w = int(dim * 0.08)
    rad = int(dim * 0.12)
    d.rounded_rectangle([dim*0.12, dim*0.28, dim*0.88, dim*0.84], radius=rad, outline=255, width=w)
    d.arc([dim*0.12, dim*0.16, dim*0.48, dim*0.36], start=180, end=0, fill=255, width=w)
    d.line([(dim*0.12, dim*0.26), (dim*0.12, dim*0.34)], fill=255, width=w)
    d.line([(dim*0.48, dim*0.26), (dim*0.56, dim*0.28)], fill=255, width=w)
    for x in [0.35, 0.50, 0.65]:
        d.line([(dim*x, dim*0.46), (dim*x, dim*0.66)], fill=255, width=w)

def draw_monitor(d, dim):
    c = dim / 2
    w = int(dim * 0.08)
    d.arc([dim*0.12, dim*0.12, dim*0.88, dim*0.88], start=135, end=45, fill=255, width=w)
    d.ellipse([c - dim*0.10, c - dim*0.10, c + dim*0.10, c + dim*0.10], fill=255)
    d.line([(c, c), (c + dim*0.25, c - dim*0.25)], fill=255, width=int(w*1.2))
    for deg in [180, 225, 270, 315, 0]:
        rad = math.radians(deg)
        x1 = c + dim*0.32 * math.cos(rad)
        y1 = c + dim*0.32 * math.sin(rad)
        x2 = c + dim*0.42 * math.cos(rad)
        y2 = c + dim*0.42 * math.sin(rad)
        d.line([(x1, y1), (x2, y2)], fill=255, width=int(w*0.8))

def draw_calibration(d, dim):
    c = dim / 2
    w = int(dim * 0.08)
    d.ellipse([dim*0.12, dim*0.12, dim*0.88, dim*0.88], outline=255, width=w)
    d.ellipse([dim*0.28, dim*0.28, dim*0.72, dim*0.72], outline=255, width=w)
    d.ellipse([c - dim*0.07, c - dim*0.07, c + dim*0.07, c + dim*0.07], fill=255)
    d.line([(dim*0.04, c), (dim*0.24, c)], fill=255, width=w)
    d.line([(dim*0.76, c), (dim*0.96, c)], fill=255, width=w)
    d.line([(c, dim*0.04), (c, dim*0.24)], fill=255, width=w)
    d.line([(c, dim*0.76), (c, dim*0.96)], fill=255, width=w)

def draw_shapes(d, dim):
    w = int(dim * 0.08)
    d.rounded_rectangle([dim*0.10, dim*0.10, dim*0.52, dim*0.52], radius=int(dim*0.08), outline=255, width=w)
    d.ellipse([dim*0.48, dim*0.35, dim*0.90, dim*0.77], outline=255, width=w)
    p1 = (dim*0.14, dim*0.88)
    p2 = (dim*0.46, dim*0.88)
    p3 = (dim*0.30, dim*0.58)
    d.line([p1, p2, p3, p1], fill=255, width=w)

def draw_settings(d, dim):
    c = dim / 2
    w = int(dim * 0.08)
    num_teeth = 6
    r_in = dim * 0.28
    r_out = dim * 0.44
    pts = []
    for i in range(num_teeth * 2):
        angle = i * (2 * math.pi / (num_teeth * 2))
        r = r_out if (i % 2 == 0) else r_in
        pts.append((c + r * math.cos(angle), c + r * math.sin(angle)))
    d.polygon(pts, fill=255)
    d.ellipse([c - dim*0.16, c - dim*0.16, c + dim*0.16, c + dim*0.16], fill=0)

def draw_wifi(d, dim):
    c = dim / 2
    w = int(dim * 0.08)
    base_y = dim * 0.70
    d.ellipse([c - dim*0.07, base_y - dim*0.07, c + dim*0.07, base_y + dim*0.07], fill=255)
    d.arc([c - dim*0.18, base_y - dim*0.18, c + dim*0.18, base_y + dim*0.18], start=218, end=322, fill=255, width=w)
    d.arc([c - dim*0.32, base_y - dim*0.32, c + dim*0.32, base_y + dim*0.32], start=218, end=322, fill=255, width=w)
    d.arc([c - dim*0.46, base_y - dim*0.46, c + dim*0.46, base_y + dim*0.46], start=218, end=322, fill=255, width=w)

def draw_wifi_med(d, dim):
    c = dim / 2
    w = int(dim * 0.08)
    base_y = dim * 0.70
    d.ellipse([c - dim*0.07, base_y - dim*0.07, c + dim*0.07, base_y + dim*0.07], fill=255)
    d.arc([c - dim*0.18, base_y - dim*0.18, c + dim*0.18, base_y + dim*0.18], start=218, end=322, fill=255, width=w)
    d.arc([c - dim*0.32, base_y - dim*0.32, c + dim*0.32, base_y + dim*0.32], start=218, end=322, fill=255, width=w)

def draw_wifi_low(d, dim):
    c = dim / 2
    w = int(dim * 0.08)
    base_y = dim * 0.70
    d.ellipse([c - dim*0.07, base_y - dim*0.07, c + dim*0.07, base_y + dim*0.07], fill=255)
    d.arc([c - dim*0.18, base_y - dim*0.18, c + dim*0.18, base_y + dim*0.18], start=218, end=322, fill=255, width=w)

def draw_wifi_dot(d, dim):
    c = dim / 2
    base_y = dim * 0.70
    d.ellipse([c - dim*0.07, base_y - dim*0.07, c + dim*0.07, base_y + dim*0.07], fill=255)

def draw_info(d, dim):
    c = dim / 2
    w = int(dim * 0.08)
    d.ellipse([dim*0.12, dim*0.12, dim*0.88, dim*0.88], outline=255, width=w)
    d.ellipse([c - dim*0.06, dim*0.28, c + dim*0.06, dim*0.40], fill=255)
    d.line([(c, dim*0.48), (c, dim*0.72)], fill=255, width=int(w*1.3))

def draw_bed(d, dim):
    w = int(dim * 0.08)
    d.rounded_rectangle([dim*0.14, dim*0.14, dim*0.86, dim*0.86], radius=int(dim*0.08), outline=255, width=w)
    d.line([(dim*0.24, dim*0.76), (dim*0.46, dim*0.76)], fill=255, width=int(w*1.2))
    d.line([(dim*0.24, dim*0.76), (dim*0.24, dim*0.54)], fill=255, width=int(w*1.2))

def draw_laser(d, dim):
    c = dim / 2
    w = int(dim * 0.08)
    d.rounded_rectangle([dim*0.20, dim*0.10, dim*0.80, dim*0.30], radius=int(dim*0.06), fill=255)
    focal_y = dim * 0.75
    d.line([(dim*0.30, dim*0.30), (c, focal_y)], fill=255, width=w)
    d.line([(dim*0.70, dim*0.30), (c, focal_y)], fill=255, width=w)
    d.line([(c, dim*0.30), (c, focal_y)], fill=255, width=w)
    d.ellipse([c - dim*0.09, focal_y - dim*0.09, c + dim*0.09, focal_y + dim*0.09], fill=255)
    d.line([(c - dim*0.18, focal_y), (c + dim*0.18, focal_y)], fill=255, width=w)
    d.line([(c, focal_y - dim*0.18), (c, focal_y + dim*0.18)], fill=255, width=w)

def draw_home(d, dim):
    c = dim / 2
    w = int(dim * 0.08)
    d.line([(dim*0.15, dim*0.45), (c, dim*0.16), (dim*0.85, dim*0.45)], fill=255, width=w)
    d.line([(dim*0.24, dim*0.42), (dim*0.24, dim*0.84), (dim*0.76, dim*0.84), (dim*0.76, dim*0.42)], fill=255, width=w)
    d.rounded_rectangle([c - dim*0.12, dim*0.58, c + dim*0.12, dim*0.84], radius=int(dim*0.04), outline=255, width=w)

def draw_play(d, dim):
    w = int(dim * 0.08)
    pts = [(dim*0.25, dim*0.15), (dim*0.85, dim*0.50), (dim*0.25, dim*0.85)]
    d.polygon(pts, fill=255)

def draw_pause(d, dim):
    rad = int(dim * 0.08)
    d.rounded_rectangle([dim*0.22, dim*0.18, dim*0.42, dim*0.82], radius=rad, fill=255)
    d.rounded_rectangle([dim*0.58, dim*0.18, dim*0.78, dim*0.82], radius=rad, fill=255)

def draw_stop(d, dim):
    rad = int(dim * 0.12)
    d.rounded_rectangle([dim*0.20, dim*0.20, dim*0.80, dim*0.80], radius=rad, fill=255)

def draw_refresh(d, dim):
    w = int(dim * 0.10)
    d.arc([dim*0.16, dim*0.16, dim*0.84, dim*0.84], start=45, end=315, fill=255, width=w)
    d.polygon([(dim*0.75, dim*0.15), (dim*0.92, dim*0.30), (dim*0.68, dim*0.35)], fill=255)

def draw_trash(d, dim):
    w = int(dim * 0.08)
    d.line([(dim*0.35, dim*0.18), (dim*0.65, dim*0.18)], fill=255, width=w)
    d.line([(dim*0.15, dim*0.28), (dim*0.85, dim*0.28)], fill=255, width=w)
    pts = [(dim*0.24, dim*0.34), (dim*0.28, dim*0.84), (dim*0.72, dim*0.84), (dim*0.76, dim*0.34)]
    d.line(pts, fill=255, width=w)
    d.line([(dim*0.42, dim*0.42), (dim*0.42, dim*0.74)], fill=255, width=w)
    d.line([(dim*0.58, dim*0.42), (dim*0.58, dim*0.74)], fill=255, width=w)

def draw_back(d, dim):
    w = int(dim * 0.12)
    c = dim / 2
    d.line([(dim*0.65, dim*0.18), (dim*0.30, c), (dim*0.65, dim*0.82)], fill=255, width=w)

def draw_check(d, dim):
    w = int(dim * 0.12)
    d.line([(dim*0.18, dim*0.52), (dim*0.42, dim*0.76), (dim*0.82, dim*0.24)], fill=255, width=w)

def draw_jog_up(d, dim):
    w = int(dim * 0.11)
    c = dim / 2
    d.line([(dim*0.20, dim*0.62), (c, dim*0.28), (dim*0.80, dim*0.62)], fill=255, width=w)

def draw_jog_down(d, dim):
    w = int(dim * 0.11)
    c = dim / 2
    d.line([(dim*0.20, dim*0.38), (c, dim*0.72), (dim*0.80, dim*0.38)], fill=255, width=w)

def draw_jog_left(d, dim):
    w = int(dim * 0.11)
    c = dim / 2
    d.line([(dim*0.62, dim*0.20), (dim*0.28, c), (dim*0.62, dim*0.80)], fill=255, width=w)

def draw_jog_right(d, dim):
    w = int(dim * 0.11)
    c = dim / 2
    d.line([(dim*0.38, dim*0.20), (dim*0.72, c), (dim*0.38, dim*0.80)], fill=255, width=w)

def draw_estop(d, dim):
    c = dim / 2
    w = int(dim * 0.09)
    r = dim * 0.44
    pts = []
    for i in range(8):
        angle = math.radians(22.5 + i * 45)
        pts.append((c + r * math.cos(angle), c + r * math.sin(angle)))
    d.polygon(pts, outline=255, fill=255)
    d.line([(c, dim*0.25), (c, dim*0.58)], fill=0, width=int(w*1.5))
    d.ellipse([c - dim*0.07, dim*0.68, c + dim*0.07, dim*0.82], fill=0)

# Shape vector icons for direct visual shape buttons
def draw_shape_circle(d, dim):
    w = int(dim * 0.11)
    d.ellipse([dim*0.14, dim*0.14, dim*0.86, dim*0.86], outline=255, width=w)

def draw_shape_square(d, dim):
    w = int(dim * 0.11)
    d.rounded_rectangle([dim*0.16, dim*0.16, dim*0.84, dim*0.84], radius=int(dim*0.08), outline=255, width=w)

def draw_shape_rect(d, dim):
    w = int(dim * 0.11)
    d.rounded_rectangle([dim*0.10, dim*0.26, dim*0.90, dim*0.74], radius=int(dim*0.08), outline=255, width=w)

def draw_shape_triangle(d, dim):
    w = int(dim * 0.11)
    c = dim / 2
    d.line([(dim*0.15, dim*0.82), (dim*0.85, dim*0.82), (c, dim*0.18), (dim*0.15, dim*0.82)], fill=255, width=w)

def draw_shape_hexagon(d, dim):
    c = dim / 2
    w = int(dim * 0.10)
    pts = []
    for i in range(6):
        a = math.radians(30 + i * 60)
        pts.append((c + dim*0.40 * math.cos(a), c + dim*0.40 * math.sin(a)))
    pts.append(pts[0])
    d.line(pts, fill=255, width=w)

def draw_theme(d, dim):
    c = dim / 2
    # Sun with rays on right, moon on left
    w = int(dim * 0.08)
    d.ellipse([c - dim*0.25, c - dim*0.25, c + dim*0.25, c + dim*0.25], outline=255, width=w)
    # Rays
    for deg in [0, 45, 90, 135, 180, 225, 270, 315]:
        rad = math.radians(deg)
        d.line([(c + dim*0.30*math.cos(rad), c + dim*0.30*math.sin(rad)),
                (c + dim*0.42*math.cos(rad), c + dim*0.42*math.sin(rad))], fill=255, width=w)

def draw_palette(d, dim):
    c = dim / 2
    w = int(dim * 0.08)
    # Palette kidney shape
    d.ellipse([dim*0.10, dim*0.15, dim*0.90, dim*0.85], outline=255, width=w)
    # 3 paint dots
    d.ellipse([dim*0.26, dim*0.32, dim*0.38, dim*0.44], fill=255)
    d.ellipse([dim*0.45, dim*0.25, dim*0.57, dim*0.37], fill=255)
    d.ellipse([dim*0.64, dim*0.35, dim*0.76, dim*0.47], fill=255)

def draw_light(d, dim):
    c = dim / 2
    w = int(dim * 0.08)
    # Bulb round dome
    d.arc([c - dim*0.26, dim*0.14, c + dim*0.26, dim*0.62], start=140, end=40, fill=255, width=w)
    # Tapering body lines down
    d.line([(c - dim*0.19, dim*0.52), (c - dim*0.12, dim*0.70)], fill=255, width=w)
    d.line([(c + dim*0.19, dim*0.52), (c + dim*0.12, dim*0.70)], fill=255, width=w)
    # Screw base ribs
    d.line([(c - dim*0.12, dim*0.70), (c + dim*0.12, dim*0.70)], fill=255, width=w)
    d.line([(c - dim*0.10, dim*0.78), (c + dim*0.10, dim*0.78)], fill=255, width=w)
    d.line([(c - dim*0.06, dim*0.86), (c + dim*0.06, dim*0.86)], fill=255, width=w)
    # Light rays emerging
    d.line([(c, dim*0.02), (c, dim*0.10)], fill=255, width=w)
    d.line([(dim*0.08, dim*0.24), (dim*0.17, dim*0.31)], fill=255, width=w)
    d.line([(dim*0.04, dim*0.46), (dim*0.13, dim*0.46)], fill=255, width=w)
    d.line([(dim*0.92, dim*0.24), (dim*0.83, dim*0.31)], fill=255, width=w)
    d.line([(dim*0.96, dim*0.46), (dim*0.87, dim*0.46)], fill=255, width=w)

def draw_rot_cw(d, dim):
    w = int(dim * 0.10)
    d.arc([dim*0.18, dim*0.18, dim*0.82, dim*0.82], start=45, end=305, fill=255, width=w)
    d.polygon([(dim*0.72, dim*0.12), (dim*0.92, dim*0.28), (dim*0.66, dim*0.34)], fill=255)

def draw_rot_ccw(d, dim):
    w = int(dim * 0.10)
    d.arc([dim*0.18, dim*0.18, dim*0.82, dim*0.82], start=-125, end=135, fill=255, width=w)
    d.polygon([(dim*0.28, dim*0.12), (dim*0.08, dim*0.28), (dim*0.34, dim*0.34)], fill=255)

def draw_star(d, dim):
    cx, cy = dim / 2, dim / 2
    r_out = dim * 0.42
    r_in = dim * 0.19
    pts = []
    for i in range(10):
        angle = -math.pi/2 + i * math.pi / 5
        r = r_out if i % 2 == 0 else r_in
        pts.append((cx + r * math.cos(angle), cy + r * math.sin(angle)))
    d.polygon(pts, fill=255)

ICONS = [
    # Launcher icons (25% larger: 28 * 1.25 = 35 px)
    ('focal_icon_launcher_control', draw_control, 35),
    ('focal_icon_launcher_files', draw_files, 35),
    ('focal_icon_launcher_monitor', draw_monitor, 35),
    ('focal_icon_launcher_calibration', draw_calibration, 35),
    ('focal_icon_launcher_shapes', draw_shapes, 35),
    ('focal_icon_launcher_settings', draw_settings, 35),

    ('focal_icon_control', draw_control, 28),
    ('focal_icon_files', draw_files, 24),
    ('focal_icon_monitor', draw_monitor, 28),
    ('focal_icon_calibration', draw_calibration, 28),
    ('focal_icon_shapes', draw_shapes, 24),
    ('focal_icon_settings', draw_settings, 22),
    ('focal_icon_wifi', draw_wifi, 24),
    ('focal_icon_laser', draw_laser, 24),
    ('focal_icon_home', draw_home, 22),
    ('focal_icon_play', draw_play, 20),
    ('focal_icon_pause', draw_pause, 20),
    ('focal_icon_stop', draw_stop, 20),
    ('focal_icon_refresh', draw_refresh, 20),
    ('focal_icon_trash', draw_trash, 20),
    ('focal_icon_back', draw_back, 20),
    ('focal_icon_check', draw_check, 20),
    ('focal_icon_jog_up', draw_jog_up, 22),
    ('focal_icon_jog_down', draw_jog_down, 22),
    ('focal_icon_jog_left', draw_jog_left, 22),
    ('focal_icon_jog_right', draw_jog_right, 22),
    ('focal_icon_estop', draw_estop, 24),
    # New icons
    ('focal_icon_shape_circle', draw_shape_circle, 24),
    ('focal_icon_shape_square', draw_shape_square, 24),
    ('focal_icon_shape_rect', draw_shape_rect, 24),
    ('focal_icon_shape_triangle', draw_shape_triangle, 24),
    ('focal_icon_shape_hexagon', draw_shape_hexagon, 24),
    ('focal_icon_theme', draw_theme, 22),
    ('focal_icon_palette', draw_palette, 22),
    ('focal_icon_light', draw_light, 22),
    ('focal_icon_rot_cw', draw_rot_cw, 22),
    ('focal_icon_rot_ccw', draw_rot_ccw, 22),
    ('focal_icon_star', draw_star, 20),
    ('focal_icon_wifi_med', draw_wifi_med, 24),
    ('focal_icon_wifi_low', draw_wifi_low, 24),
    ('focal_icon_wifi_dot', draw_wifi_dot, 24),
    ('focal_icon_info', draw_info, 22),
    ('focal_icon_bed', draw_bed, 22),
]

header_lines = [
    "/**",
    " * @file focal_icons.h",
    " * @brief Vector-rendered Apple-style anti-aliased icon descriptors for Focal CNC (LVGL v8)",
    " */",
    "#pragma once",
    "",
    '#include "lvgl.h"',
    "",
    "#ifdef __cplusplus",
    'extern "C" {',
    "#endif",
    "",
]

source_lines = [
    "/**",
    " * @file focal_icons.c",
    " * @brief Vector icon pixel data (LV_IMG_CF_ALPHA_8BIT)",
    " */",
    '#include "focal_icons.h"',
    "",
]

for name, fn, size in ICONS:
    img = render_icon(fn, size)
    data = list(img.tobytes())
    header_lines.append(f"extern const lv_img_dsc_t {name};")
    source_lines.append(f"static const uint8_t {name}_map[{len(data)}] = {{")
    
    for i in range(0, len(data), 16):
        chunk = data[i:i+16]
        hex_str = ", ".join(f"0x{b:02x}" for b in chunk)
        source_lines.append(f"    {hex_str},")
    source_lines.append("};")
    source_lines.append(f"const lv_img_dsc_t {name} = {{")
    source_lines.append("    .header.always_zero = 0,")
    source_lines.append(f"    .header.w = {size},")
    source_lines.append(f"    .header.h = {size},")
    source_lines.append(f"    .data_size = {size * size},")
    source_lines.append("    .header.cf = LV_IMG_CF_ALPHA_8BIT,")
    source_lines.append(f"    .data = {name}_map,")
    source_lines.append("};")
    source_lines.append("")

header_lines.append("")
header_lines.append("#ifdef __cplusplus")
header_lines.append("}")
header_lines.append("#endif")
header_lines.append("")

out_h = "components/ui/include/focal_icons.h"
out_c = "components/ui/focal_icons.c"

with open(out_h, "w") as f:
    f.write("\n".join(header_lines))

with open(out_c, "w") as f:
    f.write("\n".join(source_lines))

print(f"Generated {len(ICONS)} icons into {out_h} and {out_c}")
