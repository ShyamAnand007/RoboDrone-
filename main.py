"""
DRONE FLIGHT TRAINING SIMULATOR
Python + Ursina (Panda3D renderer)

Features
  * 6-axis quadcopter model: throttle, pitch, roll, yaw
  * Motor lag, battery voltage sag, ground effect, quadratic air drag, wind + gusts
  * Crash detection: hard landing, tilted landing, fast skid, building/tree impact
  * City of 30 procedural buildings, roads, parks, trees, hills, clouds
  * 8 flight rings (combo scoring) + 2 precision landing pads
  * Dynamic sun shadows, fog, MSAA, day / night mode, neon lighting
  * Chase / FPV / orbit cameras with speed-based FOV and crash shake
  * Full HUD, mini-map, explosion + debris + smoke effects

Set the environment variable DRONE_LOW=1 to disable shadows on weak GPUs.
"""
import math
import os
import random

from panda3d.core import AntialiasAttrib, loadPrcFileData

# QUALITY: 0 = low (fastest), 1 = medium (default), 2 = high.  Set with:  set DRONE_QUALITY=0
QUALITY = int(os.environ.get("DRONE_QUALITY", "1"))
if os.environ.get("DRONE_LOW") == "1":
    QUALITY = 0
_msaa = {0: 0, 1: 2, 2: 4}[QUALITY]
if _msaa:
    loadPrcFileData("", f"framebuffer-multisample 1\nmultisamples {_msaa}\n")

from ursina import *                                   # noqa: E402,F403
from ursina.shaders import lit_with_shadows_shader, unlit_shader  # noqa: E402

LOW_QUALITY = QUALITY == 0


# ----------------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------------
def C(r, g, b, a=255):
    """0-255 colour helper (independent of Ursina version)."""
    return Color(r / 255, g / 255, b / 255, a / 255)


def approach(cur, target, rate, dt):
    return cur + (target - cur) * min(1.0, rate * dt)


def glow(**kw):
    """Emissive (unlit) entity."""
    kw.setdefault("shader", unlit_shader)
    return Entity(**kw)


# ----------------------------------------------------------------------------
# constants
# ----------------------------------------------------------------------------
G = 9.81
MASS = 1.0
MAX_THRUST = 2.0 * G * MASS          # 50 % throttle == hover
DRONE_R = 0.45                       # collision radius
CLEAR = 0.20                         # origin -> skid bottom
TILT_MAX = 34.0                      # max pitch / roll (deg)
CRASH_VSPEED = 4.5                   # m/s
CRASH_TILT = 15.0                    # deg
CRASH_HSPEED = 8.0                   # m/s
K_LIN, K_QUAD = 0.30, 0.055          # drag coefficients

DAY_FOG = C(176, 205, 232)
NIGHT_FOG = C(10, 14, 32)
SUN_DIR = Vec3(1, -1.6, 0.7).normalized()

# ----------------------------------------------------------------------------
# app / window
# ----------------------------------------------------------------------------
app = Ursina(title="Drone Flight Training Simulator", borderless=False,
             vsync=False, development_mode=False, editor_ui_enabled=False)
window.exit_button.visible = False
window.fps_counter.enabled = True
window.color = DAY_FOG
mouse.visible = False
camera.clip_plane_far = 3000
camera.clip_plane_near = 0.05
try:
    if QUALITY:
        base.render.setAntialias(AntialiasAttrib.MAuto)  # noqa: F821
except Exception:
    pass

# ----------------------------------------------------------------------------
# lighting / atmosphere
# ----------------------------------------------------------------------------
sky = Sky()
ambient = AmbientLight(color=C(125, 135, 155))
sun = DirectionalLight(shadows=not LOW_QUALITY,
                       shadow_map_resolution=Vec2(*{1: (1536, 1536), 2: (3072, 3072)}.get(QUALITY, (1024, 1024))))
sun.look_at(SUN_DIR)
sun.color = C(255, 238, 210)
try:
    lens = sun._light.get_lens()
    lens.set_film_size(130 if QUALITY == 1 else 170, 130 if QUALITY == 1 else 170)
    lens.set_near_far(1, 340)
except Exception:
    pass
scene.fog_color = DAY_FOG
scene.fog_density = (70, 900)

if not LOW_QUALITY:
    Entity.default_shader = lit_with_shadows_shader     # world uses shadows

# ----------------------------------------------------------------------------
# world data (filled by build_world)
# ----------------------------------------------------------------------------
rng = random.Random(2024)
BUILDINGS = []      # (cx, cz, half_w, half_d, top_y)
TREES = []          # (x, z, radius, top_y)
PADS = [{"name": "HOME", "x": 0.0, "z": 0.0},
        {"name": "PAD B", "x": 100.0, "z": 100.0}]
RINGS = []
BEACONS = []
NEON = []
CLOUDS = []

RING_DEFS = [   # (x, y, z, yaw_deg)  -> ring plane normal follows yaw
    (0, 6, 30, 0), (0, 10, 70, 0), (0, 14, 105, 0), (40, 12, 120, 90),
    (80, 9, 120, 90), (120, 12, 90, 0), (120, 8, 40, 0), (80, 6, 0, 90),
]


def bead_ring(parent, radius, n, size, col):
    beads = []
    for i in range(n):
        a = 2 * math.pi * i / n
        beads.append(glow(parent=parent, model="sphere", scale=size, color=col,
                          position=(math.cos(a) * radius, math.sin(a) * radius, 0)))
    return beads


def make_building(cx, cz, w, d, h, col, neon_col):
    base = 0.12
    Entity(model="cube", position=(cx, base + h / 2, cz), scale=(w, h, d), color=col,
           texture="white_cube", texture_scale=(w / 3.2, h / 3.2))
    Entity(model="cube", position=(cx, base + h + 0.3, cz), scale=(w + 0.7, 0.6, d + 0.7),
           color=C(38, 40, 46))
    Entity(model="cube", position=(cx + w * 0.15, base + h + 1.1, cz - d * 0.1),
           scale=(w * 0.3, 1.6, d * 0.3), color=C(70, 74, 82))
    band = glow(model="cube", position=(cx, base + h * 0.72, cz),
                scale=(w + 0.2, 0.45, d + 0.2), color=neon_col)
    NEON.append(band)
    if h > 34:
        Entity(model="cube", position=(cx, base + h + 4.5, cz), scale=(0.25, 7, 0.25),
               color=C(90, 90, 96))
        BEACONS.append(glow(model="sphere", position=(cx, base + h + 8.2, cz), scale=0.6,
                            color=C(255, 40, 40)))
    BUILDINGS.append((cx, cz, w / 2, d / 2, base + h))


def make_tree(x, z, s=1.0):
    th = 3.2 * s
    Entity(model="cube", position=(x, th / 2, z), scale=(0.55 * s, th, 0.55 * s),
           color=C(84, 58, 38))
    g = rng.randint(90, 150)
    Entity(model="sphere", position=(x, th + 1.8 * s, z), scale=(3.8 * s, 4.8 * s, 3.8 * s),
           color=C(38, g, 52))
    TREES.append((x, z, 1.6 * s, th + 3.6 * s))


def draw_letter(parent, letter, col):
    def bar(x, z, w, d):
        Entity(parent=parent, model="cube", position=(x, 0.11, z), scale=(w, 0.03, d), color=col)
    if letter == "H":
        bar(-1.2, 0, 0.5, 4.0)
        bar(1.2, 0, 0.5, 4.0)
        bar(0, 0, 2.3, 0.5)
    else:  # B
        bar(-1.1, 0, 0.5, 4.0)
        bar(0.0, 1.75, 2.1, 0.5)
        bar(0.0, 0, 2.1, 0.5)
        bar(0.0, -1.75, 2.1, 0.5)
        bar(1.1, 0.9, 0.5, 1.9)
        bar(1.1, -0.9, 0.5, 1.9)


def make_pad(pad, col):
    root = Entity(position=(pad["x"], 0, pad["z"]))
    Entity(parent=root, model="cube", position=(0, 0.05, 0), scale=(10, 0.1, 10), color=C(36, 38, 44))
    draw_letter(root, "H" if pad["name"] == "HOME" else "B", col)
    lights = []
    NL = (12, 16, 28)[QUALITY]
    for i in range(NL):
        a = 2 * math.pi * i / NL
        lights.append(glow(parent=root, model="sphere", scale=0.28, color=col,
                           position=(math.cos(a) * 4.6, 0.16, math.sin(a) * 4.6)))
    pad["lights"] = lights


def build_world():
    # ground
    Entity(model="plane", scale=(1800, 1, 1800), texture="grass", texture_scale=(360, 360),
           color=C(118, 150, 104))
    # roads
    asphalt = C(44, 46, 52)
    for k in range(-3, 4):
        v = k * 40
        Entity(model="cube", position=(v, 0.04, 0), scale=(12, 0.08, 280), color=asphalt)
        Entity(model="cube", position=(0, 0.03, v), scale=(280, 0.06, 12), color=asphalt)
    for k in (range(-3, 4) if QUALITY else []):
        v = k * 40
        for s in range(-3, 3):
            mid = s * 40 + 20
            Entity(model="cube", position=(v, 0.095, mid), scale=(0.25, 0.02, 20),
                   color=C(230, 200, 60))
            Entity(model="cube", position=(mid, 0.075, v), scale=(20, 0.02, 0.25),
                   color=C(230, 200, 60))

    # city cells
    cells = [(kx, kz) for kx in range(-3, 3) for kz in range(-3, 3)]
    pad_cell = (2, 2)
    others = [c for c in cells if c != pad_cell]
    rng.shuffle(others)
    parks = [pad_cell] + others[:5]
    build_cells = others[5:]                      # exactly 30 buildings

    palette = [C(150, 155, 165), C(120, 130, 145), C(175, 165, 150), C(95, 110, 130),
               C(160, 120, 105), C(110, 125, 118), C(190, 190, 195)]
    neons = [C(0, 220, 255), C(255, 60, 200), C(120, 255, 90), C(255, 170, 30)]
    for (kx, kz) in build_cells:
        cx, cz = kx * 40 + 20, kz * 40 + 20
        Entity(model="cube", position=(cx, 0.06, cz), scale=(28, 0.12, 28), color=C(150, 150, 152))
        w, d = rng.uniform(13, 21), rng.uniform(13, 21)
        h = rng.choice([rng.uniform(12, 24), rng.uniform(22, 40), rng.uniform(38, 58)])
        make_building(cx, cz, w, d, h, rng.choice(palette), rng.choice(neons))

    for (kx, kz) in parks:
        cx, cz = kx * 40 + 20, kz * 40 + 20
        Entity(model="cube", position=(cx, 0.06, cz), scale=(28, 0.12, 28), color=C(82, 130, 70))
        if (kx, kz) == pad_cell:
            for sx in (-1, 1):
                for sz in (-1, 1):
                    make_tree(cx + sx * 10.5, cz + sz * 10.5, rng.uniform(0.9, 1.3))
        else:
            for _ in range(rng.randint(4, 7)):
                make_tree(cx + rng.uniform(-10, 10), cz + rng.uniform(-10, 10), rng.uniform(0.8, 1.4))

    # outskirts trees
    placed = 0
    while placed < (25, 60, 120)[QUALITY]:
        x, z = rng.uniform(-330, 330), rng.uniform(-330, 330)
        if abs(x) < 145 and abs(z) < 145:
            continue
        make_tree(x, z, rng.uniform(0.9, 1.8))
        placed += 1

    # pads
    make_pad(PADS[0], C(255, 150, 30))
    make_pad(PADS[1], C(0, 220, 255))

    # rings
    for (x, y, z, yaw) in RING_DEFS:
        holder = Entity(position=(x, y, z), rotation_y=yaw)
        beads = bead_ring(holder, 4.4, (20, 26, 44)[QUALITY], 0.7 if QUALITY < 2 else 0.55, C(255, 140, 20))
        inner = bead_ring(holder, 3.7, 30, 0.22, C(255, 220, 120)) if QUALITY == 2 else []
        disc = glow(parent=holder, model="quad", texture="circle", double_sided=True,
                    scale=8.4, color=C(255, 150, 30, 38))
        r = math.radians(yaw)
        RINGS.append({"pos": Vec3(x, y, z), "n": Vec3(math.sin(r), 0, math.cos(r)),
                      "beads": beads + inner, "disc": disc, "holder": holder, "passed": False})

    # hills on the horizon
    for i in range(10):
        a = 2 * math.pi * i / 10 + rng.uniform(-0.15, 0.15)
        dist = rng.uniform(560, 700)
        s = rng.uniform(160, 300)
        Entity(model="sphere", position=(math.cos(a) * dist, -10, math.sin(a) * dist),
               scale=(s, rng.uniform(70, 150), s), color=C(72, 100, 96))

    # clouds
    for _ in range((5, 9, 16)[QUALITY]):
        root = Entity(position=(rng.uniform(-500, 500), rng.uniform(110, 170), rng.uniform(-500, 500)))
        for _ in range(3 if QUALITY < 2 else 4):
            glow(parent=root, model="sphere", color=C(255, 255, 255, 215),
                 position=(rng.uniform(-25, 25), rng.uniform(-3, 3), rng.uniform(-14, 14)),
                 scale=(rng.uniform(30, 55), rng.uniform(9, 16), rng.uniform(20, 32)))
        CLOUDS.append(root)


build_world()

# ----------------------------------------------------------------------------
# drone model
# ----------------------------------------------------------------------------
drone = Entity()
ROTORS = []
LEDS = []


def build_drone():
    dark, mid, accent = C(26, 28, 34), C(64, 68, 78), C(255, 110, 25)
    Entity(parent=drone, model="cube", scale=(0.42, 0.12, 0.62), color=dark)
    Entity(parent=drone, model="sphere", position=(0, 0.07, -0.02), scale=(0.34, 0.15, 0.52), color=accent)
    Entity(parent=drone, model="cube", position=(0, -0.07, -0.02), scale=(0.3, 0.08, 0.36), color=mid)
    Entity(parent=drone, model="sphere", position=(0, -0.05, 0.33), scale=0.13, color=dark)
    glow(parent=drone, model="sphere", position=(0, -0.05, 0.395), scale=0.055, color=C(80, 220, 255))
    for rot in (45, -45):
        Entity(parent=drone, model="cube", rotation_y=rot, scale=(0.08, 0.05, 1.7), color=mid)
    for sx in (-1, 1):
        for sz in (-1, 1):
            px, pz = sx * 0.6, sz * 0.6
            Entity(parent=drone, model="cube", position=(px, 0.05, pz), scale=(0.15, 0.13, 0.15), color=dark)
            holder = Entity(parent=drone, position=(px, 0.13, pz))
            Entity(parent=holder, model="cube", scale=(0.66, 0.012, 0.055), color=C(210, 214, 222))
            Entity(parent=holder, model="cube", scale=(0.055, 0.012, 0.66), color=C(210, 214, 222))
            Entity(parent=holder, model="sphere", scale=0.07, color=accent)
            disc = glow(parent=drone, model="sphere", position=(px, 0.135, pz),
                        scale=(0.78, 0.004, 0.78), color=C(200, 220, 255, 0))
            ROTORS.append({"h": holder, "d": disc, "dir": 1 if sx * sz > 0 else -1})
            col = C(255, 30, 30) if sz > 0 else C(40, 255, 90)
            LEDs = glow(parent=drone, model="sphere", position=(px * 1.18, 0.0, pz * 1.18), scale=0.07, color=col)
            LEDS.append((LEDs, sz > 0, col))
    for sx in (-1, 1):
        Entity(parent=drone, model="cube", position=(sx * 0.2, -0.175, 0), scale=(0.035, 0.03, 0.62), color=dark)
        for sz in (-1, 1):
            Entity(parent=drone, model="cube", position=(sx * 0.2, -0.115, sz * 0.2),
                   scale=(0.03, 0.12, 0.03), color=dark)


build_drone()
blob = Entity(model="quad", texture="radial_gradient", double_sided=True, rotation_x=90,
              color=C(0, 0, 0, 150), scale=1.6, shader=unlit_shader)

Entity.default_shader = None      # everything after this is UI / effects

# calibrate Euler sign conventions so that +pitch -> thrust toward +Z, +roll -> +X
_t = Entity()
_t.rotation = Vec3(20, 0, 0)
PITCH_SIGN = 1 if _t.up.z > 0 else -1
_t.rotation = Vec3(0, 0, 20)
ROLL_SIGN = 1 if _t.up.x > 0 else -1
destroy(_t)

# ----------------------------------------------------------------------------
# HUD
# ----------------------------------------------------------------------------
HUD_COL = C(120, 255, 200)


def txt(text="", pos=(0, 0), scale=1.0, col=HUD_COL, origin=(-.5, .5)):
    return Text(text=text, position=pos, origin=origin, scale=scale, color=col)


tl = window.top_left
hud_left = txt(pos=(tl[0] + 0.03, tl[1] - 0.03), scale=1.05)
hud_score = txt(pos=(0, 0.47), scale=1.5, col=C(255, 220, 90), origin=(0, .5))
hud_next = txt(pos=(0, 0.425), scale=1.0, col=C(255, 170, 60), origin=(0, .5))
hud_toast = txt(pos=(0, 0.3), scale=2.2, col=color.white, origin=(0, 0))
hud_warn = txt(pos=(0, 0.2), scale=1.6, col=C(255, 60, 60), origin=(0, 0))
hud_cam = txt(pos=(window.bottom_right[0] - 0.03, window.bottom_right[1] + 0.045), scale=0.9,
              col=C(200, 220, 255), origin=(.5, -.5))

vign = Entity(parent=camera.ui, model="quad", texture="vignette", color=C(0, 0, 0, 170),
              scale=(window.aspect_ratio * 1.02, 1.02), z=3)


class Bar:
    def __init__(self, x, y, w, h, col, label):
        self.h = h
        Entity(parent=camera.ui, model="quad", position=(x, y), scale=(w, h), color=C(0, 0, 0, 150))
        self.fill = Entity(parent=camera.ui, model="quad", color=col, origin=(0, -.5),
                           position=(x, y - h / 2 + 0.004), scale=(w * 0.7, 0.001), z=-0.01)
        txt(label, (x, y - h / 2 - 0.008), 0.85, col=C(220, 230, 240), origin=(0, .5))

    def set(self, v, col=None):
        self.fill.scale_y = max(0.001, (self.h - 0.008) * clamp(v, 0, 1))
        if col is not None:
            self.fill.color = col


bar_thr = Bar(tl[0] + 0.05, -0.18, 0.035, 0.36, C(90, 200, 255), "THR")
bar_bat = Bar(tl[0] + 0.12, -0.18, 0.035, 0.36, C(120, 255, 120), "BAT")

# mini-map
MM_SIZE = 0.30
MM_SCALE = MM_SIZE / 300.0
tr = window.top_right
mm = Entity(parent=camera.ui, position=(tr[0] - MM_SIZE / 2 - 0.03, tr[1] - MM_SIZE / 2 - 0.03))
Entity(parent=mm, model="quad", scale=MM_SIZE, color=C(10, 20, 30, 170))
for (cx, cz, hw, hd, top) in BUILDINGS:
    Entity(parent=mm, model="quad", position=(cx * MM_SCALE, cz * MM_SCALE, -0.001),
           scale=(hw * 2 * MM_SCALE, hd * 2 * MM_SCALE), color=C(90, 110, 140, 200))
for p in PADS:
    Entity(parent=mm, model="quad", position=(p["x"] * MM_SCALE, p["z"] * MM_SCALE, -0.002),
           scale=0.014, color=C(0, 220, 255))
mm_rings = [Entity(parent=mm, model="quad", position=(r["pos"].x * MM_SCALE, r["pos"].z * MM_SCALE, -0.003),
                   scale=0.011, color=C(255, 140, 20)) for r in RINGS]
mm_drone = Entity(parent=mm, model="quad", scale=0.014, color=color.white, z=-0.005)
mm_head = Entity(parent=mm, model="quad", scale=0.008, color=C(255, 230, 60), z=-0.006)

help_panel = Entity(parent=camera.ui, z=-0.5)
Entity(parent=help_panel, model="quad", scale=(0.86, 0.66), color=C(4, 10, 18, 215))
help_text = Text(parent=help_panel, text=(
    "DRONE FLIGHT TRAINING SIMULATOR\n\n"
    "SPACE / L-SHIFT   throttle up / down      (hover ~ 50%)\n"
    "W / S  (or arrows)   pitch forward / back\n"
    "A / D              roll left / right\n"
    "Q / E              yaw left / right\n\n"
    "R  reset     C  camera     Z  hover assist\n"
    "T  wind      N  night      H  this help      ESC  quit\n\n"
    "Fly through the orange rings (combos = bonus).\n"
    "Land softly on HOME or PAD B for precision points.\n"
    "Crash: >4.5 m/s descent, >15 deg tilt, walls, trees."),
    origin=(0, 0), position=(0, 0, -0.01), scale=1.0, color=C(190, 255, 235))

# ----------------------------------------------------------------------------
# simulation state
# ----------------------------------------------------------------------------
class State:
    pass


S = State()
S.best = 0
S.cam_mode = 0
S.assist = False
S.wind_on = True
S.night = False
S.t = 0.0
S.orbit = 0.0
S.toast_t = 0.0
S.hud_t = 0.0
S.trail_t = 0.0
S.smoke_t = 0.0
S.help_t = 9.0
DEBRIS = []
CAM_NAMES = ["CHASE", "FPV", "ORBIT"]


def toast(msg, col=None, dur=2.4):
    hud_toast.text = msg
    hud_toast.color = col if col is not None else color.white
    S.toast_t = dur
    S.toast_col = hud_toast.color


def reset():
    S.pos = Vec3(PADS[0]["x"], CLEAR, PADS[0]["z"])
    S.prev = Vec3(S.pos)
    S.vel = Vec3(0, 0, 0)
    S.pitch = S.roll = S.yaw = S.yawrate = 0.0
    S.throttle = S.motor = 0.0
    S.batt = 1.0
    S.grounded = True
    S.crashed = False
    S.crash_msg = ""
    S.score = 0
    S.combo = 0
    S.last_ring_t = -99.0
    S.rings_done = 0
    S.flight_t = 0.0
    S.finished = False
    S.shake = 0.0
    S.armed_pad = {p["name"]: False for p in PADS}
    S.cam_pos = S.pos + Vec3(0, 3, -8)
    for r in RINGS:
        r["passed"] = False
        for b in r["beads"]:
            b.color = C(255, 140, 20) if b.scale_x > 0.4 else C(255, 220, 120)
        r["disc"].color = C(255, 150, 30, 38)
    for d in mm_rings:
        d.color = C(255, 140, 20)
    for e, *_ in DEBRIS:
        destroy(e)
    DEBRIS.clear()
    drone.enabled = True
    blob.enabled = True
    drone.position = S.pos
    drone.rotation = Vec3(0, 0, 0)
    toast("READY  -  hold SPACE to lift off", C(120, 255, 200), 3.0)


# ----------------------------------------------------------------------------
# effects
# ----------------------------------------------------------------------------
def explode(p):
    fire = glow(model="sphere", position=p, scale=0.6, color=C(255, 170, 40, 235))
    fire.animate_scale(8, duration=0.45, curve=curve.out_expo)
    fire.fade_out(duration=0.7, delay=0.15)
    destroy(fire, delay=1.0)
    core = glow(model="sphere", position=p, scale=0.4, color=C(255, 255, 230, 255))
    core.animate_scale(4, duration=0.25, curve=curve.out_expo)
    core.fade_out(duration=0.35, delay=0.05)
    destroy(core, delay=0.6)
    cols = [C(26, 28, 34), C(64, 68, 78), C(255, 110, 25), C(210, 214, 222)]
    for _ in range(30):
        s = rng.uniform(0.08, 0.26)
        e = Entity(model="cube", position=p, scale=(s, s * 0.5, s * rng.uniform(1, 2.2)),
                   color=rng.choice(cols), shader=unlit_shader)
        v = Vec3(rng.uniform(-9, 9), rng.uniform(3, 13), rng.uniform(-9, 9))
        DEBRIS.append([e, v, Vec3(rng.uniform(-400, 400), rng.uniform(-400, 400), rng.uniform(-400, 400)),
                       rng.uniform(5, 9)])


def puff(p, col, size, grow, rise, life):
    e = glow(model="sphere", position=p, scale=size, color=col)
    e.animate_scale(grow, duration=life)
    e.animate_position(p + Vec3(rng.uniform(-1, 1), rise, rng.uniform(-1, 1)), duration=life)
    e.fade_out(duration=life)
    destroy(e, delay=life + 0.05)


def do_crash(reason):
    S.crashed = True
    S.crash_msg = reason
    S.shake = 1.0
    S.vel = Vec3(0, 0, 0)
    S.smoke_t = 7.0
    explode(Vec3(S.pos))
    drone.enabled = False
    blob.enabled = False
    S.combo = 0
    toast("CRASHED  -  " + reason, C(255, 70, 70), 4.0)


# ----------------------------------------------------------------------------
# physics
# ----------------------------------------------------------------------------
def wind_vector():
    if not S.wind_on:
        return Vec3(0, 0, 0)
    t = S.t
    base = 2.2 + 1.2 * math.sin(t * 0.21)
    gust = 1.3 * math.sin(t * 1.7) * math.sin(t * 0.43)
    return Vec3(math.sin(0.35) * (base + gust), 0.25 * math.sin(t * 0.9), math.cos(0.35) * (base + gust))


def query_world(p):
    support, hit = 0.0, False
    for (cx, cz, hw, hd, top) in BUILDINGS:
        if abs(p.x - cx) < hw + DRONE_R and abs(p.z - cz) < hd + DRONE_R:
            if p.y >= top - 0.05:
                support = max(support, top)
            else:
                hit = True
    if not hit:
        for (tx, tz, tr_, top) in TREES:
            dx, dz = p.x - tx, p.z - tz
            if dx * dx + dz * dz < (tr_ + DRONE_R * 0.5) ** 2 and p.y < top:
                hit = True
                break
    return support, hit


def tilt_deg():
    return math.degrees(math.acos(clamp(drone.up.y, -1, 1)))


def axis(pos_keys, neg_keys):
    v = 0
    for k in pos_keys:
        v += held_keys[k]
    for k in neg_keys:
        v -= held_keys[k]
    return clamp(v, -1, 1)


def try_award_landing(support):
    if support > 0.5:
        return
    for p in PADS:
        dx, dz = S.pos.x - p["x"], S.pos.z - p["z"]
        if abs(dx) < 4.8 and abs(dz) < 4.8 and S.armed_pad[p["name"]]:
            dist = math.hypot(dx, dz)
            pts = 100 + int(max(0.0, 1 - dist / 5.0) * 400)
            S.score += pts
            S.armed_pad[p["name"]] = False
            label = "PERFECT" if dist < 1 else "GOOD" if dist < 2.5 else "OK"
            toast(f"{label} LANDING on {p['name']}  +{pts}", C(90, 255, 160), 3.0)
            return


def step_physics(dt):
    # ---- controls -------------------------------------------------------
    thr_up = held_keys["space"]
    thr_dn = held_keys["left shift"] or held_keys["right shift"]
    cos_t = max(0.5, drone.up.y)
    sag = 0.82 + 0.18 * S.batt
    if thr_up:
        S.throttle += 0.6 * dt
    elif thr_dn:
        S.throttle -= 0.8 * dt
    elif S.assist:
        want = 0.5 / cos_t / sag - S.vel.y * 0.07
        S.throttle = approach(S.throttle, want, 6, dt)
    S.throttle = clamp(S.throttle, 0, 1)

    cmd = S.throttle if S.batt > 0 else 0.0
    S.motor = approach(S.motor, cmd, 1 / 0.07, dt)

    lim = TILT_MAX
    if S.grounded and S.motor < 0.45:
        lim = 0.0
    kp = axis(("w", "up arrow"), ("s", "down arrow"))
    kr = axis(("d", "right arrow"), ("a", "left arrow"))
    ky = axis(("e",), ("q",))
    wob_p = wob_r = 0.0
    if not S.grounded and S.wind_on:
        wob_p = 1.6 * math.sin(S.t * 2.3) * math.sin(S.t * 0.7)
        wob_r = 1.6 * math.sin(S.t * 1.9 + 1) * math.sin(S.t * 0.6)
    S.pitch = approach(S.pitch, kp * lim + wob_p, 6.5, dt)
    S.roll = approach(S.roll, kr * lim + wob_r, 6.5, dt)
    S.yawrate = approach(S.yawrate, ky * 115, 8, dt)
    S.yaw = (S.yaw + S.yawrate * dt) % 360
    drone.rotation = Vec3(S.pitch * PITCH_SIGN, S.yaw, S.roll * ROLL_SIGN)
    up = drone.up

    # ---- battery ----------------------------------------------------------
    if S.motor > 0.03 or not S.grounded:
        S.batt = max(0.0, S.batt - dt * (0.0004 + 0.0055 * S.motor ** 2))

    # ---- integrate ----------------------------------------------------------
    wind = wind_vector()
    n = max(1, math.ceil(dt / (1 / 120)))
    h = dt / n
    for _ in range(n):
        support, hit = query_world(S.pos)
        alt = S.pos.y - CLEAR - support
        ge = 1.0 + 0.25 * max(0.0, 1.0 - alt / 1.5)
        thrust = S.motor * MAX_THRUST * sag * ge
        acc = up * (thrust / MASS) + Vec3(0, -G, 0)
        vrel = S.vel - wind
        acc -= vrel * (K_LIN + K_QUAD * vrel.length())
        S.vel += acc * h
        S.prev = Vec3(S.pos)
        S.pos += S.vel * h

        support, hit = query_world(S.pos)
        if hit:
            do_crash("collision with obstacle")
            return
        floor = support + CLEAR
        if S.pos.y <= floor:
            if not S.grounded:
                hspd = math.hypot(S.vel.x, S.vel.z)
                tilt = tilt_deg()
                if -S.vel.y > CRASH_VSPEED:
                    do_crash(f"hard landing ({-S.vel.y:.1f} m/s)")
                    return
                if tilt > CRASH_TILT:
                    do_crash(f"tilted landing ({tilt:.0f} deg)")
                    return
                if hspd > CRASH_HSPEED:
                    do_crash(f"landed too fast ({hspd:.1f} m/s)")
                    return
                try_award_landing(support)
            S.pos.y = floor
            if S.vel.y < 0:
                S.vel.y = 0
            damp = max(0.0, 1 - 5.0 * h)
            S.vel.x *= damp
            S.vel.z *= damp
            S.grounded = True
        else:
            S.grounded = False
        if S.pos.y > 300:
            S.pos.y = 300
            S.vel.y = min(S.vel.y, 0)
        if abs(S.pos.x) > 900 or abs(S.pos.z) > 900:
            S.vel = Vec3(0, S.vel.y, 0)
            S.pos.x = clamp(S.pos.x, -900, 900)
            S.pos.z = clamp(S.pos.z, -900, 900)

    drone.position = S.pos
    if not S.grounded:
        S.flight_t += dt
        for p in PADS:
            if S.pos.y > 3.5:
                S.armed_pad[p["name"]] = True
    elif S.motor > 0.15:
        S.flight_t += dt


def check_rings():
    for i, r in enumerate(RINGS):
        if r["passed"]:
            continue
        d0 = (S.prev - r["pos"]).dot(r["n"])
        d1 = (S.pos - r["pos"]).dot(r["n"])
        if d0 * d1 < 0:
            k = d0 / (d0 - d1)
            ip = S.prev + (S.pos - S.prev) * k
            if (ip - r["pos"]).length() < 4.0:
                r["passed"] = True
                S.rings_done += 1
                S.combo = S.combo + 1 if S.flight_t - S.last_ring_t < 16 else 1
                S.last_ring_t = S.flight_t
                pts = 100 * min(S.combo, 5)
                S.score += pts
                for b in r["beads"]:
                    b.color = C(70, 255, 120)
                r["disc"].color = C(70, 255, 120, 45)
                mm_rings[i].color = C(70, 255, 120)
                msg = f"RING {S.rings_done}/{len(RINGS)}  +{pts}"
                if S.combo > 1:
                    msg += f"   COMBO x{min(S.combo, 5)}"
                toast(msg, C(120, 255, 160), 1.8)
                if S.rings_done == len(RINGS):
                    bonus = max(0, 800 - int(S.flight_t * 4))
                    S.score += bonus
                    S.finished = True
                    toast(f"COURSE COMPLETE!  time bonus +{bonus}", C(255, 230, 90), 5.0)
                break


# ----------------------------------------------------------------------------
# camera
# ----------------------------------------------------------------------------
def update_camera(dt):
    yaw = math.radians(S.yaw)
    fwd = Vec3(math.sin(yaw), 0, math.cos(yaw))
    speed = S.vel.length()
    mode = S.cam_mode
    if mode == 0:
        tgt = S.pos - fwd * 7.0 + Vec3(0, 2.6, 0) - S.vel * 0.05
        S.cam_pos = S.cam_pos + (tgt - S.cam_pos) * min(1.0, dt * 5.0)
        S.cam_pos.y = max(S.cam_pos.y, 0.6)
        camera.position = S.cam_pos
        camera.look_at(S.pos + Vec3(0, 0.6, 0))
        base_fov = 78
    elif mode == 1:
        camera.position = S.pos + drone.up * 0.09 + drone.forward * 0.33
        camera.rotation = Vec3(drone.rotation_x - 22, drone.rotation_y, drone.rotation_z)
        base_fov = 108
    else:
        S.orbit += dt * 14
        a = math.radians(S.orbit)
        camera.position = S.pos + Vec3(math.sin(a) * 38, 24, math.cos(a) * 38)
        camera.look_at(S.pos)
        base_fov = 62
    if S.shake > 0.01:
        camera.position += Vec3(rng.uniform(-1, 1), rng.uniform(-1, 1), rng.uniform(-1, 1)) * S.shake * 0.5
        S.shake = approach(S.shake, 0, 3, dt)
    camera.fov = approach(camera.fov, base_fov + min(speed, 14) * 0.9, 3, dt)


# ----------------------------------------------------------------------------
# HUD update
# ----------------------------------------------------------------------------
def update_hud(dt):
    S.hud_t += dt
    up_speed = S.vel.length()
    if S.hud_t > 0.06:
        S.hud_t = 0
        alt = max(0.0, S.pos.y - CLEAR)
        hspd = math.hypot(S.vel.x, S.vel.z)
        hud_left.text = (f"ALT   {alt:6.1f} m\n"
                         f"SPD   {hspd:6.1f} m/s  ({hspd * 3.6:4.0f} km/h)\n"
                         f"V/S   {S.vel.y:+6.1f} m/s\n"
                         f"HDG   {S.yaw:6.0f} deg\n"
                         f"TILT  {tilt_deg() if drone.enabled else 0:6.1f} deg\n"
                         f"TIME  {S.flight_t:6.1f} s\n"
                         f"ASSIST {'ON' if S.assist else 'off'}   "
                         f"WIND {'ON' if S.wind_on else 'off'}")
        hud_score.text = f"SCORE {S.score}   BEST {max(S.best, S.score)}   RINGS {S.rings_done}/{len(RINGS)}"
        hud_cam.text = f"CAM: {CAM_NAMES[S.cam_mode]}   [C]    {int(1 / max(dt, 1e-4))} fps"
        # next ring guidance
        best, bd = None, 1e9
        for r in RINGS:
            if not r["passed"]:
                d = (r["pos"] - S.pos).length()
                if d < bd:
                    best, bd = r, d
        if best is None:
            hud_next.text = "ALL RINGS CLEARED - land on a pad!" if S.rings_done else ""
        else:
            dx, dz = best["pos"].x - S.pos.x, best["pos"].z - S.pos.z
            rel = (math.degrees(math.atan2(dx, dz)) - S.yaw + 540) % 360 - 180
            arrow = ("<<" if rel < -60 else "<" if rel < -12 else ">>" if rel > 60 else ">" if rel > 12 else "^")
            vert = "UP" if best["pos"].y - S.pos.y > 2.5 else "DOWN" if best["pos"].y - S.pos.y < -2.5 else "LEVEL"
            hud_next.text = f"NEXT RING  {bd:5.0f} m   {arrow}   {vert}"
        warn = ""
        blink = int(S.t * 4) % 2 == 0
        if S.crashed:
            warn = "PRESS R TO RESET"
        elif S.batt <= 0:
            warn = "BATTERY DEAD"
        elif S.batt < 0.15 and blink:
            warn = "LOW BATTERY"
        elif not S.grounded and S.vel.y < -CRASH_VSPEED and S.pos.y < 12 and blink:
            warn = "SINK RATE!"
        hud_warn.text = warn
    bar_thr.set(S.throttle, C(90, 200, 255))
    bar_bat.set(S.batt, C(120, 255, 120) if S.batt > 0.3 else C(255, 80, 60))
    mm_drone.position = (S.pos.x * MM_SCALE, S.pos.z * MM_SCALE, -0.005)
    yr = math.radians(S.yaw)
    mm_head.position = ((S.pos.x * MM_SCALE) + math.sin(yr) * 0.014,
                        (S.pos.z * MM_SCALE) + math.cos(yr) * 0.014, -0.006)
    if S.toast_t > 0:
        S.toast_t -= dt
        c = S.toast_col
        hud_toast.color = Color(c[0], c[1], c[2], clamp(S.toast_t / 0.6, 0, 1))
    else:
        hud_toast.text = ""


# ----------------------------------------------------------------------------
# day / night
# ----------------------------------------------------------------------------
def apply_time():
    if S.night:
        sky.color = C(28, 34, 70)
        scene.fog_color = NIGHT_FOG
        window.color = NIGHT_FOG
        ambient.color = C(52, 62, 105)
        sun.color = C(120, 140, 210)
    else:
        sky.color = color.white
        scene.fog_color = DAY_FOG
        window.color = DAY_FOG
        ambient.color = C(125, 135, 155)
        sun.color = C(255, 238, 210)


# ----------------------------------------------------------------------------
# main loop
# ----------------------------------------------------------------------------
def update():
    dt = min(time.dt, 1 / 30)
    S.t += dt

    if not S.crashed:
        step_physics(dt)
        if not S.crashed:
            check_rings()
    S.best = max(S.best, S.score)

    # visuals: rotors, LEDs, shadow blob
    if drone.enabled:
        spin = (300 + S.motor * 4200) * dt
        for r in ROTORS:
            r["h"].rotation_y += spin * r["dir"]
            r["d"].color = C(200, 220, 255, int(20 + 70 * S.motor))
        blink = (int(S.t * 6) % 2 == 0)
        for e, front, col in LEDS:
            e.color = col if (front or blink) else C(20, 20, 20)
        sup, _ = query_world(S.pos)
        blob.position = (S.pos.x, sup + 0.04, S.pos.z)
        a = max(0.0, S.pos.y - sup)
        blob.scale = 1.4 + a * 0.05
        blob.color = C(0, 0, 0, int(clamp(170 - a * 4, 0, 170)))
        S.trail_t += dt
        spd = S.vel.length()
        if S.trail_t > (0.09 if QUALITY < 2 else 0.045) and spd > 3.5 and QUALITY > 0:
            S.trail_t = 0
            puff(S.pos - S.vel.normalized() * 0.5, C(110, 220, 255, 150), 0.16, 0.01, 0.0, 0.7)

    # crash effects
    if S.crashed and S.smoke_t > 0:
        S.smoke_t -= dt
        if int(S.smoke_t * 8) != int((S.smoke_t + dt) * 8):
            puff(S.pos + Vec3(0, 0.2, 0), C(60, 60, 64, 170), 0.6, 3.2, 7.0, 2.6)
    for item in DEBRIS[:]:
        e, v, spin, life = item
        v.y -= G * dt
        e.position += v * dt
        e.rotation += spin * dt
        if e.y < 0.06:
            e.y = 0.06
            v.y = -v.y * 0.3
            v.x *= 0.6
            v.z *= 0.6
            item[2] = spin * 0.6
        item[3] -= dt
        if item[3] <= 0:
            destroy(e)
            DEBRIS.remove(item)

    # world animation
    for i, b in enumerate(BEACONS):
        b.color = C(255, 40, 40) if int(S.t * 1.5 + i) % 2 == 0 else C(70, 10, 10)
    for c in CLOUDS:
        c.x += 2.5 * dt
        if c.x > 600:
            c.x = -600
    pulse = 0.5 + 0.5 * math.sin(S.t * 5)
    for r in RINGS:
        if not r["passed"]:
            r["disc"].alpha = 0.10 + 0.10 * pulse
    for p in PADS:
        on = int(S.t * 2) % 2 == 0
        for j, l in enumerate(p["lights"]):
            l.scale = 0.34 if (j % 2 == 0) == on else 0.2

    # sun follows drone so shadows stay sharp near the action
    sun.position = S.pos - SUN_DIR * 140

    update_camera(dt)
    S.help_t -= dt
    if S.help_t < 0 and help_panel.enabled and not getattr(S, "help_pinned", False):
        help_panel.enabled = False
    update_hud(dt)


def input(key):
    if key == "escape":
        application.quit()
    elif key == "r":
        S.best = max(S.best, S.score)
        reset()
    elif key == "c":
        S.cam_mode = (S.cam_mode + 1) % 3
        toast(f"CAMERA: {CAM_NAMES[S.cam_mode]}", C(200, 220, 255), 1.2)
    elif key == "h":
        help_panel.enabled = not help_panel.enabled
        S.help_pinned = help_panel.enabled
    elif key == "z":
        S.assist = not S.assist
        toast("HOVER ASSIST " + ("ON" if S.assist else "OFF"), C(200, 220, 255), 1.4)
    elif key == "t":
        S.wind_on = not S.wind_on
        toast("WIND " + ("ON" if S.wind_on else "OFF"), C(200, 220, 255), 1.4)
    elif key == "n":
        S.night = not S.night
        apply_time()
        toast("NIGHT MODE" if S.night else "DAY MODE", C(200, 220, 255), 1.4)
    elif key == "f11":
        window.fullscreen = not window.fullscreen


reset()
S.help_pinned = False

if __name__ == "__main__":
    app.run()