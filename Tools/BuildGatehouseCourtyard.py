"""왕관 성문 광장의 정적 glTF와 공용 색상 아틀라스를 굽는다. Python, Pillow, NumPy 필요."""

import io
import json
import math
import struct
from itertools import pairwise
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw

OUTPUT = Path(__file__).resolve().parents[1] / "Assets/Maps/Gatehouse"
ATLAS_SIZE = 256
TILES = {
    "wall": (0, 0, 128, 128),
    "paving": (128, 0, 128, 128),
    "trim": (0, 128, 64, 64),
    "slate": (64, 128, 64, 64),
    "timber": (128, 128, 64, 64),
    "banner": (192, 128, 64, 128),
    "vault": (0, 192, 64, 64),
    "brass": (64, 192, 32, 32),
    "iron": (96, 192, 32, 32),
    "blue": (128, 192, 32, 32),
    "border": (160, 192, 32, 32),
    "recess": (64, 224, 32, 32),
    "ridge": (96, 224, 32, 32),
    "haze": (128, 224, 32, 32),
    "amber": (160, 224, 32, 32),
}


def make_atlas():
    rng = np.random.default_rng(417)
    atlas = Image.new("RGB", (ATLAS_SIZE, ATLAS_SIZE))

    def surface(name, color, pattern="", rows=4, columns=3):
        x, y, w, h = TILES[name]
        fine = rng.normal(0, 2.1, (h, w))
        grains = rng.integers(0, 255, (8, 8), dtype=np.uint8)
        cloud = np.asarray(Image.fromarray(grains).resize((w, h), Image.Resampling.BICUBIC), dtype=float)
        shade = fine + (cloud - 128) * 0.043
        yy, xx = np.mgrid[:h, :w]
        if pattern in ("joints", "slates"):
            for row in range(rows):
                low, high = row * h // rows, (row + 1) * h // rows
                offset = (row % 2) * w // (columns * 2)
                cuts = np.rint(np.linspace(0, w, columns + 1)).astype(int)
                cuts[1:-1] += rng.integers(-3, 4, columns - 1)
                for left, right in pairwise(cuts):
                    u = (xx[low:high] + offset - left) % w
                    inside = u < right - left
                    v = yy[low:high] - low
                    seam = (u == 0) | (v == 0)
                    bevel = (u == 1) | (v == 1)
                    bottom = (v == high - low - 1) | (u == right - left - 1)
                    value = rng.uniform(-8, 8) + np.where(seam, -21, np.where(bevel, 8, np.where(bottom, -7, 0)))
                    if pattern == "slates":
                        value = value + 8 * (v / (high - low) - 0.5)
                    shade[low:high] += np.where(inside, value, 0)
        elif pattern == "grain":
            shade += 7 * np.sin(xx * 1.22 + 1.9 * np.sin(yy * 0.06 + xx * 0.09))
            shade -= 20 * (xx % 16 == 0)
        elif name == "banner":
            shade += 7 * np.sin(xx * 0.32) + 2 * (xx % 2)
        pixels = np.clip(np.array(color) + shade[:, :, None], 0, 255).astype(np.uint8)
        tile = Image.fromarray(pixels)
        if name == "banner":
            draw = ImageDraw.Draw(tile)
            gold = (199, 161, 77)
            light = (232, 205, 139)
            for inset in (4, 7):
                draw.line(
                    [(inset, 0), (inset, 109), (32, 122 - inset), (63 - inset, 109), (63 - inset, 0)], fill=gold
                )
            draw.line([(8, 15), (55, 15)], fill=light, width=2)
            for ray in range(12):
                angle = math.tau * ray / 12
                sn, cs = math.sin(angle), math.cos(angle)
                draw.polygon(
                    [
                        (32 + 13 * cs - 2 * sn, 52 + 13 * sn + 2 * cs),
                        (32 + 21 * cs, 52 + 21 * sn),
                        (32 + 13 * cs + 2 * sn, 52 + 13 * sn - 2 * cs),
                    ],
                    fill=gold,
                )
            draw.ellipse((19, 39, 45, 65), outline=gold, width=2)
            draw.polygon([(21, 46), (26, 51), (31, 43), (36, 51), (42, 46), (39, 56), (24, 56)], fill=light)
            draw.line([(25, 59), (39, 59)], fill=gold, width=2)
            draw.polygon([(32, 74), (35, 82), (32, 99), (29, 82)], fill=gold)
        atlas.paste(tile, (x, y))

    surface("wall", (122, 120, 110), "joints", 5, 4)
    surface("paving", (99, 100, 94), "joints", 5, 5)
    surface("trim", (156, 148, 129))
    surface("slate", (56, 73, 86), "slates", 5, 4)
    surface("timber", (100, 73, 51), "grain")
    surface("banner", (34, 71, 101))
    surface("vault", (82, 89, 86), "joints", 4, 3)
    for name, color in [
        ("brass", (182, 138, 67)),
        ("iron", (59, 64, 65)),
        ("blue", (29, 66, 91)),
        ("border", (166, 149, 113)),
        ("recess", (35, 39, 40)),
        ("ridge", (117, 140, 146)),
        ("haze", (146, 164, 165)),
        ("amber", (222, 161, 72)),
    ]:
        surface(name, color)
    return atlas


def write_bc1_atlas(image, path):
    # DX10 헤더에 BC1 sRGB와 전체 밉 수를 명시한다. 하위 밉은 선형광에서 평균한다.
    width, height = image.size
    mips = int(math.log2(max(width, height))) + 1
    header = [124, 0xA1007, height, width, ((width + 3) // 4) * ((height + 3) // 4) * 8, 0, mips]
    header += [0] * 11 + [32, 4, int.from_bytes(b"DX10", "little"), 0, 0, 0, 0, 0]
    header += [0x401008, 0, 0, 0, 0]
    data = bytearray(b"DDS " + struct.pack("<31I", *header) + struct.pack("<5I", 72, 3, 0, 1, 3))
    srgb = np.asarray(image, dtype=float) / 255.0
    linear = np.where(srgb <= 0.04045, srgb / 12.92, ((srgb + 0.055) / 1.055) ** 2.4)
    for level in range(mips):
        if level:
            h, w, _ = linear.shape
            linear = linear.reshape(h // 2, 2, w // 2, 2, 3).mean(axis=(1, 3))
            srgb = np.where(linear <= 0.0031308, 12.92 * linear, 1.055 * linear ** (1 / 2.4) - 0.055)
            image = Image.fromarray(np.rint(255 * srgb).clip(0, 255).astype(np.uint8))
        stream = io.BytesIO()
        image.save(stream, format="DDS", pixel_format="DXT1")
        data.extend(stream.getvalue()[128:])
    path.write_bytes(data)


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def mix(a, b, t):
    return tuple(x + (y - x) * t for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


class Courtyard:
    def __init__(self):
        self.vertices = []
        self.indices = []
        self.lookup = {}
        self.collisions = []

    def face(self, points, tile, uvs=None, toward=None):
        normal = cross(sub(points[1], points[0]), sub(points[2], points[0]))
        length = math.hypot(*normal)
        if length < 0.000001:
            raise ValueError(f"면의 넓이가 0입니다: {points}")
        normal = tuple(v / length for v in normal)
        if uvs is None:
            uvs = [(0, 1), (1, 1), (1, 0), (0, 0)][: len(points)]
        if toward and sum(a * b for a, b in zip(normal, toward)) < 0:
            points, uvs = points[::-1], uvs[::-1]
            normal = tuple(-v for v in normal)
        x, y, w, h = TILES[tile]
        local = []
        for point, (u, v) in zip(points, uvs, strict=True):
            uv = ((x + 2 + u * (w - 4)) / ATLAS_SIZE, (y + 2 + v * (h - 4)) / ATLAS_SIZE)
            vertex = tuple(round(k, 6) for k in (*point, *normal, *uv))
            index = self.lookup.get(vertex)
            if index is None:
                index = len(self.vertices)
                self.vertices.append(vertex)
                self.lookup[vertex] = index
            local.append(index)
        for i in range(1, len(local) - 1):
            self.indices.extend((local[0], local[i], local[i + 1]))

    def panel(self, a, b, c, d, tile, toward=None):
        # 큰 평면만 몇 m 단위로 나누어 아틀라스의 같은 타일을 다시 쓴다.
        size = {
            "wall": (3.6, 2.4),
            "vault": (3.0, 2.2),
            "paving": (3.2, 2.8),
            "slate": (2.8, 2.3),
            "timber": (3.0, 3.2),
        }.get(tile, (10.0, 10.0))
        nx = max(1, round(math.dist(a, b) / size[0]))
        ny = max(1, round(math.dist(a, d) / size[1]))
        for j in range(ny):
            l0, r0 = mix(a, d, j / ny), mix(b, c, j / ny)
            l1, r1 = mix(a, d, (j + 1) / ny), mix(b, c, (j + 1) / ny)
            for i in range(nx):
                self.face(
                    [
                        mix(l0, r0, i / nx),
                        mix(l0, r0, (i + 1) / nx),
                        mix(l1, r1, (i + 1) / nx),
                        mix(l1, r1, i / nx),
                    ],
                    tile,
                    toward=toward,
                )

    def collider(self, name, x, y, z, width, height, depth):
        self.collisions.append(
            {
                "name": "COL_" + name,
                "translation": [x, y + height / 2, z],
                "scale": [width / 2, height / 2, depth / 2],
            }
        )

    def box(self, x, y, z, w, h, d, tile="wall", collision=None, bottom=False):
        x0, x1, z0, z1, y1 = x - w / 2, x + w / 2, z - d / 2, z + d / 2, y + h
        self.panel((x0, y, z1), (x1, y, z1), (x1, y1, z1), (x0, y1, z1), tile)
        self.panel((x1, y, z0), (x0, y, z0), (x0, y1, z0), (x1, y1, z0), tile)
        self.panel((x1, y, z1), (x1, y, z0), (x1, y1, z0), (x1, y1, z1), tile)
        self.panel((x0, y, z0), (x0, y, z1), (x0, y1, z1), (x0, y1, z0), tile)
        self.panel((x0, y1, z1), (x1, y1, z1), (x1, y1, z0), (x0, y1, z0), tile)
        if bottom:
            self.face([(x0, y, z0), (x1, y, z0), (x1, y, z1), (x0, y, z1)], tile)
        if collision:
            self.collider(collision, x, y, z, w, h, d)

    def cone(self, x, z, y, top, radius, cap, sides, tile, lid=False):
        def ring(r, height):
            return [
                (x + r * math.sin(a), height, z + r * math.cos(a))
                for a in (math.tau * (i + 0.5) / sides for i in range(sides))
            ]

        base, crown = ring(radius, y), ring(cap, top)
        for i in range(sides):
            j = (i + 1) % sides
            if cap:
                self.panel(base[i], base[j], crown[j], crown[i], tile)
                if lid:
                    self.face([(x, top, z), crown[i], crown[j]], tile, toward=(0, 1, 0))
            else:
                self.face([base[i], base[j], (x, top, z)], tile, [(0, 1), (1, 1), (0.5, 0)])

    def window(self, x, y, z, w, h, direction=(0, 1)):
        nx, nz = direction

        def point(u, v, relief):
            return (x + u * nz + relief * nx, y + v, z - u * nx + relief * nz)

        for width, height, base, relief, tile in [
            (w, h, 0, 0.026, "trim"),
            (w * 0.72, h * 0.83, 0.09, 0.034, "recess"),
        ]:
            shape = [
                (-width / 2, base),
                (width / 2, base),
                (width / 2, base + height * 0.70),
                (0, base + height),
                (-width / 2, base + height * 0.70),
            ]
            uv = [(0, 1), (1, 1), (1, 0.32), (0.5, 0), (0, 0.32)]
            self.face([point(u, v, relief) for u, v in shape], tile, uv, toward=(nx, 0, nz))
        if w > 0.85:
            self.face(
                [
                    point(-0.035, 0.1, 0.045),
                    point(0.035, 0.1, 0.045),
                    point(0.035, 0.76 * h, 0.045),
                    point(-0.035, 0.76 * h, 0.045),
                ],
                "trim",
            )

    def banner(self, x, y, z, width, height):
        # 천의 주름과 두 갈래 끝은 고정 메시이므로 천 시뮬레이션이 필요 없다.
        def point(u, v):
            hem = 0.16 * height * max(0, 1 - abs(u - 0.5) * 2) * max(0, (v - 0.66) / 0.34)
            return (
                x + (u - 0.5) * width,
                y - height * v + hem,
                z + 0.075 * math.cos(u * math.tau * 2) + 0.055 * math.sin(v * math.pi),
            )

        self.box(x, y + 0.035, z, 0.31 + width, 0.09, 0.10, "brass", bottom=True)
        for j in range(3):
            for i in range(8):
                u, u1, v, v1 = i / 8, (i + 1) / 8, j / 3, (j + 1) / 3
                uv = [(u, v1), (u1, v1), (u1, v), (u, v)]
                self.face([point(s, t) for s, t in uv], "banner", uv)

    def pennant(self, x, y, z, height):
        self.box(x, y, z, 0.075, height, 0.075, "brass")
        top = y + height - 0.09
        rings = [
            (x, top, z),
            (x + 0.50, top + 0.07, z + 0.08),
            (x + 1.02, top - 0.04, z + 0.12),
            (x + 1.54, top + 0.025, z + 0.21),
        ]
        widths = [0.49, 0.42, 0.26, 0.025]
        for i in range(3):
            a, b = rings[i : i + 2]
            c = (a[0], a[1] - widths[i], a[2])
            d = (b[0], b[1] - widths[i + 1], b[2])
            for points in [[c, d, b, a], [a, b, d, c]]:
                self.face(points, "blue")

    def tower(self, x, z, r, h, spire=False, flag=False):
        def merlon(rad, height, angle):
            return (x + rad * math.sin(angle), height, z + rad * math.cos(angle))

        def bracket(direction, u, v, rad):
            dx, dz = direction
            return (x + dx * rad + dz * u, h + v, z + dz * rad - dx * u)

        sides = 10
        self.cone(x, z, 0, 0.7, r * 1.10, r * 1.10, sides, "vault")
        self.cone(x, z, 0.7, 1.55, r * 1.10, r, sides, "trim")
        self.cone(x, z, 1.55, h - 0.75, r, r * 0.96, sides, "wall")
        self.cone(x, z, h - 0.76, h - 0.48, r * 0.97, r * 1.12, sides, "vault")
        self.cone(x, z, h - 0.48, h - 0.28, r * 1.12, r * 1.12, sides, "trim")
        self.cone(x, z, h - 0.28, h + 0.18, r * 1.10, r * 1.10, sides, "wall", lid=True)
        self.cone(x, z, 3.0, 3.17, r * 1.015, r * 1.015, sides, "trim")
        for k in range(sides):
            a = math.tau * k / sides
            direction = (math.sin(a), math.cos(a))
            dx, dz = direction
            self.window(
                x + dx * r * 0.99 * math.cos(math.pi / sides),
                h - 3.05,
                z + dz * r * 0.99 * math.cos(math.pi / sides),
                0.42,
                1.42,
                direction,
            )

            da = math.pi / sides * 0.59
            footprint = [
                merlon(rad, h + 0.17, angle)
                for rad, angle in [
                    (r * 1.095, a - da),
                    (r * 1.095, a + da),
                    (r * 0.91, a + da),
                    (r * 0.91, a - da),
                ]
            ]
            crown = [(px, py + 0.75, pz) for px, py, pz in footprint]
            for i in range(4):
                j = (i + 1) % 4
                self.face([footprint[i], footprint[j], crown[j], crown[i]], "trim")
            self.face(crown, "trim", toward=(0, 1, 0))

            block = [
                bracket(direction, -0.14, -0.98, r * 0.969),
                bracket(direction, 0.14, -0.98, r * 0.969),
                bracket(direction, 0.14, -0.48, r * 1.07),
                bracket(direction, -0.14, -0.48, r * 1.07),
            ]
            self.face(block, "trim", toward=(dx, 0, dz))
            for kside in (-1, 1):
                self.face(
                    [
                        bracket(direction, kside * 0.14, -0.98, r * 0.969),
                        bracket(direction, kside * 0.14, -0.48, r * 1.07),
                        bracket(direction, kside * 0.14, -0.48, r * 0.943),
                    ],
                    "vault",
                    toward=(kside * dz, 0, -kside * dx),
                )
        if spire:
            self.cone(x, z, h + 0.17, h + 1.35, r * 0.75, r * 0.75, sides, "wall")
            self.cone(x, z, h + 1.33, h + 1.53, r * 0.91, r * 0.93, sides, "trim")
            self.cone(x, z, h + 1.53, h + r * 2.15, r * 0.94, r * 0.15, sides, "slate")
            self.cone(x, z, h + r * 2.15, h + r * 2.84, r * 0.15, 0, sides, "slate")
            if flag:
                self.pennant(x, h + r * 2.84 - 0.13, z, 1.18)
        elif flag:
            self.pennant(x, h + 0.18, z, 3.45)

    def arch(self, x, z, width, depth, height, span, spring):
        def profile(radius):
            center = 0.46 * radius
            limit = math.acos(center / (radius + center))
            half = [
                (
                    center - (radius + center) * math.cos(limit * i / 7),
                    spring + (radius + center) * math.sin(limit * i / 7),
                )
                for i in range(8)
            ]
            return half + [(-u, v) for u, v in half[-2::-1]]

        arch = profile(span)
        outer = profile(span + 0.31)
        front, back = z + depth / 2, z - depth / 2

        def at(p, plane):
            return (x + p[0], p[1], plane)

        pillar = width / 2 - span
        for side in (-1, 1):
            px = x + side * (span + pillar / 2)
            self.box(px, 0, z, pillar, height, depth, "wall", "GatePier")
            for i in range(5):
                self.box(
                    x + side * (span + 0.12),
                    spring * i / 5,
                    front + 0.035,
                    0.32,
                    spring / 5 - 0.022,
                    0.23,
                    "trim",
                )
            self.box(
                x + side * (span + 0.15), spring - 0.09, front + 0.06, 0.50, 0.19, 0.29, "border", bottom=True
            )
        for i in range(len(arch) - 1):
            a, b = arch[i : i + 2]
            for plane, nz in [(front, 1), (back, -1)]:
                self.panel(
                    at(a, plane),
                    at(b, plane),
                    (x + b[0], height, plane),
                    (x + a[0], height, plane),
                    "wall",
                    toward=(0, 0, nz),
                )
            self.panel(
                at(a, front),
                at(b, front),
                at(b, back),
                at(a, back),
                "vault",
                toward=(b[1] - a[1], a[0] - b[0], 0),
            )
            aa, bb = mix(a, b, 0.018), mix(a, b, 0.982)
            c, d = mix(outer[i], outer[i + 1], 0.018), mix(outer[i], outer[i + 1], 0.982)
            face = front + (0.27 if i in (6, 7) else 0.18)
            self.face([at(aa, face), at(bb, face), at(d, face), at(c, face)], "trim", toward=(0, 0, 1))
            self.face(
                [at(aa, front), at(bb, front), at(bb, face), at(aa, face)],
                "border",
                toward=(b[1] - a[1], a[0] - b[0], 0),
            )
        self.box(x, height, z, width + 0.24, 0.23, depth + 0.35, "trim", bottom=True)
        self.collider(
            "GateLintel", x, arch[len(arch) // 2][1], z, 2 * span, height - arch[len(arch) // 2][1], depth
        )
        apex = arch[len(arch) // 2][1]
        for step in (2, 4):
            start, base = arch[step]
            end = arch[step - 2][0]
            for side in (-1, 1):
                self.collider(
                    "ArchHaunch", x + side * (start + end) / 2, base, z, start - end, apex - base, depth
                )

    def parapet(self, x, z, length, base, axis):
        dx, dz = axis
        n = max(2, round(length / 1.62))
        self.box(
            x, base, z, length + 0.12 if dx else 0.81, 0.19, 0.81 if dx else length + 0.12, "trim", bottom=True
        )
        for i in range(n):
            t = length * ((i + 0.5) / n - 0.5)
            self.box(
                x + t * dx, base + 0.18, z + t * dz, 0.82 if dx else 0.62, 0.64, 0.62 if dx else 0.82, "trim"
            )

    def hipped_roof(self, x, z, w, d, base, peak):
        x0, x1, z0, z1 = x - w / 2, x + w / 2, z - d / 2, z + d / 2
        r0, r1 = x0 + d * 0.36, x1 - d * 0.36
        self.panel((x0, base, z1), (x1, base, z1), (r1, peak, z), (r0, peak, z), "slate")
        self.panel((x1, base, z0), (x0, base, z0), (r0, peak, z), (r1, peak, z), "slate")
        self.face([(x0, base, z0), (x0, base, z1), (r0, peak, z)], "slate", toward=(-1, 1, 0))
        self.face([(x1, base, z1), (x1, base, z0), (r1, peak, z)], "slate", toward=(1, 1, 0))
        self.box(x, peak - 0.03, z, r1 - r0, 0.12, 0.11, "brass")

    def plaza(self):
        def pavement(x0, x1, z0, z1, tile):
            self.panel((x0, 0, z1), (x1, 0, z1), (x1, 0, z0), (x0, 0, z0), tile)

        # 통행로와 문장은 바닥의 같은 평면에 맞물려 중복으로 그리지 않는다.
        for x0, x1, tile in [
            (-13, -3.37, "paving"),
            (-3.37, -3.16, "border"),
            (3.16, 3.37, "border"),
            (3.37, 13, "paving"),
        ]:
            pavement(x0, x1, -22, 16, tile)
        pavement(-3.16, 3.16, -22, -2.16, "paving")
        pavement(-3.16, 3.16, 4.16, 16, "paving")
        for i in range(32):
            a, b = math.tau * i / 32, math.tau * (i + 1) / 32

            def point(r, t):
                return (r * math.sin(t), 0, 1 + r * math.cos(t))

            def edge(t):
                return point(3.16 / max(abs(math.sin(t)), abs(math.cos(t))), t)

            shape = [point(2.47, a), point(2.47, b), edge(b), edge(a)]
            self.face(
                shape, "paving", [(p[0] / 6.32 + 0.5, (p[2] + 2.16) / 6.32) for p in shape], toward=(0, 1, 0)
            )
            for r0, r1, tile in [(2.34, 2.47, "border"), (2.24, 2.34, "iron"), (2.14, 2.24, "border")]:
                self.face([point(r0, a), point(r0, b), point(r1, b), point(r1, a)], tile, toward=(0, 1, 0))
            inner0 = 1.99 if i % 4 == 0 else (0.79 if i % 2 == 0 else 0.46)
            inner1 = 1.99 if (i + 1) % 4 == 0 else (0.79 if (i + 1) % 2 == 0 else 0.46)
            self.face(
                [point(inner0, a), point(inner1, b), point(2.14, b), point(2.14, a)], "vault", toward=(0, 1, 0)
            )
            self.face(
                [(0, 0, 1), point(inner0, a), point(inner1, b)], "border" if i % 2 else "brass", toward=(0, 1, 0)
            )
        self.collider("Plaza", 0, -0.40, -3, 26, 0.4, 38)

        # 주 성문과 안뜰. 탑의 하단을 정방형 기단으로 만들어 단순한 충돌 상자가 외곽과 일치한다.
        self.arch(0, -10.8, 9, 3.2, 7.45, 2.48, 2.25)
        self.parapet(0, -9.24, 9, 7.68, (1, 0))
        self.parapet(0, -12.36, 9, 7.68, (1, 0))
        for side in (-1, 1):
            x = side * 5.6
            self.box(x, 0, -10.9, 4.94, 0.88, 4.94, "vault", "TowerBase")
            self.tower(x, -10.9, 2.42, 10.55 if side < 0 else 9.85, spire=side > 0, flag=True)
            self.collider("TowerShaft", x, 0.88, -10.9, 4.57, 9.5, 2.82)
            self.collider("TowerShaft", x, 0.88, -10.9, 2.82, 9.5, 4.51)
            self.banner(x, 6.9, -8.46, 1.16, 3.95)
            self.window(side * 1.59, 5.67, -9.20, 0.63, 1.22)
            self.box(side * 10.3, 0, -10.8, 5.6, 5.35, 1.42, "wall", "Curtain")
            self.parapet(side * 10.3, -10.11, 5.6, 5.35, (1, 0))
            self.box(side * 10.7, 0, -9.94, 0.78, 3.98, 0.71, "trim")
            self.banner(side * 10.7, 4.79, -9.53, 0.89, 2.27)
        self.window(0, 6.04, -9.20, 0.9, 1.00)
        for i in range(-5, 6):
            self.box(i * 0.72, 7.03, -9.13, 0.22, 0.42, 0.23, "vault", bottom=True)
            self.box(i * 0.72, 7.26, -9.06, 0.30, 0.18, 0.30, "trim", bottom=True)
        # 격자는 아치의 위쪽에 걷힌 상태로 둔다.
        for i in range(-3, 4):
            base = 4.44 - 0.15 * abs(i)
            self.box(i * 0.32, base, -9.55, 0.075, 1.02, 0.08, "iron", bottom=True)
        self.box(0, 4.79, -9.56, 2.24, 0.10, 0.10, "iron")

        # 광장 둘레. 보행 가능한 부분에만 프록시를 둔다.
        for side in (-1, 1):
            x = side * 12.72
            self.box(x, 0, -3, 0.64, 3.9, 38, "wall", "SideWall")
            self.parapet(x, -3, 38, 3.9, (0, 1))
            for z in (-18, -13, -7.8, -3.2, 1.4, 6, 10.6, 15):
                self.box(x - 0.14 * side, 0, z, 0.91, 3.12, 0.79, "vault")
                self.box(x - 0.14 * side, 3.12, z, 1.03, 0.18, 0.91, "trim", bottom=True)
        self.box(0, 0, 15.72, 26, 3.28, 0.64, "wall", "SouthWall")
        self.parapet(0, 15.72, 26, 3.28, (1, 0))
        self.box(0, 0, -21.72, 26, 4.44, 0.64, "wall", "NorthWall")
        self.parapet(0, -21.72, 26, 4.44, (1, 0))
        for x, z in [(-12.45, 15.45), (12.45, 15.45), (-12.45, -21.45), (12.45, -21.45)]:
            self.box(x, 0, z, 1.7, 4.8, 1.7, "wall", "Corner")
            self.box(x, 4.8, z, 1.97, 0.21, 1.97, "trim", bottom=True)
            self.cone(x, z, 5.01, 6.72, 1.31, 0, 4, "slate")

        # 동쪽 전망대. 개별 계단과 난간에는 실제 보행 충돌이 있다.
        for i in range(8):
            h = (i + 1) * 0.15
            self.box(9.65, 0, 3.15 - i * 0.44, 3.25, h, 0.44, "paving", "Step")
            self.box(9.65, h - 0.045, 3.15 - i * 0.44 + 0.204, 3.25, 0.044, 0.036, "border")
        self.box(9.65, 0, -2.22, 3.25, 1.20, 4.16, "paving", "Terrace")
        self.box(11.14, 1.20, -2.22, 0.27, 0.70, 4.16, "wall", "Parapet")
        self.box(9.65, 1.20, -4.16, 3.25, 0.70, 0.28, "wall", "Parapet")
        self.box(11.14, 1.90, -2.22, 0.39, 0.115, 4.20, "trim")
        self.box(9.65, 1.90, -4.16, 3.30, 0.115, 0.40, "trim")

        # 서쪽의 개방 회랑. 기둥 사이와 처마 아래를 지날 수 있다.
        for z in (-5.3, -2.2, 0.9, 4.0):
            self.box(-9.25, 0, z, 0.68, 0.38, 0.68, "trim")
            self.box(-9.25, 0.38, z, 0.45, 2.25, 0.45, "vault")
            self.box(-9.25, 2.63, z, 0.72, 0.24, 0.72, "trim")
            self.collider("Column", -9.25, 0, z, 0.56, 2.87, 0.56)
        self.box(-9.25, 2.87, -0.65, 0.45, 0.33, 10.1, "timber")
        self.panel(
            (-8.89, 3.10, 4.43),
            (-8.89, 3.10, -5.73),
            (-12.44, 4.22, -5.73),
            (-12.44, 4.22, 4.43),
            "slate",
            toward=(0, 1, 0),
        )
        self.box(-8.91, 3.01, -0.65, 0.12, 0.13, 10.18, "timber")
        for z in (-4.55, -1.48, 1.6):
            self.box(-11.75, 0, z, 0.58, 0.48, 1.5, "trim", "Bench")

        # 안뜰 너머 본성과 첨탑은 실루엣 위주로 짓는다. 외곽 벽 밖에는 충돌체가 필요 없다.
        self.box(-1.55, 0, -25.3, 11.0, 10.9, 5.6, "wall")
        self.box(-1.55, 9.55, -25.3, 11.19, 0.22, 5.78, "trim")
        self.box(-1.55, 10.9, -25.3, 11.55, 0.32, 5.93, "trim", bottom=True)
        self.hipped_roof(-1.55, -25.3, 12.05, 6.45, 11.22, 14.9)
        for x in (-5.65, -3.63, -1.61, 0.41, 2.43):
            self.window(x, 7.35, -22.5, 0.92, 1.86)
            self.window(x, 4.87, -22.5, 0.48, 1.24)
        self.tower(-6.78, -24.2, 1.70, 12.15, spire=True, flag=True)
        self.tower(3.77, -24.7, 1.42, 11.61, spire=True)
        self.tower(-2.8, -28.7, 1.95, 14.0, spire=True, flag=True)
        for side in (-1, 1):
            self.box(side * 11.2, 0, -25, 8, 6.4, 4.5, "vault")
            self.hipped_roof(side * 11.2, -25, 8.4, 4.9, 6.4, 8.64)
            for offset in (-2.4, 0, 2.4):
                self.window(side * 11.2 + offset, 4.47, -22.75, 0.62, 1.14)

        # 멀리 보이는 두 능선도 같은 아틀라스의 무광 색 한 조각을 공유한다.
        def ridge_height(x, layer):
            return (
                5
                + 2.7 * layer
                + 3.4 * math.sin(x * 0.13 + layer)
                + 2.1 * math.cos(x * 0.23 + layer * 0.8)
                + 1.4 * math.sin(x * 0.48)
            )

        for layer, (z, tile) in enumerate([(-49, "ridge"), (-65, "haze")]):
            for x in range(-64, 64, 4):
                self.face(
                    [
                        (x, -1, z),
                        (x + 4, -1, z),
                        (x + 4, ridge_height(x + 4, layer), z),
                        (x, ridge_height(x, layer), z),
                    ],
                    tile,
                )

    def save(self):
        if len(self.vertices) >= 65536:
            raise ValueError("16비트 인덱스 범위를 초과했습니다.")
        OUTPUT.mkdir(parents=True, exist_ok=True)
        atlas = make_atlas()
        atlas.save(OUTPUT / "Stonework.png", optimize=True)
        write_bc1_atlas(atlas, OUTPUT / "Stonework.dds")
        data = bytearray()
        views = []
        accessors = []

        def stream(rows, fmt, kind, component, target, limits=False):
            while len(data) % 4:
                data.append(0)
            offset = len(data)
            pack = struct.Struct("<" + fmt).pack
            for row in rows:
                data.extend(pack(*row))
            views.append({"buffer": 0, "byteOffset": offset, "byteLength": len(data) - offset, "target": target})
            accessor = {
                "bufferView": len(views) - 1,
                "componentType": component,
                "count": len(rows),
                "type": kind,
            }
            if limits:
                accessor["min"] = [min(v) for v in zip(*rows)]
                accessor["max"] = [max(v) for v in zip(*rows)]
            accessors.append(accessor)
            return len(accessors) - 1

        position = stream([v[:3] for v in self.vertices], "3f", "VEC3", 5126, 34962, True)
        normal = stream([v[3:6] for v in self.vertices], "3f", "VEC3", 5126, 34962)
        uv = stream([v[6:] for v in self.vertices], "2f", "VEC2", 5126, 34962)
        indices = stream([(i,) for i in self.indices], "H", "SCALAR", 5123, 34963)
        nodes = [{"name": "Royal Gatehouse", "mesh": 0}] + self.collisions
        model = {
            "asset": {"version": "2.0", "generator": "PARS / BuildGatehouseCourtyard"},
            "scene": 0,
            "extensionsUsed": ["MSFT_texture_dds"],
            "scenes": [{"nodes": list(range(len(nodes)))}],
            "nodes": nodes,
            "meshes": [
                {
                    "primitives": [
                        {
                            "attributes": {"POSITION": position, "NORMAL": normal, "TEXCOORD_0": uv},
                            "indices": indices,
                            "material": 0,
                        }
                    ]
                }
            ],
            "materials": [
                {
                    "name": "Courtyard",
                    "pbrMetallicRoughness": {
                        "baseColorTexture": {"index": 0},
                        "metallicFactor": 0,
                        "roughnessFactor": 0.88,
                    },
                }
            ],
            "images": [{"uri": "Stonework.png"}, {"uri": "Stonework.dds"}],
            "textures": [{"source": 0, "sampler": 0, "extensions": {"MSFT_texture_dds": {"source": 1}}}],
            "samplers": [{"magFilter": 9729, "minFilter": 9987, "wrapS": 33071, "wrapT": 33071}],
            "buffers": [{"uri": "Courtyard.bin", "byteLength": len(data)}],
            "bufferViews": views,
            "accessors": accessors,
        }
        (OUTPUT / "Courtyard.bin").write_bytes(data)
        (OUTPUT / "Courtyard.gltf").write_text(json.dumps(model, indent=1) + "\n", encoding="utf8", newline="\n")
        print(
            f"{len(self.vertices):,} vertices, {len(self.indices) // 3:,} triangles, {len(self.collisions)} collision boxes"
        )


if __name__ == "__main__":
    courtyard = Courtyard()
    courtyard.plaza()
    courtyard.save()
