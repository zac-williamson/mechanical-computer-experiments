#!/usr/bin/env python3
"""Small, offline LDraw/MPD orthographic renderer (numpy + Pillow only).

The result is an illustration of the supplied geometry, not a collision test.
Uses two-sided flat shading and a triangle depth buffer; transparent colours
are rendered opaque. JSON records missing references and rendering limitations.
"""
from __future__ import annotations

import argparse
import collections
import json
import math
from pathlib import Path
import re
import sys
import time

import numpy as np
from PIL import Image, ImageDraw

import os
LIBRARY = Path(os.environ.get('LDRAW_PATH','/Applications/Studio 2.0/ldraw'))


def key(name):
    return name.strip().replace('\\', '/').lower()


class LDraw:
    def __init__(self, source, library=LIBRARY):
        self.source = Path(source).resolve()
        self.library = Path(library)
        self.embedded = {}
        self.parsed = {}
        self.cache = {}
        self.missing = collections.Counter()
        self.warnings = []
        self.references = collections.Counter()
        self.roots = [self.source.parent, self.library / 'parts',
                      self.library / 'p', self.library / 'UnOfficial/parts',
                      self.library / 'UnOfficial/p', self.library / 'p/48',
                      self.library / 'UnOfficial/p/48']
        self.case_maps = {}
        lines = self.source.read_text(encoding='utf-8-sig', errors='replace').splitlines()
        sections = []
        current = None
        for line in lines:
            if line.upper().startswith('0 FILE '):
                current = key(line[7:])
                self.embedded[current] = []
                sections.append(current)
            elif line.upper().startswith('0 NOFILE'):
                current = None
            elif current is not None:
                self.embedded[current].append(line)
        self.root = sections[0] if sections else key(self.source.name)
        if not sections:
            self.embedded[self.root] = lines
        self.colours = self.read_colours()

    def read_colours(self):
        colours = {16: (190, 195, 198), 24: (50, 50, 50)}
        config = self.library / 'LDConfig.ldr'
        if config.exists():
            for line in config.read_text(errors='replace').splitlines():
                m = re.search(r'!COLOUR .*? CODE\s+(\d+)\s+VALUE\s+#([\da-fA-F]{6})', line)
                if m:
                    code, rgb = int(m[1]), m[2]
                    colours[code] = tuple(int(rgb[i:i+2], 16) for i in (0, 2, 4))
        return colours

    def locate(self, name):
        for root in self.roots:
            path = root / name
            if path.is_file():
                return path
            # Library filenames and nested directories occasionally use capitals.
            current = root
            for segment in name.split('/'):
                if current not in self.case_maps:
                    self.case_maps[current] = ({p.name.lower(): p for p in current.iterdir()}
                                               if current.is_dir() else {})
                current = self.case_maps[current].get(segment)
                if current is None:
                    break
            if current is not None and current.is_file():
                return current
        return None

    def records(self, name):
        if name in self.parsed:
            return self.parsed[name]
        if name in self.embedded:
            lines = self.embedded[name]
        else:
            path = self.locate(name)
            if path is None:
                self.missing[name] += 1
                self.parsed[name] = []
                return []
            lines = path.read_text(encoding='utf-8-sig', errors='replace').splitlines()
        records = []
        for number, line in enumerate(lines, 1):
            fields = line.split()
            if not fields:
                continue
            try:
                typ = int(fields[0])
                if typ == 1:
                    # A submodel name can contain spaces.
                    fields = line.split(maxsplit=14)
                    vals = np.asarray([float(v) for v in fields[2:14]], dtype=np.float32)
                    records.append((1, int(fields[1], 0), vals[:3], vals[3:].reshape(3, 3), key(fields[14])))
                elif typ in (3, 4):
                    verts = np.asarray([float(v) for v in fields[2:2+typ*3]], dtype=np.float32).reshape(typ, 3)
                    records.append((typ, int(fields[1], 0), verts))
            except (ValueError, IndexError) as exc:
                self.warnings.append(f'{name}:{number}: ignored malformed geometry ({exc})')
        self.parsed[name] = records
        return records

    def mesh(self, name=None, active=()):
        name = key(name or self.root)
        if name in self.cache:
            return self.cache[name]
        if name in active:
            self.warnings.append(f'Recursive reference ignored: {name}')
            return np.empty((0, 3, 3), np.float32), np.empty(0, np.int64)
        chunks, colour_chunks = [], []
        for record in self.records(name):
            typ, colour = record[:2]
            if typ == 1:
                _, _, offset, matrix, child = record
                self.references[child] += 1
                triangles, colours = self.mesh(child, (*active, name))
                if len(triangles):
                    chunks.append(triangles @ matrix.T + offset)
                    colour_chunks.append(np.where(colours == 16, colour, colours))
            else:
                vertices = record[2]
                if typ == 3:
                    chunks.append(vertices[None, :, :])
                    colour_chunks.append(np.asarray([colour], np.int64))
                else:
                    chunks.append(vertices[[[0, 1, 2], [0, 2, 3]]])
                    colour_chunks.append(np.asarray([colour, colour], np.int64))
        result = (np.concatenate(chunks), np.concatenate(colour_chunks)) if chunks else (
            np.empty((0, 3, 3), np.float32), np.empty(0, np.int64))
        self.cache[name] = result
        return result

    def rgb(self, code):
        code = int(code)
        if code in self.colours:
            return self.colours[code]
        if code & 0xFF000000 in (0x02000000, 0x03000000):
            return ((code >> 16) & 255, (code >> 8) & 255, code & 255)
        return (183, 181, 188)


def render(model, destination, root=None, azimuth=45, elevation=30, width=1600,
           height=1100, zoom=1.0, background='#f5f5f1', supersample=2,
           depth_buffer=True):
    started = time.monotonic()
    triangles, colours = model.mesh(root)
    if not len(triangles):
        raise ValueError('No triangle geometry found')
    bounds = np.stack([triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))])
    centre = bounds.mean(axis=0)
    az, el = math.radians(azimuth), math.radians(elevation)
    towards_camera = np.array([math.cos(az)*math.cos(el), -math.sin(el), math.sin(az)*math.cos(el)])
    right = np.cross([0, -1, 0], towards_camera)
    right /= np.linalg.norm(right)
    up = np.cross(towards_camera, right)
    camera = np.stack([right, up, towards_camera])
    view = (triangles - centre) @ camera.T
    projected = view[:, :, :2]
    screen_lo = projected.min(axis=(0, 1))
    screen_hi = projected.max(axis=(0, 1))
    projected -= (screen_hi + screen_lo) / 2
    extent = screen_hi - screen_lo
    scale = min(width * .9 / extent[0], height * .9 / extent[1]) * zoom
    ss = int(supersample)
    xy = projected * [scale * ss, -scale * ss] + [width * ss / 2, height * ss / 2]
    normal = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normal, axis=1)
    normal /= np.maximum(lengths, 1e-10)[:, None]
    # Force faces toward viewer so both windings receive sensible lighting.
    normal *= np.where(normal @ towards_camera >= 0, 1, -1)[:, None]
    light = towards_camera * .6 + up * .65 + right * -.45
    light /= np.linalg.norm(light)
    shade = .48 + .52 * np.maximum(normal @ light, 0)
    unique = {int(c): model.rgb(c) for c in np.unique(colours)}
    rgb = np.asarray([unique[int(c)] for c in colours], np.float32)
    # Slight ambient lift helps dark LEGO pieces remain legible.
    rgb = np.clip((rgb * .9 + 15) * shade[:, None], 0, 255).astype(np.uint8)
    order = np.argsort(view[:, :, 2].mean(axis=1), kind='stable')
    image = Image.new('RGB', (width*ss, height*ss), background)
    draw = ImageDraw.Draw(image)
    if depth_buffer:
        pixels = np.asarray(image).copy()
        depth = np.full((height*ss, width*ss), -np.inf, np.float32)
    visible = 0
    for i in order:
        points = xy[i]
        if lengths[i] < 1e-8 or points[:, 0].max() < 0 or points[:, 0].min() >= width*ss or points[:, 1].max() < 0 or points[:, 1].min() >= height*ss:
            continue
        if depth_buffer:
            x0 = max(0, int(math.floor(points[:, 0].min())))
            y0 = max(0, int(math.floor(points[:, 1].min())))
            x1 = min(width*ss-1, int(math.ceil(points[:, 0].max())))
            y1 = min(height*ss-1, int(math.ceil(points[:, 1].max())))
            a, b, c = points
            denominator = (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1])
            if abs(denominator) < 1e-8:
                continue
            xx = np.arange(x0, x1+1, dtype=np.float32)[None, :] + .5
            yy = np.arange(y0, y1+1, dtype=np.float32)[:, None] + .5
            u = ((b[1]-c[1])*(xx-c[0]) + (c[0]-b[0])*(yy-c[1])) / denominator
            v = ((c[1]-a[1])*(xx-c[0]) + (a[0]-c[0])*(yy-c[1])) / denominator
            w = 1-u-v
            z = u*view[i, 0, 2] + v*view[i, 1, 2] + w*view[i, 2, 2]
            tile = depth[y0:y1+1, x0:x1+1]
            mask = (u >= -1e-5) & (v >= -1e-5) & (w >= -1e-5) & (z >= tile)
            tile[mask] = z[mask]
            pixels[y0:y1+1, x0:x1+1][mask] = rgb[i]
        else:
            draw.polygon(tuple(map(tuple, points)), fill=tuple(rgb[i]))
        visible += 1
    if depth_buffer:
        image = Image.fromarray(pixels)
    if ss != 1:
        image = image.resize((width, height), Image.Resampling.LANCZOS)
    destination = Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    image.save(destination)
    metadata = {
        'source': str(model.source), 'root': root or model.root,
        'image': str(destination.resolve()), 'triangle_count': len(triangles),
        'triangles_drawn': visible, 'bounds_ldu': bounds.tolist(),
        'bounds_mm': (bounds * .4).tolist(), 'size_mm': ((bounds[1]-bounds[0])*.4).tolist(),
        'camera': {'azimuth_degrees': azimuth, 'elevation_degrees': elevation, 'zoom': zoom},
        'renderer': 'numpy depth buffer' if depth_buffer else 'Pillow triangle painter',
        'missing_references': dict(model.missing), 'warnings': model.warnings,
        'render_seconds': round(time.monotonic()-started, 3),
        'limitations': ['Illustrative orthographic render, not mechanical or collision validation.',
                        ('Orthographic per-pixel triangle depth buffer; no shadows or ambient occlusion.'
                         if depth_buffer else 'Triangle painter sorting can create minor overlap artifacts.'),
                        'Transparent materials rendered opaque; line and conditional-line geometry omitted.']
    }
    return metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--library', type=Path, default=LIBRARY)
    parser.add_argument('--root', help='Embedded MPD submodel name (case insensitive)')
    parser.add_argument('--azimuth', type=float, default=45)
    parser.add_argument('--elevation', type=float, default=30)
    parser.add_argument('--width', type=int, default=1600)
    parser.add_argument('--height', type=int, default=1100)
    parser.add_argument('--zoom', type=float, default=1)
    parser.add_argument('--supersample', type=int, default=2)
    parser.add_argument('--background', default='#f5f5f1')
    parser.add_argument('--painter', action='store_true', help='Faster preview, but approximate overlap sorting')
    parser.add_argument('--metadata', type=Path, help='JSON destination; defaults to PNG name with .json suffix')
    args = parser.parse_args()
    model = LDraw(args.source, args.library)
    info = render(model, args.destination, args.root, args.azimuth, args.elevation,
                  args.width, args.height, args.zoom, args.background, args.supersample,
                  not args.painter)
    metadata = args.metadata or args.destination.with_suffix('.json')
    metadata.write_text(json.dumps(info, indent=2) + '\n')
    print(json.dumps(info, indent=2))
    if model.missing:
        print('WARNING: Missing part geometry; inspect metadata before presenting this render.', file=sys.stderr)


if __name__ == '__main__':
    main()
