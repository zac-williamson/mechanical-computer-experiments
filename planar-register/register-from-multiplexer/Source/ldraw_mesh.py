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


LIBRARY = Path('/Applications/Studio 2.0/ldraw')


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

