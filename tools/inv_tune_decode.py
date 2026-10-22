#!/usr/bin/env python3
"""Rebuild the inverter tune from an AiM CSV export.

The LVCU sends one parameter per 0x560 frame (InvTune_Addr, InvTune_Value)
next to the 0x55D status. This collects every address seen while the read was
done, takes the most common value for each (AiM can catch a pair mid-change),
and checks the result against InvTune_CRC.

    python3 tools/inv_tune_decode.py session.csv

Names and scaling come from the comments in Core/Inc/inv_tune_params.h.
"""

import argparse
import collections
import csv
import os
import re
import sys

PARAMS_H = os.path.join(os.path.dirname(__file__), '..', 'Core', 'Inc', 'inv_tune_params.h')
MISSING = 0x8000
DONE = 4
COLUMNS = ('InvTune_Addr', 'InvTune_Value', 'InvTune_State', 'InvTune_CRC', 'InvTune_Missing')


def load_params(path):
    text = open(path).read()
    table = text[text.index('inv_tune_addrs[]'):]
    table = table[:table.index('};')]
    params = []
    for addr, note in re.findall(r'^\s*(\d+),\s*/\*\s*(.*?)\s*\*/', table, re.M):
        unit = re.search(r'\(([^)]*)\)', note)
        scale = re.search(r'\bx(\d+)\b', note)
        params.append({
            'addr': int(addr),
            'name': re.split(r' \(|,| x\d', note)[0].strip(),
            'unit': unit.group(1) if unit else '',
            'scale': int(scale.group(1)) if scale else 1,
            'signed': 'signed' in note,
        })
    return params


def crc16(values):
    c = 0xFFFF
    for v in values:
        for b in (v & 0xFF, v >> 8):
            c ^= b << 8
            for _ in range(8):
                c = ((c << 1) ^ 0x1021) & 0xFFFF if c & 0x8000 else (c << 1) & 0xFFFF
    return c


def key(name):
    return re.sub(r'[^a-z0-9]', '', name.lower())


def read_rows(path):
    """Yields dicts of the tune columns as ints. Skips AiM's metadata and unit rows."""
    want = {key(c): c for c in COLUMNS}
    cols = None
    with open(path, newline='', encoding='utf-8-sig', errors='replace') as f:
        for row in csv.reader(f):
            if cols is None:
                found = {want[key(h)]: i for i, h in enumerate(row) if key(h) in want}
                if 'InvTune_Addr' in found and 'InvTune_Value' in found:
                    cols = found
                continue
            try:
                yield {c: int(round(float(row[i]))) & 0xFFFF for c, i in cols.items()}
            except (ValueError, IndexError):
                continue
    if cols is None:
        sys.exit(f'{path}: no InvTune_Addr / InvTune_Value columns')


def scaled(p, raw):
    v = raw - 0x10000 if p['signed'] and raw & 0x8000 else raw
    if p['scale'] == 1:
        return str(v)
    digits = len(str(p['scale'])) - 1
    return f'{v / p["scale"]:.{digits}f}'


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('csv', help='AiM CSV export with the InvTune channels')
    args = ap.parse_args()

    params = load_params(PARAMS_H)
    by_addr = {p['addr']: p for p in params}

    # crc -> addr -> Counter of values
    tunes = collections.defaultdict(lambda: collections.defaultdict(collections.Counter))
    n_missing = {}
    for r in read_rows(args.csv):
        if 'InvTune_State' in r and r['InvTune_State'] != DONE:
            continue
        if r['InvTune_Addr'] not in by_addr:
            continue
        crc = r.get('InvTune_CRC')
        tunes[crc][r['InvTune_Addr']][r['InvTune_Value']] += 1
        if 'InvTune_Missing' in r:
            n_missing[crc] = r['InvTune_Missing']

    if not tunes:
        sys.exit('no tune values while InvTune_State was done')

    status = 0
    for crc, seen in tunes.items():
        values = {a: c.most_common(1)[0][0] for a, c in seen.items()}
        absent = [p for p in params if p['addr'] not in values]
        print(f'== Tune CRC {crc:04X}' if crc is not None else '== Tune (no CRC column)')
        for i, p in enumerate(params):
            if p['addr'] not in values:
                shown = '(not in log)'
            elif values[p['addr']] == MISSING and n_missing.get(crc, 1):
                shown = '0x8000 (inverter did not answer)'
            else:
                shown = f'{scaled(p, values[p["addr"]])} {p["unit"]}'.rstrip()
            print(f'{i:3d}  {p["addr"]:3d}  {p["name"]:<32} {shown}')

        if absent:
            print(f'{len(absent)} parameters never logged, log longer or raise the AiM sample rate')
            status = 1
        elif crc is not None:
            got = crc16([values[p['addr']] for p in params])
            if got == crc:
                print('CRC matches')
            else:
                print(f'CRC MISMATCH, decoded {got:04X}')
                status = 1
        print()
    return status


if __name__ == '__main__':
    sys.exit(main())
