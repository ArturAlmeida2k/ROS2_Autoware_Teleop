#!/usr/bin/env python3
"""
pcq_codec.py — Codec "PCQ1" (PointCloud Quantized v1).

Módulo puro numpy (SEM dependências ROS) para que possa ser testado
offline e reutilizado nos dois lados do link.

Formato do blob:
    offset  0 : magic          4B   b'PCQ1'
    offset  4 : flags          1B   bit0 = intensidade presente
    offset  5 : comp           1B   0 = raw, 1 = lz4, 2 = zlib
    offset  6 : reserved       2B
    offset  8 : count          4B   uint32  (nº de pontos)
    offset 12 : scale          4B   float32 (metros por LSB)
    offset 16 : origin x,y,z  12B   3x float32
    offset 28 : payload        ...  (eventualmente comprimido)

Payload (colunar — melhora muito o rácio de compressão vs. intercalado):
    int16  x[N]
    int16  y[N]
    int16  z[N]
    uint8  i[N]     (apenas se flags bit0)

Custo: 6 bytes/ponto (7 com intensidade) vs. 16-32 bytes/ponto do CDR cru.
"""

import struct
import zlib

import numpy as np

try:
    import lz4.frame as _lz4
except ImportError:  # pragma: no cover
    _lz4 = None

MAGIC = b'PCQ1'
HEADER_FMT = '<4sBBHIf3f'
HEADER_SIZE = struct.calcsize(HEADER_FMT)  # 28

COMP_NONE, COMP_LZ4, COMP_ZLIB = 0, 1, 2
FLAG_INTENSITY = 0x01

# sensor_msgs/PointField datatype -> numpy
_PF_DTYPES = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}


# --------------------------------------------------------------------------
# 1. PointCloud2 -> numpy (sem sensor_msgs_py, sem cópias desnecessárias)
# --------------------------------------------------------------------------
def pointcloud2_to_array(msg):
    """Devolve um numpy structured array (1-D) com os campos da PointCloud2.

    Respeita offsets, padding interno, padding de linha (row_step) e
    endianness declarada na mensagem.
    """
    fields = sorted(msg.fields, key=lambda f: f.offset)

    dtype_list = []
    offset = 0
    for i, f in enumerate(fields):
        if f.offset > offset:
            dtype_list.append((f'__pad{i}', np.uint8, (f.offset - offset,)))
        np_type = _PF_DTYPES.get(f.datatype)
        if np_type is None:
            raise ValueError(f'PointField datatype não suportado: {f.datatype}')
        count = max(1, f.count)
        if count > 1:
            dtype_list.append((f.name, np_type, (count,)))
        else:
            dtype_list.append((f.name, np_type))
        offset = f.offset + np.dtype(np_type).itemsize * count

    if offset < msg.point_step:
        dtype_list.append(('__pad_end', np.uint8, (msg.point_step - offset,)))

    dt = np.dtype(dtype_list)
    if dt.itemsize != msg.point_step:
        raise ValueError(f'point_step={msg.point_step} != dtype={dt.itemsize}')
    if msg.is_bigendian:
        dt = dt.newbyteorder('>')

    buf = np.frombuffer(msg.data, dtype=np.uint8)

    # row_step pode incluir padding no fim de cada linha
    if msg.row_step != msg.width * msg.point_step and msg.height > 1:
        buf = buf[:msg.height * msg.row_step].reshape(msg.height, msg.row_step)
        buf = buf[:, :msg.width * msg.point_step].reshape(-1)

    n = msg.width * msg.height
    return np.frombuffer(buf.tobytes(), dtype=dt, count=n)


# --------------------------------------------------------------------------
# 2. Filtro voxel (numpy puro, sem PCL/Open3D)
# --------------------------------------------------------------------------
def voxel_downsample(xyz, voxel_size):
    """Mantém um ponto por voxel. Devolve os ÍNDICES mantidos.

    Nota: guarda o primeiro ponto de cada voxel (não o centróide). É ~10x
    mais rápido e, a 2-5 cm, visualmente indistinguível para deteção de
    obstáculos. Se precisar do centróide, use np.add.reduceat sobre a
    ordenação por chave.
    """
    if voxel_size <= 0 or xyz.shape[0] == 0:
        return np.arange(xyz.shape[0])

    keys = np.floor(xyz / voxel_size).astype(np.int64)
    keys -= keys.min(axis=0)
    dims = keys.max(axis=0) + 1

    # Linearização 3D -> 1D. Verificar overflow antes de multiplicar.
    if float(dims[0]) * float(dims[1]) * float(dims[2]) > 9e18:
        _, idx = np.unique(keys, axis=0, return_index=True)
        return idx

    lin = (keys[:, 0] * dims[1] * dims[2]) + (keys[:, 1] * dims[2]) + keys[:, 2]
    _, idx = np.unique(lin, return_index=True)
    return idx


# --------------------------------------------------------------------------
# 3. Encode
# --------------------------------------------------------------------------
def encode(xyz, intensity=None, scale=0.005, compression='auto'):
    """Quantiza XYZ para int16 e empacota no formato PCQ1.

    scale=0.005 -> 5 mm de resolução, alcance +-163 m à volta da origem.
    A origem é o centro da bounding box do frame, enviada em float32.
    """
    xyz = np.asarray(xyz, dtype=np.float32)
    n = xyz.shape[0]

    if n == 0:
        origin = np.zeros(3, dtype=np.float32)
        payload = b''
        flags = 0
    else:
        lo = xyz.min(axis=0)
        hi = xyz.max(axis=0)
        origin = ((lo + hi) * 0.5).astype(np.float32)

        q = np.rint((xyz - origin) / scale)
        np.clip(q, -32768, 32767, out=q)
        q = q.astype('<i2')

        # Colunar: todos os X, depois todos os Y, depois todos os Z.
        parts = [np.ascontiguousarray(q[:, 0]).tobytes(),
                 np.ascontiguousarray(q[:, 1]).tobytes(),
                 np.ascontiguousarray(q[:, 2]).tobytes()]

        flags = 0
        if intensity is not None:
            inten = np.asarray(intensity, dtype=np.float32)
            imax = float(inten.max()) if inten.size else 0.0
            norm = inten / imax * 255.0 if imax > 0 else inten
            parts.append(np.clip(norm, 0, 255).astype(np.uint8).tobytes())
            flags |= FLAG_INTENSITY

        payload = b''.join(parts)

    comp = COMP_NONE
    if compression == 'auto':
        compression = 'lz4' if _lz4 is not None else 'zlib'
    if payload:
        if compression == 'lz4' and _lz4 is not None:
            payload, comp = _lz4.compress(payload), COMP_LZ4
        elif compression == 'zlib':
            payload, comp = zlib.compress(payload, 1), COMP_ZLIB

    header = struct.pack(HEADER_FMT, MAGIC, flags, comp, 0, n,
                         float(scale), *origin.tolist())
    return header + payload


# --------------------------------------------------------------------------
# 4. Decode
# --------------------------------------------------------------------------
def decode(blob):
    """Devolve (xyz float32 [N,3], intensity uint8 [N] ou None)."""
    if len(blob) < HEADER_SIZE:
        raise ValueError('blob demasiado curto')

    magic, flags, comp, _, n, scale, ox, oy, oz = struct.unpack(
        HEADER_FMT, blob[:HEADER_SIZE])
    if magic != MAGIC:
        raise ValueError(f'magic inválido: {magic!r}')

    payload = blob[HEADER_SIZE:]
    if comp == COMP_LZ4:
        if _lz4 is None:
            raise RuntimeError('blob em LZ4 mas o pacote lz4 não está instalado')
        payload = _lz4.decompress(payload)
    elif comp == COMP_ZLIB:
        payload = zlib.decompress(payload)

    if n == 0:
        return np.zeros((0, 3), np.float32), None

    expected = n * 6 + (n if flags & FLAG_INTENSITY else 0)
    if len(payload) != expected:
        raise ValueError(f'payload {len(payload)}B, esperado {expected}B')

    q = np.frombuffer(payload, dtype='<i2', count=n * 3)
    xyz = np.empty((n, 3), dtype=np.float32)
    xyz[:, 0] = q[0:n] * scale + ox
    xyz[:, 1] = q[n:2 * n] * scale + oy
    xyz[:, 2] = q[2 * n:3 * n] * scale + oz

    inten = None
    if flags & FLAG_INTENSITY:
        inten = np.frombuffer(payload, dtype=np.uint8, count=n, offset=n * 6)

    return xyz, inten
