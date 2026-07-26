"""Minimal reader for the building boxes emitted by the RoadRunner stage."""

import re
import struct


def _varint(data, index):
    value = 0
    shift = 0
    while index < len(data):
        byte = data[index]
        if not isinstance(byte, int):
            byte = ord(byte)
        index += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, index
        shift += 7
        if shift > 63:
            raise ValueError("invalid protobuf varint")
    raise ValueError("truncated protobuf varint")


def _triple(message):
    values = {}
    index = 0
    while index < len(message):
        key, index = _varint(message, index)
        field, wire = key >> 3, key & 7
        if wire == 1:
            if index + 8 > len(message):
                break
            values[field] = struct.unpack("<d", message[index:index + 8])[0]
            index += 8
        elif wire == 2:
            length, index = _varint(message, index)
            index += length
        elif wire == 0:
            _, index = _varint(message, index)
        else:
            break
    return values


def read_building_boxes(path):
    """Read BldgN_M boxes as dictionaries in patch-local metres."""
    with open(path, "rb") as stream:
        data = stream.read()
    boxes = []
    for match in re.finditer(rb"\x0a(.)Bldg", data, re.DOTALL):
        raw = match.group(1)
        name_length = raw[0] if not isinstance(raw[0], str) else ord(raw[0])
        name_start = match.start() + 2
        name = data[name_start:name_start + name_length]
        if not re.match(rb"Bldg[0-9]+_[0-9]+$", name):
            continue
        index = name_start + name_length
        if index >= len(data) or data[index:index + 1] != b"\x12":
            continue
        geometry_length, geometry_start = _varint(data, index + 1)
        geometry = data[geometry_start:geometry_start + geometry_length]
        fields = {}
        cursor = 0
        while cursor < len(geometry):
            key, cursor = _varint(geometry, cursor)
            field, wire = key >> 3, key & 7
            if wire == 2:
                length, cursor = _varint(geometry, cursor)
                fields[field] = geometry[cursor:cursor + length]
                cursor += length
            elif wire == 1:
                cursor += 8
            elif wire == 0:
                _, cursor = _varint(geometry, cursor)
            else:
                break
        center = _triple(fields.get(1, b""))
        dimension = _triple(fields.get(2, b""))
        yaw = 0.0
        orientation = fields.get(3)
        if orientation:
            _, inner_start = _varint(orientation, 0)
            inner_length, inner_start = _varint(orientation, inner_start)
            yaw = _triple(orientation[inner_start:inner_start + inner_length]).get(3, 0.0)
        height = 2.0 * center.get(3, 0.0)
        boxes.append({
            "id": name.decode("ascii"),
            "x": center.get(1, 0.0),
            "y": center.get(2, 0.0),
            "z": 0.0,
            "half_x": dimension.get(1, 0.0),
            "half_y": dimension.get(2, 0.0),
            "height": height if height > 0.0 else 10.0,
            "yaw_rad": yaw,
            "source": "roadrunner_rrhd",
            "confidence": "high",
        })
    return boxes
