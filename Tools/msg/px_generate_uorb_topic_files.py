#!/usr/bin/env python3
"""Emit uORB topic headers from the .msg files this step keeps.

main's copy of this script drives the EmPy templates. It sorts fields by
size, inserts padding, and also emits ORB_DECLARE, topic sources, and IDL.
This copy writes one header per message and leaves fields in file order.
"""

import argparse
import os
import re
import sys

TYPE_MAP = {
    "uint64": "uint64_t",
    "uint32": "uint32_t",
    "uint16": "uint16_t",
    "uint8": "uint8_t",
    "int64": "int64_t",
    "int32": "int32_t",
    "int16": "int16_t",
    "int8": "int8_t",
    "float32": "float",
    "float64": "double",
    "bool": "bool",
}

CONSTANT_RE = re.compile(r"^(\w+)\s+([A-Z][A-Z0-9_]*)\s*=\s*(-?\d+)$")
FIELD_RE = re.compile(r"^(\w+)(?:\[(\d+)\])?\s+([A-Za-z_][A-Za-z0-9_]*)$")


def snake_case(name):
    name = re.sub(r"(.)([A-Z][a-z]+)", r"\1_\2", name)
    name = re.sub(r"([a-z0-9])([A-Z])", r"\1_\2", name)
    return name.lower()


def cpp_type(msg_type):
    try:
        return TYPE_MAP[msg_type]
    except KeyError:
        raise SystemExit("unsupported msg type: %s" % msg_type)


def parse_msg(path):
    fields = []
    constants = []
    with open(path, "r", encoding="utf-8") as handle:
        for raw in handle:
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            constant = CONSTANT_RE.match(line)
            if constant:
                msg_type, name, value = constant.groups()
                constants.append((cpp_type(msg_type), name, value))
                continue
            field = FIELD_RE.match(line)
            if not field:
                raise SystemExit("%s: cannot parse %r" % (path, line))
            msg_type, count, name = field.groups()
            fields.append((cpp_type(msg_type), count, name))
    return fields, constants


def render(struct_name, alias, fields, constants):
    lines = [
        "#pragma once",
        "",
        "#include <stdint.h>",
        "",
        "struct %s {" % struct_name,
    ]
    for cpp, count, name in fields:
        if count is None:
            lines.append("\t%s %s;" % (cpp, name))
        else:
            lines.append("\t%s %s[%s];" % (cpp, name, count))
    for cpp, name, value in constants:
        lines.append("\tstatic constexpr %s %s = %s;" % (cpp, name, value))
    lines += [
        "};",
        "",
        "namespace px4 {",
        "namespace msg {",
        "using %s = %s;" % (alias, struct_name),
        "} // namespace msg",
        "} // namespace px4",
        "",
    ]
    return "\n".join(lines)


def generate(path, out_dir):
    alias = os.path.splitext(os.path.basename(path))[0]
    struct_name = snake_case(alias) + "_s"
    fields, constants = parse_msg(path)
    header = os.path.join(out_dir, snake_case(alias) + ".h")
    with open(header, "w", encoding="utf-8") as handle:
        handle.write(render(struct_name, alias, fields, constants))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headers", action="store_true")
    parser.add_argument("-f", nargs="+", required=True)
    parser.add_argument("-o", required=True)
    parser.add_argument("-i", nargs="+")
    parser.add_argument("-e")
    args = parser.parse_args()
    if not args.headers:
        raise SystemExit("this copy only implements --headers")
    os.makedirs(args.o, exist_ok=True)
    for path in args.f:
        generate(path, args.o)


if __name__ == "__main__":
    sys.exit(main())
