"""Read the M20 Pro calibration from the URDF for the joint bench."""

import argparse
import math
from pathlib import Path
from xml.etree import ElementTree


def read_motor_joints(urdf_path):
    return {
        joint.attrib['name']: joint
        for joint in ElementTree.parse(urdf_path).findall('joint')
        if joint.attrib['type'] in ('revolute', 'continuous')
    }


def revolute_limits(joint):
    limit = joint.find('limit')
    lower, upper = (float(limit.attrib[key]) for key in ('lower', 'upper'))
    if not (math.isfinite(lower) and math.isfinite(upper) and upper > lower):
        raise ValueError(f"Invalid limits for {joint.attrib['name']}")
    return lower, upper


def write_limits_header(urdf_path, output_path):
    limits = []
    suffix = '_joint'
    for name, joint in read_motor_joints(urdf_path).items():
        if joint.attrib['type'] == 'revolute' and name.endswith(suffix):
            lower, upper = revolute_limits(joint)
            motor_name = name[:-len(suffix)]
            limits.append(f'    {{"{motor_name}", {lower!r}F, {upper!r}F}},')
    output_path.write_text(
        '// Generated from M20_Pro/urdf/M20.urdf.\n'
        '#pragma once\n\n'
        '#include <string_view>\n\n'
        'namespace m20 {\n'
        'struct UrdfLimit { std::string_view name; float lower; float upper; };\n'
        'inline constexpr UrdfLimit pro_urdf_limits[] = {\n'
        + '\n'.join(limits) + '\n};\n}\n',
        encoding='utf-8',
    )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Generate the joint bench URDF limits header')
    parser.add_argument('urdf', type=Path)
    parser.add_argument('header', type=Path)
    args = parser.parse_args()
    write_limits_header(args.urdf, args.header)
