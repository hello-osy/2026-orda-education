#!/usr/bin/env python3
"""외부 모델 없이 Session 4의 도로·차선·라바콘·월드를 다시 생성한다."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
NS = 'http://www.collada.org/2005/11/COLLADASchema'
HALF_STRAIGHT, RADIUS, ROAD_WIDTH = 8.0, 5.0, 3.0
LENGTH = 4 * HALF_STRAIGHT + 2 * math.pi * RADIUS


def sub(parent, tag, value=None, **attrs):
    element = ET.SubElement(parent, tag, attrs)
    if value is not None:
        element.text = str(value)
    return element


def write_xml(path, root):
    path.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root, space='  ')
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def mesh(path, vertices, triangles, color):
    """색·단위법선이 명시된 COLLADA. Metal에서도 아스팔트와 도색을 구분한다."""
    doc = ET.Element('COLLADA', xmlns=NS, version='1.4.1')
    asset = sub(doc, 'asset')
    sub(asset, 'created', '2026-11-10T00:00:00')
    sub(asset, 'modified', '2026-11-10T00:00:00')
    sub(asset, 'unit', name='meter', meter='1')
    sub(asset, 'up_axis', 'Z_UP')
    effect = sub(sub(doc, 'library_effects'), 'effect', id='paint-effect')
    lambert = sub(sub(sub(effect, 'profile_COMMON'), 'technique', sid='common'), 'lambert')
    for field in ('ambient', 'diffuse'):
        sub(sub(lambert, field), 'color', color)
    material = sub(sub(doc, 'library_materials'), 'material', id='paint')
    sub(material, 'instance_effect', url='#paint-effect')
    geometry = sub(sub(doc, 'library_geometries'), 'geometry', id='shape')
    body = sub(geometry, 'mesh')
    # 각 면의 법선을 별도로 저장해 라바콘 곡면에도 올바른 조명이 적용된다.
    normals = []
    for a, b, c in triangles:
        p, q, r = vertices[a], vertices[b], vertices[c]
        u, v = [q[i]-p[i] for i in range(3)], [r[i]-p[i] for i in range(3)]
        n = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
        length = math.sqrt(sum(x*x for x in n))
        normals.append([x/length for x in n])
    for name, values in [('positions', vertices), ('normals', normals)]:
        source = sub(body, 'source', id=name)
        sub(source, 'float_array', ' '.join(f'{x:.7f}' for row in values for x in row),
            id=name+'-array', count=str(len(values)*3))
        accessor = sub(sub(source, 'technique_common'), 'accessor', source='#'+name+'-array',
                       count=str(len(values)), stride='3')
        for axis in 'XYZ':
            sub(accessor, 'param', name=axis, type='float')
    sub(sub(body, 'vertices', id='vertices'), 'input', semantic='POSITION', source='#positions')
    group = sub(body, 'triangles', count=str(len(triangles)), material='paint')
    sub(group, 'input', semantic='VERTEX', source='#vertices', offset='0')
    sub(group, 'input', semantic='NORMAL', source='#normals', offset='1')
    sub(group, 'p', ' '.join(f'{v} {i}' for i, triangle in enumerate(triangles) for v in triangle))
    node = sub(sub(sub(doc, 'library_visual_scenes'), 'visual_scene', id='scene'), 'node', id='node')
    instance = sub(node, 'instance_geometry', url='#shape')
    sub(sub(sub(instance, 'bind_material'), 'technique_common'), 'instance_material', symbol='paint', target='#paint')
    sub(sub(doc, 'scene'), 'instance_visual_scene', url='#scene')
    write_xml(path, doc)


def point(s, offset):
    """반시계 방향 중심선의 누적 거리와 바깥쪽 오프셋 → xy."""
    s %= LENGTH
    straight = 2 * HALF_STRAIGHT
    arc = math.pi * RADIUS
    if s < straight:
        return -HALF_STRAIGHT+s, -RADIUS-offset
    if s < straight+arc:
        a = -math.pi/2+(s-straight)/RADIUS
        return HALF_STRAIGHT+(RADIUS+offset)*math.cos(a), (RADIUS+offset)*math.sin(a)
    if s < 2*straight+arc:
        return HALF_STRAIGHT-(s-straight-arc), RADIUS+offset
    a = math.pi/2+(s-2*straight-arc)/RADIUS
    return -HALF_STRAIGHT+(RADIUS+offset)*math.cos(a), (RADIUS+offset)*math.sin(a)


def ribbons(intervals, offset, width):
    vertices, triangles = [], []
    for start, end in intervals:
        steps = max(1, math.ceil((end-start)/.08))
        base = len(vertices)
        for i in range(steps+1):
            s = start+(end-start)*i/steps
            for edge in (offset-width/2, offset+width/2):
                vertices.append((*point(s, edge), 0.0))
        for i in range(steps):
            a = base+2*i
            triangles += [(a, a+1, a+2), (a+1, a+3, a+2)]
    return vertices, triangles


def material(parent, color, emissive=False):
    node = sub(parent, 'material')
    for field in ('ambient', 'diffuse'):
        sub(node, field, color)
    if emissive:
        sub(node, 'emissive', color)


def geometry(parent, kind, **dimensions):
    node = sub(sub(parent, 'geometry'), kind)
    for field, value in dimensions.items():
        sub(node, field, value)


def visual(parent, name, pose, color, kind, **dimensions):
    node = sub(parent, 'visual', name=name)
    sub(node, 'pose', pose)
    geometry(node, kind, **dimensions)
    material(node, color)
    return node


def model_config(path, name):
    doc = ET.Element('model')
    sub(doc, 'name', name)
    sub(doc, 'version', '1.0')
    sub(doc, 'sdf', 'model.sdf', version='1.9')
    sub(doc, 'description', 'ORDA Session 4: generated local assets')
    write_xml(path/'model.config', doc)


def generate_track():
    folder = ROOT/'models/session4_track'
    pieces = [('road', 0, ROAD_WIDTH, '.10 .12 .14 1'),
              ('inner_line', -ROAD_WIDTH/2+.04, .08, '1 1 1 1'),
              ('outer_line', ROAD_WIDTH/2-.04, .08, '1 1 1 1')]
    doc = ET.Element('sdf', version='1.9')
    body = sub(sub(doc, 'model', name='session4_track'), 'link', name='surface')
    sub(doc.find('model'), 'static', 'true')
    for name, offset, width, color in pieces:
        vertices, triangles = ribbons([(0, LENGTH)], offset, width)
        mesh(folder/'meshes'/f'{name}.dae', vertices, triangles, color)
        visual(body, name, f'0 0 {0.003 if name == "road" else 0.006} 0 0 0', color,
               'mesh', uri=f'model://session4_track/meshes/{name}.dae')
    dashes = [(s, min(s+.85, LENGTH)) for s in [i*1.6 for i in range(math.ceil(LENGTH/1.6))]]
    vertices, triangles = ribbons(dashes, 0, .08)
    mesh(folder/'meshes/center_dashes.dae', vertices, triangles, '1 .83 .04 1')
    visual(body, 'yellow_dashed_center', '0 0 .006 0 0 0', '1 .83 .04 1', 'mesh',
           uri='model://session4_track/meshes/center_dashes.dae')
    write_xml(folder/'model.sdf', doc)
    model_config(folder, 'session4_track')


def generate_cone():
    folder = ROOT/'models/session4_cone'
    groups = {'orange': ([], [], '1 .24 .015 1'), 'white': ([], [], '.97 .97 .97 1')}
    bands = [(0.04, .27, 'orange'), (.27, .36, 'white'), (.36, .51, 'orange'),
             (.51, .59, 'white'), (.59, .75, 'orange')]
    for z0, z1, color in bands:
        vertices, triangles, _ = groups[color]
        base = len(vertices)
        count = 32
        for z in (z0, z1):
            radius = .18-(z-.04)/.71*.15
            for i in range(count):
                a = 2*math.pi*i/count
                vertices.append((radius*math.cos(a), radius*math.sin(a), z))
        for i in range(count):
            j = (i+1)%count
            triangles += [(base+i, base+j, base+count+i), (base+j, base+count+j, base+count+i)]
    doc = ET.Element('sdf', version='1.9')
    model = sub(doc, 'model', name='session4_cone')
    sub(model, 'static', 'true')
    body = sub(model, 'link', name='cone_body')
    for name, (vertices, triangles, color) in groups.items():
        mesh(folder/'meshes'/f'{name}.dae', vertices, triangles, color)
        visual(body, name, '0 0 0 0 0 0', color, 'mesh', uri=f'model://session4_cone/meshes/{name}.dae')
    visual(body, 'base', '0 0 .02 0 0 0', '.045 .045 .045 1', 'box', size='.40 .40 .04')
    collision = sub(body, 'collision', name='cone_collision')
    sub(collision, 'pose', '0 0 .375 0 0 0')
    geometry(collision, 'cylinder', radius='.18', length='.75')
    write_xml(folder/'model.sdf', doc)
    model_config(folder, 'session4_cone')


def generate_car():
    source = ROOT.parent.parent/'session_3/models/orda_car/model.sdf'
    doc = ET.parse(source).getroot()
    model = doc.find('model')
    model.set('name', 'session4_car')
    model.find("plugin[@name='gz::sim::systems::AckermannSteering']/topic").text = '/model/session4_car/cmd_vel'
    model.find("link[@name='chassis']/sensor[@name='high_camera']/pose").text = '.34 0 .22 0 .05 0'
    # 카메라 교재와 동일한 센서 토픽을 유지하며 제어 토픽만 실차와 분리한다.
    write_xml(ROOT/'models/session4_car/model.sdf', doc)
    model_config(ROOT/'models/session4_car', 'session4_car')


def generate_world():
    doc = ET.parse(ROOT.parent.parent/'session_3/worlds/solid_track.sdf').getroot()
    world = doc.find('world')
    world.set('name', 'session4_track')
    for node in list(world):
        if node.tag in ('model', 'include'):
            world.remove(node)
    world.find("gui/plugin[@name='3D View']/gz-gui/title").text = 'ORDA Session 4 · Collaborative Driving'
    world.find("gui/plugin[@name='3D View']/camera_pose").text = '-18 -22 27 0 .80 .88'
    ground = sub(world, 'model', name='ground')
    sub(ground, 'static', 'true')
    sub(ground, 'pose', '0 0 -.05 0 0 0')
    body = sub(ground, 'link', name='ground_body')
    geometry(sub(body, 'collision', name='ground_collision'), 'box', size='40 24 .10')
    visual(body, 'grass', '0 0 0 0 0 0', '.14 .34 .16 1', 'box', size='40 24 .10')
    sub(sub(world, 'include'), 'uri', 'model://session4_track')
    car = sub(world, 'include')
    sub(car, 'uri', 'model://session4_car')
    sub(car, 'pose', '-6 -5.75 .015 0 0 0')
    # 신호등 반대편 직선에 배치합니다. 기존 배열 전체를 180도 회전해 간격·진행 순서를 유지합니다.
    for i in range(7):
        x = -2 + i*.8
        center = -5.75+.16*math.sin(i*math.pi/3)
        half_gap = .68 if i not in (0, 6) else .74
        for side in (-1, 1):
            item = sub(world, 'include')
            sub(item, 'uri', 'model://session4_cone')
            sub(item, 'name', f'cone_{i}_{"left" if side > 0 else "right"}')
            sub(item, 'pose', f'{-x:.3f} {-(center+side*half_gap):.3f} .005 0 0 {math.pi:.6f}')
    # 정지선은 우측 차로에만 칠한다. 신호등은 상부에 달아 차량과 충돌하지 않는다.
    line = sub(world, 'model', name='stop_line')
    sub(line, 'static', 'true')
    body = sub(line, 'link', name='stop_line_body')
    visual(body, 'paint', '4.8 -5.75 .008 0 0 0', '1 1 1 1', 'box', size='.10 1.42 .004')
    signal = sub(world, 'model', name='traffic_light')
    sub(signal, 'static', 'true')
    body = sub(signal, 'link', name='traffic_light_body')
    visual(body, 'pole', '6 -6.85 .83 0 0 0', '.12 .13 .14 1', 'cylinder', radius='.045', length='1.66')
    visual(body, 'arm', '6 -6.3 1.48 0 0 0', '.12 .13 .14 1', 'box', size='.07 1.15 .07')
    visual(body, 'housing', '6 -5.75 .95 0 0 0', '.035 .035 .04 1', 'box', size='.18 .82 .32')
    for color, y in [('red', -5.50), ('yellow', -5.75), ('green', -6.00)]:
        rgba = '1 .02 .01 1' if color == 'red' else '.015 .015 .015 1'
        lamp = visual(body, color+'_lamp', f'5.885 {y} .95 0 0 0', rgba, 'sphere', radius='.10')
        sub(lamp.find('material'), 'emissive', rgba)
    write_xml(ROOT/'worlds/collaboration_track.sdf', doc)


def main():
    generate_track()
    generate_cone()
    generate_car()
    generate_world()
    print('생성 완료: 2차선 육상트랙 · 노란 점선 · 흰 실선 · 라바콘 14개 · 신호등 · 차량')


if __name__ == '__main__':
    main()
