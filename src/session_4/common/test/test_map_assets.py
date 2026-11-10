"""실제 배포 자산의 차로 폭·통로·참조 경로·센서 높이를 검사한다."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


ROOT = Path(__file__).resolve().parents[1]
MODELS = ROOT / 'models'
WORLD = ROOT / 'worlds' / 'collaboration_track.sdf'
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def load_mesh(name):
    doc = ET.parse(MODELS / 'session4_track' / 'meshes' / name).getroot()
    mesh = doc.find('c:library_geometries/c:geometry/c:mesh', NS)
    values = [float(value) for value in mesh.find("c:source[@id='positions']/c:float_array", NS).text.split()]
    positions = [tuple(values[i:i + 3]) for i in range(0, len(values), 3)]
    triangles = mesh.find('c:triangles', NS)
    inputs = triangles.findall('c:input', NS)
    stride = max(int(item.get('offset')) for item in inputs) + 1
    vertex_offset = int(next(item for item in inputs if item.get('semantic') == 'VERTEX').get('offset'))
    indices = [int(value) for value in triangles.find('c:p', NS).text.split()]
    faces = [tuple(indices[i + corner * stride + vertex_offset] for corner in range(3))
             for i in range(0, len(indices), 3 * stride)]
    return mesh, positions, faces


def components(positions, faces):
    neighbors = {i: set() for i in range(len(positions))}
    for face in faces:
        for a in face:
            neighbors[a].update(face)
    unseen, groups = set(neighbors), []
    while unseen:
        pending, group = [unseen.pop()], set()
        while pending:
            index = pending.pop()
            group.add(index)
            fresh = neighbors[index] & unseen
            unseen.difference_update(fresh)
            pending.extend(fresh)
        groups.append([positions[index] for index in group])
    return groups


def resolve_uri(uri):
    assert uri.startswith('model://'), f'외부 네트워크 자산: {uri}'
    path = MODELS / uri.removeprefix('model://')
    return path / 'model.sdf' if path.is_dir() else path


def test_world_and_nested_model_uris_resolve_from_any_working_directory(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    documents = [WORLD] + list(MODELS.glob('*/model.sdf'))
    for document in documents:
        root = ET.parse(document).getroot()
        for element in root.iter('uri'):
            assert resolve_uri(element.text).is_file(), (document, element.text)
    assert {'session4_car', 'session4_track', 'session4_cone'} <= {p.name for p in MODELS.iterdir() if p.is_dir()}


def test_actual_road_has_two_equal_1_5_meter_lanes_with_continuous_white_boundaries():
    _, road, faces = load_mesh('road.dae')
    ys = [y for x, y, _ in road if abs(x) < 7.9 and y < 0]
    assert max(ys) - min(ys) == pytest.approx(3.0)
    _, yellow, _ = load_mesh('center_dashes.dae')
    center_ys = [y for x, y, _ in yellow if abs(x) < 7.9 and y < 0]
    center = (min(center_ys) + max(center_ys)) / 2
    assert center - min(ys) == pytest.approx(1.5)
    assert max(ys) - center == pytest.approx(1.5)
    assert len(components(road, faces)) == 1
    track = ET.parse(MODELS / 'session4_track/model.sdf').getroot()
    for name, expected_y in [('inner_line', -3.54), ('outer_line', -6.46)]:
        _, vertices, triangles = load_mesh(name + '.dae')
        edge = [y for x, y, _ in vertices if abs(x) < 7.9 and y < 0]
        assert (min(edge) + max(edge)) / 2 == pytest.approx(expected_y)
        assert max(edge) - min(edge) == pytest.approx(.08)
        assert len(components(vertices, triangles)) == 1, '흰 경계선은 끊어지면 안 됩니다.'
        material = track.find(f"model/link/visual[@name='{name}']/material/diffuse")
        assert tuple(map(float, material.text.split())) == (1, 1, 1, 1)


def test_center_paint_is_yellow_and_has_real_gaps_in_generated_mesh():
    _, vertices, triangles = load_mesh('center_dashes.dae')
    groups = components(vertices, triangles)
    assert len(groups) >= 30, '연결된 노란 실선 대신 서로 떨어진 점선이어야 합니다.'
    bottom = [group for group in groups if all(abs(x) < 7.9 and y < -4.9 for x, y, _ in group)]
    intervals = sorted((min(p[0] for p in group), max(p[0] for p in group)) for group in bottom)
    assert len(intervals) > 5
    for start, end in intervals:
        assert .7 < end - start < 1.0
    for (_, end), (start, _) in zip(intervals, intervals[1:]):
        assert start - end > .5
    track = ET.parse(MODELS / 'session4_track/model.sdf').getroot()
    color = tuple(map(float, track.find("model/link/visual[@name='yellow_dashed_center']/material/diffuse").text.split()))
    assert color[0] > .9 and color[1] > .7 and color[2] < .1


def test_spawn_is_centered_in_right_lane_and_faces_along_bottom_straight():
    world = ET.parse(WORLD).getroot().find('world')
    car = next(item for item in world.findall('include') if item.findtext('uri') == 'model://session4_car')
    x, y, z, roll, pitch, yaw = map(float, car.findtext('pose').split())
    assert -8 < x < -2 and y == pytest.approx(-5.75)
    assert z >= 0 and (roll, pitch, yaw) == (0, 0, 0)
    # 진행방향(+x)의 오른쪽은 -y. 중심 점선과 바깥 흰 선 사이에 출발한다.
    assert -6.46 < y < -5.0
    assert -5.0 - y == pytest.approx(.75)


def test_cone_corridor_fits_whole_vehicle_and_intersects_lidar_scan_height():
    world = ET.parse(WORLD).getroot().find('world')
    cones = [item for item in world.findall('include') if item.findtext('uri') == 'model://session4_cone']
    assert len(cones) >= 12 and len(cones) % 2 == 0
    rows = {}
    for cone in cones:
        x, y, z, *_ = map(float, cone.findtext('pose').split())
        assert y > 4.8, '라바콘은 신호등 반대편 직선에 배치합니다.'
        rows.setdefault(x, []).append((y, z))
    car = ET.parse(MODELS / 'session4_car/model.sdf').getroot().find('model')
    wheel_edges = []
    for link in car.findall('link'):
        if link.get('name').endswith('_wheel'):
            wheel_y = float(link.findtext('pose').split()[1])
            half_thickness = float(link.findtext('collision/geometry/cylinder/length')) / 2
            wheel_edges.extend([wheel_y - half_thickness, wheel_y + half_thickness])
    car_width = max(wheel_edges) - min(wheel_edges)
    cone = ET.parse(MODELS / 'session4_cone/model.sdf').getroot()
    cone_radius = float(cone.findtext('model/link/collision/geometry/cylinder/radius'))
    cone_height = float(cone.findtext('model/link/collision/geometry/cylinder/length'))
    for pair in rows.values():
        assert len(pair) == 2
        clear_gap = abs(pair[0][0] - pair[1][0]) - 2 * cone_radius
        assert clear_gap > car_width + .25, '바퀴 전체가 통과하고도 좌우 여유가 있어야 합니다.'
    chassis = car.find("link[@name='chassis']")
    chassis_z = float(chassis.findtext('pose').split()[2])
    lidar_z = float(chassis.findtext("sensor[@name='lidar']/pose").split()[2]) + chassis_z + .015
    assert .1 < lidar_z < cone_height, '라이다가 라바콘 위로 지나가면 안 됩니다.'
    assert min(rows) > -4 and max(rows) < 4, '직선 구간 안에 라바콘 배열이 있어야 합니다.'


@pytest.mark.parametrize('name', ['road.dae', 'inner_line.dae', 'outer_line.dae', 'center_dashes.dae'])
def test_road_mesh_has_upward_faces_and_explicit_unit_normals(name):
    mesh, vertices, triangles = load_mesh(name)
    normals = [float(value) for value in mesh.find("c:source[@id='normals']/c:float_array", NS).text.split()]
    assert mesh.find("c:triangles/c:input[@semantic='NORMAL']", NS) is not None
    for i in range(0, len(normals), 3):
        assert math.sqrt(sum(x * x for x in normals[i:i + 3])) == pytest.approx(1.0, abs=1e-6)
        assert normals[i + 2] > .99
    for a, b, c in triangles:
        a, b, c = vertices[a], vertices[b], vertices[c]
        assert (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]) > 0


def test_vehicle_commands_are_isolated_from_physical_motor_topic():
    bridge = (ROOT / 'config/bridge.yaml').read_text()
    assert '/motor_control' not in bridge
    assert '/session4/sim/cmd_vel' in bridge
    car = ET.parse(MODELS / 'session4_car/model.sdf').getroot()
    command = car.findtext("model/plugin[@name='gz::sim::systems::AckermannSteering']/topic")
    assert command == '/model/session4_car/cmd_vel'
    assert command in bridge


def test_signal_lamps_have_unique_service_parent_and_clear_the_vehicle_body():
    world = ET.parse(WORLD).getroot().find('world')
    bodies = world.findall("model/link[@name='traffic_light_body']")
    assert len(bodies) == 1, 'visual_config가 이름으로 찾을 링크는 하나여야 합니다.'
    body = bodies[0]
    for color in ('red', 'yellow', 'green'):
        lamp = body.find(f"visual[@name='{color}_lamp']")
        x, y, z, *_ = map(float, lamp.findtext('pose').split())
        assert x > 5 and -6.5 < y < -5
        assert z == pytest.approx(.95)
        assert float(lamp.findtext('geometry/sphere/radius')) == pytest.approx(.10)
    housing = body.find("visual[@name='housing']")
    bottom = float(housing.findtext('pose').split()[2]) - float(housing.findtext('geometry/box/size').split()[2]) / 2
    assert bottom > .6, '신호등 본체는 차량과 라이다 위에 있어야 합니다.'
