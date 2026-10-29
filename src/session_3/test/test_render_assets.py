"""법선이 빠져 Gazebo Metal에서 트랙 전체가 흰색이 되던 오류를 검사한다."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


MESH_DIR = Path(__file__).resolve().parents[1] / 'models' / 'orda_track' / 'meshes'
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def source_vectors(mesh, reference):
    """COLLADA accessor가 지정한 범위와 stride로 좌표/법선을 읽는다."""
    source = mesh.find(f"c:source[@id='{reference.removeprefix('#')}']", NS)
    assert source is not None, f'없는 source: {reference}'
    accessor = source.find('c:technique_common/c:accessor', NS)
    assert accessor is not None
    array_id = accessor.attrib['source'].removeprefix('#')
    array = source.find(f"c:float_array[@id='{array_id}']", NS)
    assert array is not None
    values = [float(value) for value in (array.text or '').split()]
    assert len(values) == int(array.attrib['count'])
    assert all(math.isfinite(value) for value in values)
    count = int(accessor.attrib['count'])
    stride = int(accessor.attrib.get('stride', '1'))
    offset = int(accessor.attrib.get('offset', '0'))
    assert count > 0 and stride >= 3 and offset >= 0
    assert offset + (count - 1) * stride + 3 <= len(values)
    return [values[offset + i * stride:offset + i * stride + 3] for i in range(count)]


@pytest.mark.parametrize('filename', ['road.dae', 'inner_line.dae', 'outer_line.dae'])
def test_track_triangles_have_valid_explicit_normals(filename):
    root = ET.parse(MESH_DIR / filename).getroot()
    meshes = root.findall('c:library_geometries/c:geometry/c:mesh', NS)
    assert meshes, f'{filename}: 메시가 없습니다.'
    for mesh in meshes:
        triangles = mesh.findall('c:triangles', NS)
        assert triangles, f'{filename}: 삼각형이 없습니다.'
        for group in triangles:
            inputs = group.findall('c:input', NS)
            normal_inputs = [item for item in inputs if item.get('semantic') == 'NORMAL']
            assert len(normal_inputs) == 1, (
                f'{filename}: 명시적인 NORMAL 입력이 필요합니다. '
                '법선이 없으면 Ogre2/Metal에서 차선과 도로의 색이 구분되지 않습니다.')
            normal_input = normal_inputs[0]
            normals = source_vectors(mesh, normal_input.attrib['source'])
            for normal in normals:
                assert math.sqrt(sum(value * value for value in normal)) == pytest.approx(1.0)

            vertex_input = next(item for item in inputs if item.get('semantic') == 'VERTEX')
            vertices_id = vertex_input.attrib['source'].removeprefix('#')
            positions_input = mesh.find(
                f"c:vertices[@id='{vertices_id}']/c:input[@semantic='POSITION']", NS)
            assert positions_input is not None
            positions = source_vectors(mesh, positions_input.attrib['source'])
            stride = max(int(item.attrib['offset']) for item in inputs) + 1
            normal_offset = int(normal_input.attrib['offset'])
            vertex_offset = int(vertex_input.attrib['offset'])
            indices_element = group.find('c:p', NS)
            assert indices_element is not None
            indices = [int(value) for value in (indices_element.text or '').split()]
            triangle_count = int(group.attrib['count'])
            assert triangle_count > 0
            assert len(indices) == triangle_count * 3 * stride

            for start in range(0, len(indices), 3 * stride):
                triangle_positions, triangle_normals = [], []
                for corner in range(3):
                    base = start + corner * stride
                    vertex_index = indices[base + vertex_offset]
                    normal_index = indices[base + normal_offset]
                    assert 0 <= vertex_index < len(positions)
                    assert 0 <= normal_index < len(normals), f'{filename}: 잘못된 법선 인덱스'
                    triangle_positions.append(positions[vertex_index])
                    triangle_normals.append(normals[normal_index])
                a, b, c = triangle_positions
                u = [b[i] - a[i] for i in range(3)]
                v = [c[i] - a[i] for i in range(3)]
                surface_normal = [u[1] * v[2] - u[2] * v[1],
                                  u[2] * v[0] - u[0] * v[2],
                                  u[0] * v[1] - u[1] * v[0]]
                assert surface_normal[2] > 0, f'{filename}: 도로 삼각형이 위를 향해야 합니다.'
                for normal in triangle_normals:
                    assert sum(normal[i] * surface_normal[i] for i in range(3)) > 0, (
                        f'{filename}: 법선이 삼각형 표면의 반대쪽을 향합니다.')
