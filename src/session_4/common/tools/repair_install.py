"""이전 폴더 배치를 사용하던 colcon의 끊어진 생성 링크만 정리한다."""
import os
import argparse
from pathlib import Path


def repair(root):
    root = Path(root).resolve()
    share = root / 'install/session_4/share/session_4'
    old = root / 'build/session_4'
    repaired = 0
    if not share.is_dir():
        return 0
    for link in share.rglob('*'):
        if not link.is_symlink() or link.exists():
            continue
        target = Path(os.path.normpath(str(link.parent / os.readlink(link))))
        try:
            relative = target.relative_to(old)
        except ValueError:
            continue
        if relative.parts and relative.parts[0] in ('launch', 'config', 'models', 'worlds'):
            link.unlink()
            repaired += 1
    return repaired


def restore_asset_alias(root):
    """colcon이 패키지 이름으로 바꾼 뒤 제거하는 공통 자산 경로를 연결한다."""
    root = Path(root).resolve()
    link = root / 'build/session_4/common'
    target = root / 'src/session_4/common'
    link.parent.mkdir(parents=True, exist_ok=True)
    if link.is_symlink():
        if link.resolve() == target:
            pass
        else:
            raise RuntimeError(f'예상하지 못한 빌드 링크를 보존했습니다: {link}')
    elif link.exists():
        raise RuntimeError(f'기존 폴더를 덮어쓰지 않습니다: {link}')
    else:
        link.symlink_to(target, target_is_directory=True)

    # Gazebo 자산은 colcon이 빌드 중 제거하는 common 별칭을 거치지 않습니다.
    # 수동주행 등을 함께 실행해도 이미 켜진 Gazebo의 모델 경로가 유지됩니다.
    share = root / 'install/session_4/share/session_4'
    for folder in ('launch', 'config', 'worlds', 'models', 'weights'):
        source = target / folder
        for asset in source.rglob('*'):
            if not asset.is_file() or '__pycache__' in asset.parts:
                continue
            installed = share / folder / asset.relative_to(source)
            if installed.is_symlink() and os.readlink(installed) != str(asset):
                installed.unlink()
                installed.symlink_to(asset)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--after-build', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[4]
    if args.after_build:
        restore_asset_alias(root)
    else:
        count = repair(root)
        if count:
            print(f'이전 빌드의 끊어진 링크 {count}개 정리')
