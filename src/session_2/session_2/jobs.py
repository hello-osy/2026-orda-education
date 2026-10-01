"""GUI에서 QProcess로 호출하는 긴 작업. 학습/추출 로그를 즉시 전달한다."""
import argparse
import json
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest='job', required=True)
    extract = sub.add_parser('extract')
    extract.add_argument('--bag', required=True)
    extract.add_argument('--output', required=True)
    extract.add_argument('--topics', nargs='+', required=True)
    extract.add_argument('--stride', type=int, default=5)
    train = sub.add_parser('train')
    train.add_argument('--data', required=True)
    train.add_argument('--output', required=True)
    train.add_argument('--model', default='yolov8n.pt')
    train.add_argument('--epochs', type=int, default=30)
    train.add_argument('--batch', type=int, default=4)
    train.add_argument('--imgsz', type=int, default=640)
    train.add_argument('--device', choices=['auto','cuda','mps','cpu'], default='auto')
    args = parser.parse_args()
    try:
        if args.job == 'extract':
            from .bagio import extract_frames
            result = extract_frames(args.bag, args.output, args.topics, args.stride)
        else:
            from .learning import train as run_train
            result = run_train(args.data, args.output, args.model, args.epochs, args.batch, args.imgsz, args.device)
        (Path(args.output)/'result.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        print('완료: '+json.dumps(result, ensure_ascii=False), flush=True)
        return 0
    except Exception as exc:
        print(f'작업 실패: {exc}', file=sys.stderr, flush=True)
        return 1


if __name__ == '__main__':
    sys.exit(main())
