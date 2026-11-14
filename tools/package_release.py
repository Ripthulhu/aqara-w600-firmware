"""Package a verified release application and optionally attach its OTA assets."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import tempfile

from package_ota import build_package, sha256, STREAM_PATH, write_new_or_identical


def prepare(inputs, stock, output):
    images = list(inputs.glob('w600-controller*.bin'))
    if len(images) != 1:
        raise ValueError('Release must contain exactly one controller application')
    image = images[0]
    match = re.fullmatch(r'w600-controller([1-9][0-9]*)\.bin', image.name)
    if not match:
        raise ValueError('Unexpected application filename')
    revision = int(match[1])
    entries = []
    for line in (inputs / 'SHA256SUMS').read_text().splitlines():
        entry = re.fullmatch(r'([0-9a-fA-F]{64}) [ *](\S+)', line)
        if entry and entry[2] == image.name:
            entries.append(entry[1].lower())
    data = image.read_bytes()
    if entries != [sha256(data)]:
        raise ValueError('Release application checksum missing, duplicated, or mismatched')
    package, facts = build_package(stock.read_bytes(), data, revision, STREAM_PATH.read_bytes())
    name = f'w600-custom{revision}.ota'
    contents = {
        name: package,
        name + '.json': (json.dumps(facts, indent=2) + '\n').encode(),
        name + '.sha256': (sha256(package) + '  ' + name + '\n').encode(),
    }
    output.mkdir(parents=True, exist_ok=True)
    for name, body in contents.items():
        write_new_or_identical(output / name, body)
    return [output / name for name in contents]


def publish(tag, files):
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9._/-]{0,127}', tag):
        raise ValueError('Invalid release tag')
    release = json.loads(subprocess.check_output(
        ['gh', 'release', 'view', tag, '--json', 'assets'], text=True))
    names = {asset['name'] for asset in release['assets']}
    missing = []
    with tempfile.TemporaryDirectory(prefix='w600-release-') as folder:
        for file in files:
            if file.name not in names:
                missing.append(file)
                continue
            subprocess.run(['gh', 'release', 'download', tag, '--pattern', file.name,
                            '--dir', folder], check=True)
            if (Path(folder) / file.name).read_bytes() != file.read_bytes():
                raise ValueError('Refusing to replace different release asset: ' + file.name)
    if missing:
        subprocess.run(['gh', 'release', 'upload', tag, *map(str, missing)], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--stock', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--publish', metavar='RELEASE_TAG')
    args = parser.parse_args()
    files = prepare(args.input, args.stock, args.output)
    if args.publish:
        publish(args.publish, files)
    print('\n'.join(str(file) for file in files))


if __name__ == '__main__':
    main()
