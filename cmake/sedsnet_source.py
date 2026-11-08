"""Select an online development branch or the last usable on-disk SEDSnet without editing it."""
import argparse
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def git(*args, timeout=30):
    return subprocess.run(['git', *map(str, args)], check=True, text=True,
                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                          timeout=timeout,
                          env={**os.environ, 'GIT_TERMINAL_PROMPT': '0'})


def usable(path):
    return all((path / f).is_file() for f in ('CMakeLists.txt', 'Cargo.toml', 'build.rs'))


def missing_apis(path, apis):
    """Return the required C APIs the source header does not declare."""
    if not apis:
        return []
    header = path / 'C-Headers' / 'sedsnet.h'
    try:
        text = header.read_text(errors='replace')
    except OSError:
        return list(apis)
    return [api for api in apis if f'{api}(' not in text]


def select_source(cache, repository, fallbacks, offline=False, timeout=15, ref="main", require_apis=()):
    if ref not in ("main", "dev"):
        raise ValueError("SEDSnet ref must be main or dev")
    cache.mkdir(parents=True, exist_ok=True)
    marker = cache / 'last-source.txt'
    if marker.exists():
        fallbacks = [Path(marker.read_text().strip()), *fallbacks]
    upstream = cache / 'upstream.git'
    if not offline:
        if not upstream.exists():
            git('init', '--bare', upstream)
        try:
            git('--git-dir', upstream, 'fetch', '--depth=1', '--no-tags', repository,
                f"+refs/heads/{ref}:refs/heads/{ref}", timeout=timeout)
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as error:
            print(f'SEDSnet {ref} unavailable; using on-disk fallback ({type(error).__name__}).', file=sys.stderr)
        else:
            revision = git('--git-dir', upstream, 'rev-parse', f"refs/heads/{ref}").stdout.strip()
            destination = cache / revision
            if not destination.exists():
                # Build preparation modifies Cargo.toml/schema. Keep the upstream
                # object store pristine and never reset an existing working copy.
                with tempfile.TemporaryDirectory(prefix='checkout-', dir=cache) as tmp:
                    stage = Path(tmp) / 'source'
                    git('clone', '--shared', '--no-checkout', upstream, stage)
                    git('-C', stage, 'checkout', '--detach', revision)
                    if not usable(stage):
                        raise RuntimeError(f'Fetched {ref} is not a usable SEDSnet source tree')
                    lacking = missing_apis(stage, require_apis)
                    if lacking:
                        raise RuntimeError(f'Fetched {ref} lacks required SEDSnet API {", ".join(lacking)}')
                    stage.rename(destination)
            marker_tmp = marker.with_suffix('.tmp')
            marker_tmp.write_text(str(destination) + '\n')
            marker_tmp.replace(marker)
            print(f'SEDSnet {ref}: {revision}', file=sys.stderr)
            return destination
    # Previously fetched revisions are complete, pristine-at-fetch copies;
    # consider them (newest first) after the explicit fallbacks.
    fetched = sorted((d for d in cache.iterdir() if d.is_dir() and len(d.name) == 40
                      and all(c in '0123456789abcdef' for c in d.name)),
                     key=lambda d: d.stat().st_mtime, reverse=True)
    seen = set()
    for source in [*fallbacks, *fetched]:
        key = str(source)
        if key in seen:
            continue
        seen.add(key)
        if usable(source):
            lacking = missing_apis(source, require_apis)
            if lacking:
                # A stale cache must not win over a compatible on-disk source.
                print(f'Skipping incompatible SEDSnet source {source} (lacks {", ".join(lacking)}).',
                      file=sys.stderr)
                continue
            try:
                revision = git('-C', source, 'rev-parse', 'HEAD').stdout.strip()
            except subprocess.CalledProcessError:
                revision = 'vendored source (no Git revision)'
            print(f'SEDSnet offline/local: {source} [{revision}]', file=sys.stderr)
            return source
    raise RuntimeError('No usable SEDSnet source (with the required API) is available on disk. Connect '
                       'once to fetch main, or set FETCHCONTENT_SOURCE_DIR_SEDSNET to an existing source checkout.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cache', type=Path, required=True)
    parser.add_argument('--repository', required=True)
    parser.add_argument('--fallback', action='append', default=[], type=Path)
    parser.add_argument('--offline', action='store_true')
    parser.add_argument('--ref', choices=('main', 'dev'), default='main')
    parser.add_argument('--require-api', action='append', default=[],
                        help='C API the selected header must declare; incompatible fallbacks are skipped.')
    args = parser.parse_args()
    try:
        print(select_source(args.cache.resolve(), args.repository, args.fallback, args.offline, ref=args.ref,
                            require_apis=args.require_api))
    except (RuntimeError, OSError, subprocess.SubprocessError) as error:
        parser.exit(1, f'{error}\n')


if __name__ == '__main__':
    main()
