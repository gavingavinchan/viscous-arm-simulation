#!/usr/bin/env python3
"""Local model preview server with an in-memory gzip cache and configurable bind."""
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path
import argparse
import gzip
import os
import threading
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parent.parent
COMPRESS = {'.stl', '.urdf', '.xml', '.json', '.js', '.html', '.css', '.txt'}
CACHE = {}
LOCK = threading.Lock()

def compressed(path):
    stat = path.stat()
    stamp = (stat.st_mtime_ns, stat.st_size)
    with LOCK:
        entry = CACHE.get(str(path))
        if entry and entry[0] == stamp:
            return entry[1], stat
        payload = gzip.compress(path.read_bytes(), compresslevel=5, mtime=0)
        CACHE[str(path)] = (stamp, payload)
        return payload, stat

class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def send_head(self):
        path = Path(self.translate_path(self.path))
        if path.is_dir():
            if not urlsplit(self.path).path.endswith('/'):
                return super().send_head()
            path = path / 'index.html'
        accepts_gzip = any(x.strip().split(';')[0] == 'gzip' and 'q=0' not in x
                           for x in self.headers.get('Accept-Encoding', '').split(','))
        if not accepts_gzip or path.suffix.lower() not in COMPRESS or not path.is_file():
            return super().send_head()
        try:
            body, stat = compressed(path)
        except OSError:
            return super().send_head()
        etag = '"%x-%x-gzip"' % (stat.st_mtime_ns, stat.st_size)
        if self.headers.get('If-None-Match') == etag:
            self.send_response(304)
            self.send_header('ETag', etag)
            self.send_header('Vary', 'Accept-Encoding')
            self.end_headers()
            return None
        self.send_response(200)
        self.send_header('Content-Type', self.guess_type(str(path)))
        self.send_header('Content-Encoding', 'gzip')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Last-Modified', self.date_time_string(stat.st_mtime))
        self.send_header('ETag', etag)
        self.send_header('Vary', 'Accept-Encoding')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        from io import BytesIO
        return BytesIO(body)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8766)
    args = parser.parse_args()
    originals = packed = 0
    for folder in [ROOT / 'model', ROOT / 'viewer']:
        for path in folder.rglob('*'):
            if path.is_file() and path.suffix.lower() in COMPRESS:
                data, stat = compressed(path)
                originals += stat.st_size
                packed += len(data)
    print(f'Gzip cache: {originals:,} -> {packed:,} bytes ({packed / max(originals, 1):.1%})', flush=True)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f'Serving http://{args.host}:{args.port}/viewer/', flush=True)
    server.serve_forever()
