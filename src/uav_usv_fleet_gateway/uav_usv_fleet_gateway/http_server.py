"""Static web and health endpoint server."""

from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import threading
from urllib.parse import unquote, urlparse


class FleetHttpServer:
    def __init__(self, host, port, web_root, health_factory):
        self.host = str(host)
        self.port = int(port)
        # Keep the install-space path itself. With --symlink-install each web
        # file resolves into the source tree and must still pass containment.
        self.web_root = Path(web_root).absolute()
        self.health_factory = health_factory
        self._server = None
        self._thread = None

    def start(self):
        outer = self

        class Handler(SimpleHTTPRequestHandler):
            def log_message(self, _format, *args):
                return

            def do_GET(self):
                path = urlparse(self.path).path
                if path == '/health':
                    body = json.dumps(
                        outer.health_factory(), separators=(',', ':')
                    ).encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Cache-Control', 'no-store')
                    self.send_header('Content-Length', str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                relative = 'index.html' if path in ('', '/') else unquote(
                    path.lstrip('/'))
                candidate = (outer.web_root / relative).absolute()
                if (outer.web_root not in candidate.parents
                        and candidate != outer.web_root):
                    self.send_error(403)
                    return
                if not candidate.is_file():
                    self.send_error(404)
                    return
                body = candidate.read_bytes()
                content_type = mimetypes.guess_type(str(candidate))[0]
                self.send_response(200)
                self.send_header(
                    'Content-Type', content_type or 'application/octet-stream')
                self.send_header('Cache-Control', 'no-cache')
                self.send_header('Content-Length', str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        self._server = ThreadingHTTPServer((self.host, self.port), Handler)
        self.port = int(self._server.server_address[1])
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name='fleet-http', daemon=True)
        self._thread.start()

    def stop(self):
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        self._server = None
        self._thread = None
