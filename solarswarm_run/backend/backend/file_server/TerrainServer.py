from http.server import HTTPServer, SimpleHTTPRequestHandler
import os
from urllib.parse import urlparse

# 
# Config
#
SERVE_DIR = "/app/terraindata"
PORT = 9010

#
# Simple HTTP-Server to serve terrain data (DEM) for
# Cesium Frontend.
#
class Terrain_Server(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=SERVE_DIR, **kwargs)


    def do_GET(self):
        clean_path = urlparse(self.path).path
        requested_path = os.path.join(SERVE_DIR, clean_path.lstrip("/"))

        if os.path.isfile(requested_path):
            original_path = self.path
            self.path = clean_path
            try:
                return super().do_GET()
            finally:
                self.path = original_path
        else:
            self.send_response(404, "Not Found")
            self.send_header("Content-type", "text/plain; charset=utf-8")
            self.end_headers()
            print(f"[ERR] Requested path {clean_path} was not found.")

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'public, max-age=0')
        super().end_headers()


    # Block directory listing completely
    def list_directory(self, path):
        self.send_response(404, "Directory listing disabled")
        self.send_header("Content-type", "text/plain; charset=utf-8")
        self.end_headers()
        return None


if __name__ == "__main__":
    server_address = ("0.0.0.0", PORT)
    httpd = HTTPServer(server_address, Terrain_Server)
    print(f"Serving {SERVE_DIR} on port {PORT}.")
    httpd.serve_forever()
