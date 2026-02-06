from http.server import HTTPServer, SimpleHTTPRequestHandler
import os

#
#  Config
#
SERVE_DIR = "/app/data"
PORT = 9005

#
# Simple HTTP-Server to serve 3D-Mesh data for
# Cesium Frontend.
#
class Three_D_Server(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=SERVE_DIR, **kwargs)

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'public, max-age=0')
        super().end_headers()

    def do_GET(self):
        requested_path = os.path.join(SERVE_DIR, self.path.lstrip("/"))
        if os.path.isfile(requested_path):
            return super().do_GET()
        else:
            self.send_response(404, "Not Found")
            self.send_header("Content-type", "text/plain; charset=utf-8")
            self.end_headers()
            print(f"[ERR] Requested path {self.path} was not found.")

    # Also block directory listing completely
    def list_directory(self, path):
        self.send_response(404, "Directory listing disabled")
        self.send_header("Content-type", "text/plain; charset=utf-8")
        self.end_headers()
        self.wfile.write(f"DEBUG 404\nDirectory listing disabled for: {path}\n".encode("utf-8"))
        return None

if __name__ == "__main__":
    server_address = ("0.0.0.0", PORT)
    httpd = HTTPServer(server_address, Three_D_Server)
    print(f"Serving {SERVE_DIR} on port {PORT}.")
    httpd.serve_forever()
