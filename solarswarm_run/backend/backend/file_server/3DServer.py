from http.server import HTTPServer, SimpleHTTPRequestHandler
import os

# -------------------------------
# Configuration
# -------------------------------
SERVE_DIR = "/app/data"
PORT = 9005

class Debug404Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=SERVE_DIR, **kwargs)

    def log_message(self, format, *args):
        # suppress all logs:
        pass

    def end_headers(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Cache-Control', 'public, max-age=0')
        super().end_headers()

    def do_GET(self):
        # Build the absolute path of requested file
        requested_path = os.path.join(SERVE_DIR, self.path.lstrip("/"))
        if os.path.isfile(requested_path):
            # File exists → serve normally
            return super().do_GET()
        else:
            # File missing or directory → return debug 404
            self.send_response(404, "Not Found")
            self.send_header("Content-type", "text/plain; charset=utf-8")
            self.end_headers()
            self.wfile.write(
                f"DEBUG 404\nRequested path: {self.path}\nResolved path: {requested_path}\n".encode("utf-8")
            )

    # Also block directory listing completely
    def list_directory(self, path):
        self.send_response(404, "Directory listing disabled")
        self.send_header("Content-type", "text/plain; charset=utf-8")
        self.end_headers()
        self.wfile.write(f"DEBUG 404\nDirectory listing disabled for: {path}\n".encode("utf-8"))
        return None

if __name__ == "__main__":
    server_address = ("0.0.0.0", PORT)
    httpd = HTTPServer(server_address, Debug404Handler)
    print(f"Serving {SERVE_DIR} on port {PORT}!\n\n")
    httpd.serve_forever()
