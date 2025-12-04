from http.server import HTTPServer, SimpleHTTPRequestHandler
import os
from urllib.parse import urlparse # Import for safe URL parsing

# -------------------------------
# Configuration
# -------------------------------
SERVE_DIR = "/app/terraindata"
PORT = 9010

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
        
        clean_path = urlparse(self.path).path
        requested_path = os.path.join(SERVE_DIR, clean_path.lstrip("/"))
        
        print(f"Requested path: {self.path}") # Logs the original path
        print(f"Cleaned path: {clean_path}") # Logs the path without the query string
        
        # If the file exists, we need to temporarily set self.path to the clean path
        # so that the super().do_GET() method serves the correct file.
        if os.path.isfile(requested_path):
            # Temporarily replace self.path with the clean path for the super method
            original_path = self.path
            self.path = clean_path 
            
            try:
                # File exists → serve normally
                return super().do_GET()
            finally:
                # Restore the original path after serving (good practice)
                self.path = original_path 
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