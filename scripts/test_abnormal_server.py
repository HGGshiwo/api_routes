#!/usr/bin/env python3
import http.server
import socketserver
import json
import cgi
import os

PORT = 8001

# 16x16 pure black PNG image bytes
BLACK_PNG_BYTES = (
    b"\x89PNG\r\n\x1a\n\x00\x00\x00\rIHDR\x00\x00\x00\x10\x00\x00\x00\x10"
    b"\x08\x02\x00\x00\x00\x90\x91h6\x00\x00\x00\x0cIDATx\x9cc` \x05\x00"
    b"\x00\x00\xff\xff\x03\x00\x00\x06\x00\x05\x57\xbf\xab\xd4\x00\x00"
    b"\x00\x00IEND\xaeB`\x82"
)

class AbnormalTestHandler(http.server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/test_image.jpg" or self.path == "/test_image.png":
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(BLACK_PNG_BYTES)))
            self.end_headers()
            self.wfile.write(BLACK_PNG_BYTES)
            print("[MockServer] Served 16x16 black PNG image.")
        else:
            self.send_response(404)
            self.end_headers()
            self.wfile.write(b"404 Not Found")

    def do_POST(self):
        if self.path == "/abnormal/report":
            content_type = self.headers.get('Content-Type')
            print(f"\n==================== [MockServer Received POST /abnormal/report] ====================")
            print(f"Content-Type: {content_type}")

            form_data = {}
            files_data = []

            if content_type and 'multipart/form-data' in content_type:
                form = cgi.FieldStorage(
                    fp=self.rfile,
                    headers=self.headers,
                    environ={'REQUEST_METHOD': 'POST', 'CONTENT_TYPE': self.headers['Content-Type']}
                )

                for key in form.keys():
                    field = form[key]
                    if isinstance(field, list):
                        for item in field:
                            if item.filename:
                                content = item.file.read()
                                files_data.append((key, item.filename, len(content)))
                            else:
                                form_data[key] = item.value
                    else:
                        if field.filename:
                            content = field.file.read()
                            files_data.append((key, field.filename, len(content)))
                        else:
                            form_data[key] = field.value

            print("\n[Form Fields]:")
            for k, v in form_data.items():
                print(f"  - {k}: {v}")

            print("\n[Uploaded Files]:")
            for field_name, filename, size in files_data:
                print(f"  - Field: '{field_name}', Filename: '{filename}', Size: {size} bytes")

            print(f"====================================================================================\n")

            response = {
                "status": "success",
                "message": "Abnormal report received successfully",
                "received_fields": list(form_data.keys()),
                "received_files_count": len(files_data)
            }

            resp_bytes = json.dumps(response).encode('utf-8')
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(resp_bytes)))
            self.end_headers()
            self.wfile.write(resp_bytes)
        else:
            self.send_response(404)
            self.end_headers()

def run_server():
    with socketserver.TCPServer(("", PORT), AbnormalTestHandler) as httpd:
        print(f"=========================================================")
        print(f" Abnormal Mock Server listening on http://0.0.0.0:{PORT}")
        print(f" Image Download URL: http://localhost:{PORT}/test_image.png")
        print(f" Report Target URL: http://localhost:{PORT}/abnormal/report")
        print(f"=========================================================")
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer shutting down.")

if __name__ == "__main__":
    run_server()
