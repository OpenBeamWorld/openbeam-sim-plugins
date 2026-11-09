# SPDX-License-Identifier: GPL-3.0-or-later
"""tools/openbeam_cloud.py against a local stand-in for the account MCP endpoint.

    python -m pytest tools/tests -q
"""

from __future__ import annotations

import base64
import importlib.util
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / "openbeam_cloud.py"
spec = importlib.util.spec_from_file_location("openbeam_cloud", SCRIPT)
openbeam_cloud = importlib.util.module_from_spec(spec)
spec.loader.exec_module(openbeam_cloud)


class Endpoint:
    def __init__(self):
        self.requests = []
        self.answer = lambda params: {"structuredContent": {"ok": True}, "isError": False}
        endpoint = self

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
                endpoint.requests.append((dict(self.headers), body))
                if self.headers.get("Authorization") != "Bearer secret":
                    self.send_response(401)
                    self.end_headers()
                    self.wfile.write(b'{"error":"unauthorized"}')
                    return
                reply = json.dumps({"jsonrpc": "2.0", "id": body["id"], "result": endpoint.answer(body["params"])}).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(reply)))
                self.end_headers()
                self.wfile.write(reply)

            def log_message(self, *args):
                pass

        self.server = HTTPServer(("127.0.0.1", 0), Handler)
        self.url = f"http://127.0.0.1:{self.server.server_port}/mcp"
        threading.Thread(target=self.server.serve_forever, daemon=True).start()


@pytest.fixture
def endpoint(monkeypatch):
    e = Endpoint()
    monkeypatch.setenv("OPENBEAM_MCP_ACCESS_TOKEN", "secret")
    monkeypatch.setenv("OPENBEAM_MCP_URL", e.url)
    yield e
    e.server.shutdown()


def test_export_writes_the_document_field(endpoint, tmp_path, capsys):
    endpoint.answer = lambda p: {"structuredContent": {"jbeam": "{\"cable\": {}}", "nodes": 3}, "isError": False}
    out = tmp_path / "cable.jbeam"
    assert openbeam_cloud.main(["cloud_softbody_export", '{"actor": 3}', "--out", str(out)]) == 0
    assert out.read_text() == '{"cable": {}}'
    printed = json.loads(capsys.readouterr().out)
    assert printed == {"nodes": 3}
    _, body = endpoint.requests[-1]
    assert body["method"] == "tools/call"
    assert body["params"] == {"name": "cloud_softbody_export", "arguments": {"actor": 3}}


def test_text_and_attach_build_the_import_arguments(endpoint, tmp_path):
    urdf = tmp_path / "arm.urdf"
    urdf.write_text("<robot name='arm'/>")
    mesh = tmp_path / "base.stl"
    mesh.write_bytes(b"solid")
    openbeam_cloud.main(["cloud_robot_import", '{"format": "urdf"}', "--text", f"document={urdf}",
                         "--attach", f"arm/meshes/base.stl={mesh}"])
    arguments = endpoint.requests[-1][1]["params"]["arguments"]
    assert arguments["document"] == "<robot name='arm'/>"
    assert arguments["files"] == [{"path": "arm/meshes/base.stl", "content_base64": base64.b64encode(b"solid").decode()}]


def test_tool_errors_and_http_errors_exit_with_the_reason(endpoint, monkeypatch):
    endpoint.answer = lambda p: {"structuredContent": {"error": "training_session_required", "message": "start one"}, "isError": True}
    with pytest.raises(SystemExit, match="training_session_required"):
        openbeam_cloud.main(["cloud_robot_replay", '{"fps": 30, "frames": [[0]]}'])
    monkeypatch.setenv("OPENBEAM_MCP_ACCESS_TOKEN", "wrong")
    with pytest.raises(SystemExit, match="HTTP 401"):
        openbeam_cloud.main(["cloud_session_list"])


def test_an_event_stream_answer_is_read(monkeypatch):
    class Response:
        def __init__(self, text):
            self.text = text

        def read(self):
            return self.text.encode()

        def __enter__(self):
            return self

        def __exit__(self, *a):
            return False

    stream = 'event: message\ndata: {"jsonrpc":"2.0","id":1,"result":{"structuredContent":{"sessions":[]},"isError":false}}\n\n'
    monkeypatch.setattr(openbeam_cloud.urllib.request, "urlopen", lambda request, timeout: Response(stream))
    assert openbeam_cloud.call("http://x/mcp", "t", "cloud_session_list", {}) == {"sessions": []}
