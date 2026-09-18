"""Integration tests use the real portable MCU protocol over a pseudo-terminal.
Run after building /tmp/base101-calibration-protocol-test (see tools/calibration/README.md).
"""
import importlib.util
import json
import os
from pathlib import Path
import pty
import subprocess
import threading
import time
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

spec = importlib.util.spec_from_file_location("calibration_server", Path(__file__).resolve().parents[1] / "tools/calibration/server.py")
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)

class ConnectionTest(unittest.TestCase):
    def setUp(self):
        master, self.slave = pty.openpty()
        self.process = subprocess.Popen(["/tmp/base101-calibration-protocol-test", "--emulate"], stdin=master, stdout=master, stderr=subprocess.PIPE)
        os.close(master)
        self.link = server.SerialLink(os.ttyname(self.slave))
        deadline = time.monotonic() + 2
        while not self.link.snapshot()["connected"] and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(self.link.snapshot()["connected"], self.link.snapshot())
        self.http = server.http.server.ThreadingHTTPServer(("127.0.0.1", 0), server.Handler)
        self.http.link = self.link
        self.worker = threading.Thread(target=self.http.serve_forever, daemon=True)
        self.worker.start()
        self.url = f"http://127.0.0.1:{self.http.server_port}"

    def tearDown(self):
        self.http.shutdown()
        self.http.server_close()
        self.worker.join()
        self.link.close()
        os.close(self.slave)
        self.process.terminate()
        self.process.communicate(timeout=2)

    def post(self, path, body, origin=None):
        headers = {"Content-Type": "application/json"}
        if origin:
            headers["Origin"] = origin
        request = Request(self.url + path, json.dumps(body).encode(), headers, method="POST")
        with urlopen(request, timeout=2) as response:
            return json.load(response)

    def test_parameters_stop_and_heartbeat(self):
        params = {"ax": .8, "jx": 3, "aw": 4, "jw": 12, "k_icr": 1.7}
        self.assertEqual({k:v for k,v in self.post("/api/parameters", params)["parameters"].items() if k in params}, params)
        for invalid in [dict(params, ax=True), dict(params, k_icr=0), dict(params, jw=float("nan")), {"ax": 1}]:
            with self.assertRaises(HTTPError) as error:
                self.post("/api/parameters", invalid)
            self.assertEqual(error.exception.code, 400)
        time.sleep(.75)  # Longer than MCU lease: server must keep it alive.
        self.assertTrue(self.link.snapshot()["status"]["session"])
        self.assertEqual(self.post("/api/stop", {})["stop_reason"], "requested")
        self.assertEqual({k:v for k,v in self.link.snapshot()["status"]["parameters"].items() if k in params}, params)
        with self.assertRaises(HTTPError) as error:
            self.post("/api/stop", {}, "http://unrelated.example")
        self.assertEqual(error.exception.code, 403)

    def test_disconnect_clears_connection(self):
        self.process.terminate()
        self.process.wait(timeout=2)
        deadline = time.monotonic() + 2
        while self.link.snapshot()["connected"] and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertFalse(self.link.snapshot()["connected"])
        with self.assertRaises(ConnectionError):
            self.link.request("stop")

    def test_operator_deadman_ordering_and_stop(self):
        identity = {"client": "test_browser_001", "seq": 1}
        self.post("/api/arm", identity)
        self.post("/api/drive", dict(identity, seq=2, vx=.1, wz=.3))
        time.sleep(.12)
        self.assertTrue(self.link.snapshot()["status"]["motion_enabled"])
        # A live server heartbeat must not keep driving after the browser stops sending.
        time.sleep(.5)
        snapshot = self.link.snapshot()
        self.assertTrue(snapshot["connected"])
        self.assertFalse(snapshot["status"]["motion_enabled"])
        self.assertFalse(snapshot["operator"]["armed"])
        with self.assertRaises(HTTPError):
            self.post("/api/drive", dict(identity, seq=3, vx=.1, wz=0))
        self.post("/api/arm", dict(identity, seq=4))
        self.post("/api/drive", dict(identity, seq=5, vx=.1, wz=0))
        self.post("/api/release", dict(identity, seq=6))
        with self.assertRaises(HTTPError):
            self.post("/api/drive", dict(identity, seq=5, vx=.1, wz=0))
        self.post("/api/stop", {})
        self.assertFalse(self.link.snapshot()["operator"]["armed"])
        self.assertFalse(self.link.snapshot()["status"]["motion_enabled"])

    def test_recording_and_static_assets(self):
        self.post("/api/record", {"action": "start"})
        time.sleep(.25)
        self.post("/api/record", {"action": "stop"})
        with urlopen(self.url + "/api/recording", timeout=2) as response:
            samples = json.load(response)["samples"]
        self.assertGreater(len(samples), 0)
        self.assertIn("parameters", samples[0])
        for path in ["/", "/app.js", "/app.css", "/theme.css"]:
            with urlopen(self.url + path, timeout=2) as response:
                self.assertEqual(response.status, 200)
                self.assertGreater(len(response.read()), 50)


class MalformedReplyTest(unittest.TestCase):
    def test_malformed_json_is_not_hidden_as_timeout(self):
        master, slave = pty.openpty()
        process = subprocess.Popen(["/tmp/base101-calibration-protocol-test", "--malformed"], stdin=master, stdout=master)
        os.close(master)
        link = server.SerialLink(os.ttyname(slave))
        try:
            deadline = time.monotonic() + 2
            while not link.snapshot()["error"] and time.monotonic() < deadline:
                time.sleep(.01)
            self.assertFalse(link.snapshot()["connected"])
            self.assertIn("malformed JSON", link.snapshot()["error"] or "")
        finally:
            link.close(); os.close(slave)
            if process.poll() is None:
                process.terminate()
            process.wait(timeout=2)

if __name__ == "__main__":
    unittest.main()
