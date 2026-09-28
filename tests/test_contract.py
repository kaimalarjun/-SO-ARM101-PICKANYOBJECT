import json
import sys
from pathlib import Path
from http.server import ThreadingHTTPServer
import threading
import time
import unittest
import urllib.error
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros"))
from kendra_robot.gateway import Gateway, handler_for
from test_safety import CONFIG


class FakeVision:
    camera = {"lift_verification_roi_px": None, "placement_zone_roi_px": None}
    def look(self, query):
        return {"id": "fresh", "monotonic_time": time.monotonic(),
                "objects": [{"label": query or "red cube", "point": {"x": 0.12, "y": 0, "z": 0.04}}],
                "point": {"x": 0.12, "y": 0, "z": 0.04},
                "unique": True, "calibrated": True, "approach_clear": True,
                "zone_clear": True}


class ContractTests(unittest.TestCase):
    def setUp(self):
        gateway = Gateway(CONFIG)
        gateway.vision = FakeVision()
        self.server = ThreadingHTTPServer(("127.0.0.1", 0), handler_for(gateway))
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = f"http://127.0.0.1:{self.server.server_port}/robot/"

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()

    def post(self, method, data):
        request = urllib.request.Request(self.url + method, json.dumps(data).encode(),
                                         {"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request) as response:
                return response.status, json.load(response)
        except urllib.error.HTTPError as exc:
            return exc.code, json.load(exc)

    def test_look_pick_stop_contract(self):
        status, found = self.post("look", {"query": "red cube"})
        self.assertEqual(status, 200)
        self.assertEqual(found["objects"][0]["label"], "red cube")
        status, result = self.post("pick", {"x": 0.12, "y": 0, "z": 0.04,
                                            "observation_id": found["id"]})
        self.assertEqual(status, 200)
        self.assertEqual(result["task_success"], "uncertain")
        self.assertEqual(self.post("pick", {"x": 0.12, "y": 0, "z": 0.04,
                                             "observation_id": found["id"]})[0], 422)
        self.assertEqual(self.post("stop", {})[0], 200)
        self.assertEqual(self.post("home", {})[0], 422)


if __name__ == "__main__":
    unittest.main()
