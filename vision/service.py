"""Loopback-only Qwen + Grounding DINO visual grounding service.

Model loading is deliberately lazy: health checks do not allocate GPU memory.
"""
from __future__ import annotations

import argparse
import base64
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import threading

from PIL import Image


class Models:
    def __init__(self, qwen_id: str, dino_id: str) -> None:
        self.qwen_id = qwen_id
        self.dino_id = dino_id
        self._lock = threading.Lock()
        self._captioner = None
        self._grounder = None

    def load(self) -> None:
        if self._captioner is not None:
            return
        from transformers import pipeline
        self._captioner = pipeline("image-text-to-text", model=self.qwen_id, device_map="auto")
        self._grounder = pipeline("zero-shot-object-detection", model=self.dino_id, device_map="auto")

    def infer(self, image: Image.Image, query: str | None, placement_roi: list[int] | None) -> dict:
        with self._lock:
            self.load()
            if query:
                names = [query.strip().lower()]
            else:
                messages = [{"role": "user", "content": [
                    {"type": "image", "image": image},
                    {"type": "text", "text": "List only small rigid isolated tabletop objects, comma separated, no explanation."}]}]
                generated = self._captioner(text=messages, max_new_tokens=80)
                description = generated[0].get("generated_text", "")
                if isinstance(description, list):
                    description = description[-1].get("content", "")
                description = str(description)
                names = [s.strip().lower() for s in description.split(",") if s.strip()][:5]
            if not names:
                return {"objects": [], "isolated": False,
                        "zone_clear": self.zone_clear(image, placement_roi)}
            detections = self._grounder(image, candidate_labels=names,
                                        threshold=0.45)
        objects = []
        for det in detections[:10]:
            box = det["box"]
            left, top, right, bottom = (box[k] for k in ("xmin", "ymin", "xmax", "ymax"))
            if right <= left or bottom <= top:
                continue
            objects.append({"label": det["label"], "score": float(det["score"]),
                            "box": [left, top, right, bottom],
                            "pixel_center": [(left + right) / 2, (top + bottom) / 2]})
        return {"objects": objects,
                "isolated": len(objects) == 1 and self.is_isolated(image, objects[0]["box"]),
                "zone_clear": self.zone_clear(image, placement_roi)}

    @staticmethod
    def is_isolated(image: Image.Image, box: list[int]) -> bool:
        import cv2
        import numpy as np
        x0, y0, x1, y1 = [int(v) for v in box]
        margin = 24
        if x0 < margin or y0 < margin or x1 + margin >= image.width or y1 + margin >= image.height:
            return False
        lab = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2LAB).astype(np.float32)
        ring = np.concatenate([lab[y0-margin:y0, x0-margin:x1+margin].reshape(-1, 3),
                               lab[y1:y1+margin, x0-margin:x1+margin].reshape(-1, 3),
                               lab[y0:y1, x0-margin:x0].reshape(-1, 3),
                               lab[y0:y1, x1:x1+margin].reshape(-1, 3)])
        color = np.median(ring, axis=0)
        return float(np.mean(np.linalg.norm(ring - color, axis=1) > 18)) < 0.03

    @staticmethod
    def zone_clear(image: Image.Image, roi: list[int] | None) -> bool:
        if not roi or len(roi) != 4:
            return False
        import cv2
        import numpy as np
        x0, y0, x1, y1 = roi
        if (x0 < 20 or y0 < 20 or x1 > image.width - 20 or y1 > image.height - 20
                or x1 - x0 < 30 or y1 - y0 < 30):
            return False
        lab = cv2.cvtColor(np.asarray(image), cv2.COLOR_RGB2LAB).astype(np.float32)
        region = lab[y0:y1, x0:x1]
        border = np.concatenate([lab[y0-15:y0, x0:x1].reshape(-1, 3),
                                 lab[y1:y1+15, x0:x1].reshape(-1, 3),
                                 lab[y0:y1, x0-15:x0].reshape(-1, 3),
                                 lab[y0:y1, x1:x1+15].reshape(-1, 3)])
        table_color = np.median(border, axis=0)
        difference = np.linalg.norm(region - table_color, axis=2)
        return float(np.mean(difference > 18.0)) < 0.01


def make_handler(models: Models) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def reply(self, code: int, body: dict) -> None:
            raw = json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self) -> None:
            self.reply(200, {"ready": True, "models_loaded": models._captioner is not None}) if self.path == "/health" else self.reply(404, {"error": "not found"})

        def do_POST(self) -> None:
            if self.path != "/infer":
                self.reply(404, {"error": "not found"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if not 0 < length <= 8_000_000:
                    raise ValueError("invalid request size")
                data = json.loads(self.rfile.read(length))
                raw = base64.b64decode(data["image_b64"], validate=True)
                if len(raw) > 5_000_000:
                    raise ValueError("image too large")
                image = Image.open(io.BytesIO(raw)).convert("RGB")
                if image.width * image.height > 4_000_000:
                    raise ValueError("image dimensions too large")
                self.reply(200, models.infer(image, data.get("query"), data.get("placement_zone_roi_px")))
            except (KeyError, ValueError, OSError) as exc:
                self.reply(422, {"error": str(exc)})

        def log_message(self, format: str, *args: object) -> None:
            pass

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--qwen", default="Qwen/Qwen3.5-4B")
    parser.add_argument("--dino", default="IDEA-Research/grounding-dino-tiny")
    args = parser.parse_args()
    if args.host not in ("127.0.0.1", "::1"):
        parser.error("vision service must bind to loopback")
    ThreadingHTTPServer((args.host, args.port), make_handler(Models(args.qwen, args.dino))).serve_forever()


if __name__ == "__main__":
    main()
