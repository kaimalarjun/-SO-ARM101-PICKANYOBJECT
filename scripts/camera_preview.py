#!/usr/bin/env python3
"""Two-camera browser preview on localhost. Does not control the robot."""
import argparse
import json
from pathlib import Path
from urllib.parse import urlsplit
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time

import cv2


class Camera:
    def __init__(self, device):
        self.frame = None
        self.sequence = 0
        self.timestamp = 0.0
        self.condition = threading.Condition()
        self.capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.capture.set(cv2.CAP_PROP_FPS, 30)
        if not self.capture.isOpened():
            raise RuntimeError("Cannot open camera: " + device)
        threading.Thread(target=self.read, daemon=True).start()

    def read(self):
        last_encode = 0
        while True:
            ok, frame = self.capture.read()
            if not ok:
                with self.condition:
                    self.frame = None
                    self.condition.notify_all()
                time.sleep(0.1)
                continue
            now = time.monotonic()
            if now - last_encode < 0.1:
                continue
            ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
            if ok:
                with self.condition:
                    self.frame = jpeg.tobytes()
                    self.timestamp = time.time()
                    self.sequence += 1
                    self.condition.notify_all()
                last_encode = now


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--external", default="/dev/video0")
    parser.add_argument("--wrist", default="/dev/video2")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--telemetry-file", type=Path)
    args = parser.parse_args()
    cameras = {"external": Camera(args.external), "wrist": Camera(args.wrist)}

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == "/":
                body = PREVIEW_HTML.encode()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/telemetry":
                try:
                    data = json.loads(args.telemetry_file.read_text()) if args.telemetry_file else {}
                    data["age_s"] = max(0, time.time() - data.get("timestamp", 0))
                    data["fresh"] = data["age_s"] < 1 and "joints" in data
                except (OSError, ValueError):
                    data = {"fresh": False}
                body = json.dumps(data).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            if path == "/camera-state":
                data = {}
                for name, camera in cameras.items():
                    with camera.condition:
                        age = time.time() - camera.timestamp
                        data[name] = {"sequence": camera.sequence,
                                      "timestamp": camera.timestamp,
                                      "age_s": age,
                                      "fresh": camera.frame is not None and age < .5}
                body = json.dumps(data).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
            if path.startswith('/snapshot/'):
                camera = cameras.get(path.removeprefix('/snapshot/'))
                if camera is None:
                    self.send_error(404)
                    return
                with camera.condition:
                    frame = camera.frame
                    timestamp, sequence = camera.timestamp, camera.sequence
                if frame is None or time.time() - timestamp > .5:
                    self.send_error(503, 'Camera unavailable')
                    return
                self.send_response(200)
                self.send_header('Content-Type', 'image/jpeg')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', str(len(frame)))
                self.send_header('X-Frame-Timestamp', str(timestamp))
                self.send_header('X-Frame-Sequence', str(sequence))
                self.end_headers()
                self.wfile.write(frame)
                return
            camera = cameras.get(path.strip("/"))
            if camera is None:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            sequence = -1
            try:
                while True:
                    with camera.condition:
                        camera.condition.wait_for(lambda: camera.frame is not None and camera.sequence != sequence, timeout=2)
                        if camera.frame is None or camera.sequence == sequence:
                            return
                        frame, sequence = camera.frame, camera.sequence
                        timestamp = camera.timestamp
                    self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                                     str(len(frame)).encode() + b"\r\nX-Frame-Timestamp: " +
                                     str(timestamp).encode() + b"\r\nX-Frame-Sequence: " +
                                     str(sequence).encode() + b"\r\n\r\n" + frame + b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass

        def log_message(self, *_):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"Camera preview listening on localhost:{args.port}", flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        for camera in cameras.values():
            camera.capture.release()


PREVIEW_HTML = '''<!doctype html><meta name="viewport" content="width=device-width"><title>Robot cameras and coordinates</title>
<style>
body{background:#11151b;color:#edf2f7;font:11px system-ui;margin:10px}h1{font-size:15px;margin:0 0 5px}h2{font-size:11px;margin:6px 0 4px}p{font-size:9px;margin:4px 0}p{color:#b9c4d3}
.feeds{display:grid;grid-template-columns:1fr 1fr;gap:10px}.feed{position:relative}img{width:100%;height:32vh;max-height:300px;object-fit:contain;background:#090c10;border-radius:5px;display:block}
.legend{position:absolute;bottom:5px;left:5px;background:#111d;padding:3px;border-radius:3px;font-size:9px}
.cards{display:flex;gap:8px;flex-wrap:wrap}.card{background:#1d2530;border-left:4px solid;padding:4px 8px;border-radius:4px;min-width:90px;font-size:10px}.value{font-size:14px;font-variant-numeric:tabular-nums}
.x{color:#ff6b6b}.y{color:#55dc89}.z{color:#6aafff}.card.x{border-color:#ff6b6b}.card.y{border-color:#55dc89}.card.z{border-color:#6aafff}
table{border-collapse:collapse;width:100%;font-size:10px}th,td{text-align:left;padding:3px 6px;border-bottom:1px solid #303c4b}td{font-variant-numeric:tabular-nums}#status{color:#ffce73}

</style>
<h1>Robot preview</h1>
<div id="torque" style="padding:7px 10px;margin-bottom:8px;border-radius:4px;font-size:12px;font-weight:bold;background:#805b00;color:white">TORQUE UNKNOWN · Do not move arm by hand</div>
<div class="feeds">
<section><h2>External camera</h2><div class="feed"><img src="/external"><div class="legend"><span class="x">X</span> · <span class="y">Y</span> · <span class="z">Z</span> — robot base axes; not aligned to this image</div></div></section>
<section><h2>Wrist camera</h2><div class="feed"><img src="/wrist"><div class="legend">Camera moves with the wrist · image pixels are not robot coordinates</div></div></section>
</div>
<h2>Estimated claw position · metres · robot base</h2>
<div class="cards"><div class="card x">X · base-frame X<div class="value" id="x">—</div></div><div class="card y">Y · base-frame Y<div class="value" id="y">—</div></div><div class="card z">Z · height above base<div class="value" id="z">—</div></div></div>
<p id="status">Waiting for joint feedback…</p>
<p>XYZ and model angles are provisional.</p>

<h2>Six live joints · normalized −100…+100; claw 0…100%</h2>
<table><thead><tr><th>Axis / joint</th><th>Model angle</th><th>Normalized position</th><th>Encoder ticks / range</th></tr></thead><tbody id="joints"></tbody></table>
<script>
document.querySelectorAll('.feeds img').forEach(img=>{
 const source=img.getAttribute('src');
 const reconnect=()=>{img.src=source+'?retry='+Date.now()};
 img.addEventListener('error',()=>setTimeout(reconnect,1000));
 setInterval(reconnect,30000);
});
const names=['Base swivel','Shoulder lift','Elbow bend','Wrist bend','Wrist rotation','Claw opening'];
const colors=['#ff6b6b','#ffad66','#ffdc70','#55dc89','#6aafff','#d49aff'];
function torqueStatus(d){
 const bar=document.getElementById('torque');
 const off=d&&d.fresh&&d.torque_off===true;
 const active=d&&d.fresh&&d.torque_off===false;
 bar.style.background=off?'#16733a':active?'#b52222':'#805b00';
 bar.textContent=off?'TORQUE OFF · Support arm before manual positioning':active?'TORQUE ACTIVE · Do not move arm by hand':'TORQUE UNKNOWN · Do not move arm by hand';
}
async function update(){
 try{
  const r=await fetch('/telemetry',{cache:'no-store'});const d=await r.json();
  torqueStatus(d);
  for(const a of ['x','y','z'])document.getElementById(a).textContent=d.fresh&&d.xyz_m?d.xyz_m[a].toFixed(3)+' m':'—';
  document.getElementById('status').textContent=d.fresh?'Live encoder feedback · '+d.age_s.toFixed(2)+' s old · '+(d.torque_off?'motor torque off':'motor torque enabled')+(d.within_model_limits?'':' · outside provisional model bounds'):'Feedback unavailable or stale — coordinates hidden';
  const tbody=document.getElementById('joints');tbody.replaceChildren();
  if(d.fresh) d.joints.forEach((j,i)=>{const row=document.createElement('tr');[(i+1)+' · '+names[i],j.degrees.toFixed(1)+'°',j.normalized.toFixed(1)+(i===5?'%':''),j.ticks+' / '+j.range_min+'–'+j.range_max+(j.in_range?'':' (outside range)')].forEach((v,k)=>{const cell=document.createElement('td');cell.textContent=v;if(k===0)cell.style.color=colors[i];row.appendChild(cell)});tbody.appendChild(row)});
 }catch(e){torqueStatus(null);for(const a of ['x','y','z'])document.getElementById(a).textContent='—';document.getElementById('status').textContent='Connection lost — coordinates hidden';document.getElementById('joints').replaceChildren()}
 setTimeout(update,200);
}update();
</script>'''

if __name__ == "__main__":
    main()
