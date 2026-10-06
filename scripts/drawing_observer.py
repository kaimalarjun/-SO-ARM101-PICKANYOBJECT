#!/usr/bin/env python3
"""Record two existing MJPEG feeds with fresh joint evidence; never commands motion.

Uses OpenCV pyramidal Lucas-Kanade tracking with a forward/backward check.
Image flow is a diagnostic, not calibrated pen depth or a motion authorization.
Keep recordings and the output directory outside Git.
"""
import argparse
import json
import signal
from pathlib import Path
import threading
import time
import urllib.request

import cv2
import numpy as np


class Feed:
    def __init__(self, url):
        self.latest = None
        self.error = None
        self.lock = threading.Lock()
        threading.Thread(target=self.read, args=(url,), daemon=True).start()

    def read(self, url):
        while True:
            try:
                with urllib.request.urlopen(url, timeout=3) as stream:
                    while True:
                        line = stream.readline()
                        if not line:
                            raise EOFError('Stream ended')
                        if not line.startswith(b'--frame'):
                            continue
                        headers = {}
                        while True:
                            line = stream.readline()
                            if line in (b'\r\n', b'\n'):
                                break
                            if not line:
                                raise EOFError('Incomplete headers')
                            key, value = line.decode().split(':', 1)
                            headers[key.lower()] = value.strip()
                        length = int(headers['content-length'])
                        if not 0 < length < 2000000:
                            raise ValueError('Invalid frame length')
                        data = bytearray()
                        while len(data) < length:
                            part = stream.read(length-len(data))
                            if not part:
                                raise EOFError('Incomplete frame')
                            data.extend(part)
                        frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
                        if frame is None:
                            raise ValueError('Invalid JPEG')
                        item = {'timestamp': float(headers['x-frame-timestamp']),
                                'sequence': int(headers['x-frame-sequence']),
                                'frame': frame}
                        with self.lock:
                            self.latest, self.error = item, None
            except Exception as error:
                with self.lock:
                    self.error = str(error)
                time.sleep(.5)

    def get(self):
        with self.lock:
            return self.latest, self.error


class Flow:
    def __init__(self):
        self.previous, self.points = None, None

    def update(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        result = {'tracked': 0, 'median_flow_px': None}
        if self.previous is not None and self.points is not None and len(self.points) >= 4:
            nxt, ok, _ = cv2.calcOpticalFlowPyrLK(self.previous, gray, self.points, None)
            if nxt is not None:
                back, back_ok, _ = cv2.calcOpticalFlowPyrLK(gray, self.previous, nxt, None)
                if back is not None:
                    good = (ok.ravel() == 1) & (back_ok.ravel() == 1)
                    good &= np.linalg.norm(back-self.points, axis=2).ravel() < 1.0
                    delta = (nxt-self.points)[good].reshape(-1, 2)
                    result['tracked'] = len(delta)
                    if len(delta):
                        result['median_flow_px'] = np.median(delta, axis=0).tolist()
                    self.points = nxt[good].reshape(-1, 1, 2)
        if self.points is None or len(self.points) < 30:
            self.points = cv2.goodFeaturesToTrack(gray, 100, .03, 8)
        self.previous = gray
        return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera-url', default='http://127.0.0.1:8765')
    parser.add_argument('--telemetry-file', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=300,
                        help='Bounded recording duration; default five minutes')
    args = parser.parse_args()
    if not 0 < args.seconds <= 1800:
        parser.error('--seconds must be between zero and 1800')
    args.output.mkdir(parents=True, mode=0o700)
    feeds = {name: Feed(args.camera_url+'/'+name) for name in ('external', 'wrist')}
    flows = {name: Flow() for name in feeds}
    writers, seen = {}, {}
    log = (args.output/'observations.jsonl').open('a')
    stopped = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stopped.set())
    try:
        deadline = time.monotonic()+args.seconds
        while not stopped.is_set() and time.monotonic() < deadline:
            started = time.monotonic()
            now = time.time()
            row = {'timestamp': now, 'cameras': {}}
            for name, feed in feeds.items():
                item, error = feed.get()
                status = {'error': error, 'fresh': False}
                if item is not None:
                    status.update(timestamp=item['timestamp'], sequence=item['sequence'],
                                  age_s=time.time()-item['timestamp'])
                    status['fresh'] = error is None and 0 <= status['age_s'] < .5
                    if status['fresh'] and item['sequence'] != seen.get(name):
                        frame = item['frame']
                        if name not in writers:
                            h, w = frame.shape[:2]
                            writers[name] = cv2.VideoWriter(str(args.output/(name+'.avi')),
                                                           cv2.VideoWriter_fourcc(*'MJPG'), 10, (w, h))
                            if not writers[name].isOpened():
                                raise RuntimeError('Cannot record '+name)
                        writers[name].write(frame)
                        status.update(flows[name].update(frame))
                        seen[name] = item['sequence']
                        cv2.imwrite(str(args.output/(name+'-latest.jpg')), frame)
                row['cameras'][name] = status
            try:
                telemetry = json.loads(args.telemetry_file.read_text())
                age = time.time()-telemetry.get('timestamp', 0)
                row['feedback'] = telemetry
                row['feedback_fresh'] = 0 <= age < .4
                row['joint_camera_skew_s'] = max(abs(telemetry.get('timestamp', 0)-c.get('timestamp', 0))
                                                  for c in row['cameras'].values())
            except (OSError, ValueError):
                row['feedback_fresh'] = False
            cams = list(row['cameras'].values())
            row['camera_skew_s'] = abs(cams[0].get('timestamp', 0)-cams[1].get('timestamp', 0))
            row['paired_fresh'] = (all(c['fresh'] for c in cams) and row['feedback_fresh']
                                   and row['camera_skew_s'] < .15
                                   and row.get('joint_camera_skew_s', float('inf')) < .4)
            # Atomic latest status; the recording index supplies actual timing.
            temporary = args.output/'status.tmp'
            temporary.write_text(json.dumps(row))
            temporary.replace(args.output/'status.json')
            log.write(json.dumps(row)+'\n');log.flush()
            time.sleep(max(0, .1-(time.monotonic()-started)))
    finally:
        log.close()
        for writer in writers.values():
            writer.release()


if __name__ == '__main__':
    main()
