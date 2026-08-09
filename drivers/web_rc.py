"""
Web RC — phone/tablet remote control over a private LAN
=======================================================
Same control scheme as the ELRS/CRSF path, but with no RC hardware: a small
HTTP server serves a touch UI, the browser posts stick positions, and this
class reports them as CRSF channel values.

Drop-in replacement for CRSFReader — same start() / stop() / get_channels()
interface and the same channel layout, so RCMapper is reused unchanged:
    CH3 (idx 2) throttle   0.0 to 1.0  → cmd_vel
    CH2 (idx 1) pitch      -1.0 to 1.0 → cmd_pitch
    CH4 (idx 3) yaw        -1.0 to 1.0 → cmd_yaw
    CH8 (idx 7) arm switch                 low = disarmed, high = armed

Failsafe: if no command arrives within `timeout` seconds (phone sleeps, WiFi
drops, tab closed), channels snap back to throttle-min / sticks-centered /
disarmed and connected goes False — same behaviour as CRSF link loss.

SECURITY: the server is unauthenticated by default. Bind it to a trusted
private LAN only, and pass token=... to require a shared secret.

Standalone test:
    python3 drivers/web_rc.py --port 8080
"""

import json
import math
import os
import socket
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

CRSF_MIN = 172
CRSF_MID = 992
CRSF_MAX = 1811

PAGE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         'web_rc_page.html')


def _clamp(val, lo, hi):
    return max(lo, min(hi, val))


def lan_ip():
    """Best-effort primary LAN address. Opens a UDP socket, sends nothing."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        return s.getsockname()[0]
    except OSError:
        return '127.0.0.1'
    finally:
        s.close()


class WebRCReader:
    """Threaded HTTP RC receiver — CRSFReader-compatible."""

    def __init__(self, host='0.0.0.0', port=8080, timeout=0.5,
                 deadband=30, token=None):
        """
        Args:
            host:     bind address ('0.0.0.0' = reachable from the LAN)
            port:     TCP port for the control page
            timeout:  seconds without a command before failsafe (link loss)
            deadband: RCMapper's deadband, in CRSF counts. Stick input is
                      pre-expanded past it so the on-screen throw stays
                      linear end to end. Keep in sync with RCMapper.
            token:    shared secret required on control requests, or None
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.deadband = deadband
        self.token = token

        self.channels = self._failsafe_channels()
        self.connected = False
        self.last_update = 0.0
        self.frame_count = 0

        self._lock = threading.Lock()
        self._httpd = None
        self._thread = None

    # ── CRSF encoding ─────────────────────────────────────────
    @staticmethod
    def _failsafe_channels():
        ch = [CRSF_MID] * 16
        ch[2] = CRSF_MIN   # throttle low
        ch[7] = CRSF_MIN   # disarmed
        return ch

    def _encode_centered(self, val):
        """Normalized -1..1 → CRSF counts, expanded past RCMapper's deadband."""
        val = _clamp(val, -1.0, 1.0)
        if val == 0.0:
            return CRSF_MID
        span = (CRSF_MAX - CRSF_MID) if val > 0 else (CRSF_MID - CRSF_MIN)
        mag = self.deadband + abs(val) * (span - self.deadband)
        return int(round(CRSF_MID + math.copysign(mag, val)))

    def _encode(self, throttle, pitch, yaw, armed):
        ch = [CRSF_MID] * 16
        t = _clamp(throttle, 0.0, 1.0)
        ch[2] = int(round(CRSF_MIN + t * (CRSF_MAX - CRSF_MIN)))
        ch[1] = self._encode_centered(pitch)
        ch[3] = self._encode_centered(yaw)
        ch[7] = CRSF_MAX if armed else CRSF_MIN
        return ch

    # ── Public API (matches CRSFReader) ───────────────────────
    def update(self, throttle=0.0, pitch=0.0, yaw=0.0, armed=False):
        """Feed one control frame. Called by the HTTP handler."""
        ch = self._encode(throttle, pitch, yaw, armed)
        with self._lock:
            self.channels = ch
            self.connected = True
            self.last_update = time.monotonic()
            self.frame_count += 1

    def get_channels(self):
        with self._lock:
            if self.connected and \
                    time.monotonic() - self.last_update > self.timeout:
                self.connected = False
                self.channels = self._failsafe_channels()
            return list(self.channels), self.connected

    @property
    def url(self):
        host = lan_ip() if self.host in ('0.0.0.0', '') else self.host
        suffix = f'?token={self.token}' if self.token else ''
        return f'http://{host}:{self.port}/{suffix}'

    def start(self):
        """Bind and serve. Raises OSError if the port is already taken."""
        self._httpd = ThreadingHTTPServer((self.host, self.port),
                                          _make_handler(self))
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd.server_close()
            self._httpd = None

    def _run(self):
        _relax_scheduling()
        self._httpd.serve_forever(poll_interval=0.2)


def _relax_scheduling():
    """Drop RT priority / core pinning inherited from the control thread.

    The control loop may already be SCHED_FIFO pinned to one core. HTTP
    threads spawned after that would inherit it and add jitter to the
    balance loop, so put this thread (and its children) back to normal.
    """
    try:
        os.sched_setscheduler(0, os.SCHED_OTHER, os.sched_param(0))
    except (AttributeError, OSError):
        pass
    try:
        os.sched_setaffinity(0, set(range(os.cpu_count() or 1)))
    except (AttributeError, OSError):
        pass


def _make_handler(reader):
    class WebRCHandler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def log_message(self, fmt, *args):
            pass   # keep the control-loop console clean

        # ── helpers ───────────────────────────────────────────
        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _send_json(self, obj, code=200):
            self._send(code, json.dumps(obj).encode(),
                       'application/json; charset=utf-8')

        def _authorized(self):
            if not reader.token:
                return True
            if self.headers.get('X-RC-Token') == reader.token:
                return True
            return f'token={reader.token}' in (self.path.split('?', 1) + [''])[1]

        # ── routes ────────────────────────────────────────────
        def do_GET(self):
            route = self.path.split('?', 1)[0]
            if route in ('/', '/index.html'):
                try:
                    with open(PAGE_FILE, 'rb') as f:
                        page = f.read()
                except OSError as e:
                    self._send_json({'error': f'page missing: {e}'}, 500)
                    return
                self._send(200, page, 'text/html; charset=utf-8')
            elif route == '/status':
                if not self._authorized():
                    self._send_json({'error': 'forbidden'}, 403)
                    return
                channels, connected = reader.get_channels()
                self._send_json({'connected': connected,
                                 'channels': channels,
                                 'frames': reader.frame_count})
            else:
                self._send_json({'error': 'not found'}, 404)

        def do_POST(self):
            if self.path.split('?', 1)[0] != '/rc':
                self._send_json({'error': 'not found'}, 404)
                return
            if not self._authorized():
                self._send_json({'error': 'forbidden'}, 403)
                return

            length = int(self.headers.get('Content-Length') or 0)
            if length <= 0 or length > 4096:
                self._send_json({'error': 'bad length'}, 400)
                return
            try:
                cmd = json.loads(self.rfile.read(length))
                reader.update(throttle=float(cmd.get('throttle', 0.0)),
                              pitch=float(cmd.get('pitch', 0.0)),
                              yaw=float(cmd.get('yaw', 0.0)),
                              armed=bool(cmd.get('armed', False)))
            except (ValueError, TypeError) as e:
                self._send_json({'error': f'bad command: {e}'}, 400)
                return

            channels, connected = reader.get_channels()
            self._send_json({'connected': connected,
                             'armed': channels[7] > 600,
                             'ch': [channels[2], channels[1],
                                    channels[3], channels[7]]})

    return WebRCHandler


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Web RC standalone test')
    parser.add_argument('--host',  type=str, default='0.0.0.0')
    parser.add_argument('--port',  type=int, default=8080)
    parser.add_argument('--token', type=str, default=None)
    args = parser.parse_args()

    rc = WebRCReader(host=args.host, port=args.port, token=args.token)
    rc.start()
    print(f'[WebRC] Serving on {rc.url}')
    if not args.token:
        print('[WebRC] No token set — anyone on this LAN can drive the robot.')
    print('[WebRC] Open that URL on your phone. Ctrl-C to stop.\n')

    try:
        while True:
            channels, connected = rc.get_channels()
            link = 'linked ' if connected else 'NO LINK'
            print(f'\r  {link}  thr={channels[2]:4d}  pitch={channels[1]:4d}  '
                  f'yaw={channels[3]:4d}  arm={channels[7]:4d}  '
                  f'frames={rc.frame_count:6d}', end='', flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('\n[WebRC] Stopping.')
    finally:
        rc.stop()

