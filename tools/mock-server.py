#!/usr/bin/env python3
"""Server mock meniru API ESP32 — untuk mengembangkan dashboard tanpa alat.

Pakai:  python3 tools/mock-server.py   lalu buka http://127.0.0.1:8791
Menyajikan file dari SistemKontrolKelembapanBudidayaJamur/data/ + /api/data palsu.
"""
import json, math, random, http.server, os

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                    '..', 'SistemKontrolKelembapanBudidayaJamur', 'data')
os.chdir(ROOT)

def data():
    hist_h, hist_t = [], []
    for i in range(90):
        t = i * 2
        hist_h.append(round(76 + 4.5 * math.sin(t / 38) + random.uniform(-0.5, 0.5), 1))
        hist_t.append(round(27.2 + 0.5 * math.sin(t / 55) + random.uniform(-0.12, 0.12), 1))
    rh, temp = hist_h[-1], hist_t[-1]
    fc = [round(min(100, rh + 0.28 * s / 10 + 0.4 * math.sin(s / 80)), 1) for s in range(10, 301, 10)]
    return {
        "t": temp, "rh": rh, "ok": True,
        "mist": {"mode": "auto", "pct": 62.0, "man": 40.0},
        "fan": {"mode": "auto", "on": True, "man": False},
        "valve": {"mode": "auto", "on": False, "man": False},
        "waterFull": True, "fillAlarm": False, "safety": False,
        "ai": "AI: prediksi 78.4% (target 80%) - kabut 62%",
        "histStep": 2, "histT": hist_t, "histH": hist_h,
        "fcStep": 10, "fc": fc,
        "w": {"ok": True, "t": 24.5, "rh": 78.0, "code": 2, "desc": "Berawan Sebagian", "ageS": 84,
              "lok": 0, "nama": "Sukosewu, Bojonegoro"},
        "sys": {"ip": "192.168.143.247", "rssi": -58, "heap": 191234, "upS": 5460, "time": 0},
    }

class H(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path.startswith('/api/data'):
            body = json.dumps(data()).encode()
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', len(body))
            self.end_headers()
            self.wfile.write(body)
        else:
            super().do_GET()
    def do_POST(self):
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.end_headers()
        self.wfile.write(b'{"ok":true}')
    def log_message(self, *a): pass

if __name__ == '__main__':
    print('Mock ESP32 di http://127.0.0.1:8791 (Ctrl+C utk berhenti)')
    http.server.ThreadingHTTPServer(('127.0.0.1', 8791), H).serve_forever()
