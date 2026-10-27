"""Persistent native GTK3 RTMPose viewer; consumes the producer's annotated JPEGs.

One bounded latest-frame slot. No V4L2, inference, motor, recording, or HTTP writes.
Requires X-Frame-Fresh / X-Capture-Ns / X-Frame-Sequence multipart headers.
Capture timestamps are CLOCK_MONOTONIC on this same Pi, not a remote PC clock.
"""
import argparse
from dataclasses import dataclass
import json
import math
import threading
import time
from urllib.parse import urlsplit
from urllib.request import urlopen

MAX_JPEG = 4 * 1024 * 1024


@dataclass(frozen=True)
class Frame:
    jpeg: bytes
    capture_ns: int
    sequence: int
    received_s: float


def read_exact(stream, size):
    data = bytearray()
    while len(data) < size:
        chunk = stream.read(size - len(data))
        if not chunk:
            raise EOFError('JPEG body ended early')
        data.extend(chunk)
    return bytes(data)


def read_frame(stream):
    # Multipart boundaries may be preceded by one empty separator line.
    for _ in range(4):
        line = stream.readline(512)
        if not line:
            raise EOFError('MJPEG stream ended')
        if line.startswith(b'--') and line.endswith(b'\n'):
            break
        if line.strip():
            raise ValueError('invalid multipart boundary')
    else:
        raise ValueError('missing multipart boundary')
    headers, used = {}, 0
    while True:
        line = stream.readline(1024)
        used += len(line)
        if not line or used > 8192 or not line.endswith(b'\n'):
            raise ValueError('missing/oversized multipart headers')
        if not line.strip():
            break
        key, separator, value = line.partition(b':')
        if not separator:
            raise ValueError('invalid multipart header')
        key = key.strip().lower()
        if key in headers:
            raise ValueError('duplicate multipart header')
        headers[key] = value.strip()
    size = int(headers.get(b'content-length', b'0'))
    if not 0 < size <= MAX_JPEG or headers.get(b'content-type') != b'image/jpeg':
        raise ValueError('invalid JPEG length/type')
    fresh = headers.get(b'x-frame-fresh')
    if fresh not in (b'0', b'1'):
        raise ValueError('producer freshness header required')
    capture_ns = int(headers[b'x-capture-ns'])
    sequence = int(headers[b'x-frame-sequence'])
    if sequence < 0 or capture_ns < 0 or (fresh == b'1' and capture_ns == 0):
        raise ValueError('invalid producer frame identity')
    jpeg = read_exact(stream, size)
    if not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9'):
        raise ValueError('invalid JPEG markers')
    return fresh == b'1', capture_ns, sequence, jpeg


class Latest:
    def __init__(self, clock=time.monotonic, ttl=.75):
        self.clock, self.ttl, self.lock = clock, ttl, threading.Lock()
        self.frame, self.available, self.reason = None, False, 'connecting'
        self.status, self.status_time = {}, 0.

    def accept(self, fresh, capture_ns, sequence, jpeg):
        now = self.clock()
        age = now - capture_ns / 1e9
        with self.lock:
            if not fresh:
                self.available, self.reason = False, 'producer has no fresh frame'
            elif not 0 <= age < self.ttl:
                self.available, self.reason = False, 'old/future capture timestamp'
            else:
                self.frame = Frame(jpeg, capture_ns, sequence, now)
                self.available, self.reason = True, 'live image'

    def unavailable(self, reason):
        with self.lock:
            self.available, self.reason = False, str(reason)[:180]

    def set_status(self, status):
        with self.lock:
            self.status, self.status_time = status, self.clock()

    def snapshot(self):
        now = self.clock()
        with self.lock:
            age = None if self.frame is None else now - self.frame.capture_ns / 1e9
            fresh = self.available and age is not None and 0 <= age < self.ttl
            reason = self.reason if fresh or not self.available else 'frame timed out'
            status = dict(self.status) if 0 <= now-self.status_time < 1. else {}
            return self.frame, fresh, age, reason, status


def frames_loop(base, latest, stop, opener=urlopen):
    while not stop.is_set():
        try:
            with opener(base+'/stream.mjpg', timeout=.5) as response:
                while not stop.is_set():
                    latest.accept(*read_frame(response))
        except Exception as exc:
            latest.unavailable(f'{type(exc).__name__}: {exc}')
        stop.wait(.5)


def status_loop(base, latest, stop, opener=urlopen):
    while not stop.is_set():
        try:
            with opener(base+'/status.json', timeout=.3) as response:
                body = response.read(65537)
                if len(body) > 65536:
                    raise ValueError('status too large')
                status = json.loads(body)
                if isinstance(status, dict):
                    latest.set_status(status)
        except Exception:
            pass  # Image freshness is independently timestamped and authoritative.
        stop.wait(.25)


def native_window(base, *, opener=urlopen):
    import gi
    gi.require_version('Gtk', '3.0')
    from gi.repository import Gtk, Gdk, GdkPixbuf, GLib
    latest, stop = Latest(), threading.Event()
    window = Gtk.Window(title='RTMPose 실시간 자세 · 시트 카메라')
    window.set_default_size(960, 760)
    box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
    box.set_border_width(12)
    title, footer, area = Gtk.Label(), Gtk.Label(), Gtk.DrawingArea()
    title.set_xalign(0)
    footer.set_xalign(0)
    footer.set_line_wrap(True)
    box.pack_start(title, False, False, 0)
    box.pack_start(area, True, True, 0)
    box.pack_start(footer, False, False, 0)
    window.add(box)
    ui = dict(pixbuf=None, identity=None, live=False)

    def draw(widget, context):
        width, height = widget.get_allocated_width(), widget.get_allocated_height()
        context.set_source_rgb(.035, .055, .075)
        context.paint()
        pixbuf = ui['pixbuf']
        if pixbuf is not None:
            scale = min(width/pixbuf.get_width(), height/pixbuf.get_height())
            w, h = max(1,int(pixbuf.get_width()*scale)), max(1,int(pixbuf.get_height()*scale))
            scaled = pixbuf.scale_simple(w,h,GdkPixbuf.InterpType.BILINEAR)
            Gdk.cairo_set_source_pixbuf(context,scaled,(width-w)/2,(height-h)/2)
            context.paint()  # Preserve the last image's original brightness while waiting.
        return False

    def update():
        frame, live, age, reason, status = latest.snapshot()
        identity = None if frame is None else (frame.capture_ns,frame.sequence)
        if live and frame is not None and identity != ui['identity']:
            try:
                loader = GdkPixbuf.PixbufLoader.new_with_type('jpeg')
                loader.write(frame.jpeg)
                loader.close()
                pixbuf = loader.get_pixbuf()
                if pixbuf is None:
                    raise ValueError('empty JPEG decode')
                ui['pixbuf'], ui['identity'] = pixbuf, identity
            except Exception as exc:
                latest.unavailable('JPEG decode failed: '+str(exc))
                live, reason = False, 'JPEG decode failed'
        ui['live'] = live
        title.set_text('RTMPose · '+('LIVE 영상' if live else 'WAITING · 새 영상 대기'))
        pose = status.get('reason','status unavailable')
        def numeric(value,unit):
            return f'{value:.2f}{unit}' if type(value) in (int,float) and math.isfinite(value) else '—'
        age_text = '없음' if age is None else f'{max(0.,age):.2f} s'
        footer.set_text(f'영상 경과: {age_text} · {reason}\n'
            f'자세 상태: {pose} · 관측 FPS: {numeric(status.get("inference_fps"),"")} · '
            f'신뢰도: {numeric(status.get("last_body_confidence"),"")}\n'
            '공유 RTMPose 주석 영상 · 카메라 추가 점유 없음 · Q / Esc: 이 창만 닫기')
        area.queue_draw()
        return not stop.is_set()

    def close(*_):
        stop.set()
        Gtk.main_quit()
    def key(_window,event):
        if event.keyval in (Gdk.KEY_q,Gdk.KEY_Q,Gdk.KEY_Escape):
            window.destroy()
            return True
        return False
    area.connect('draw',draw)
    window.connect('destroy',close)
    window.connect('key-press-event',key)
    window.show_all()
    update()
    for target in (frames_loop,status_loop):
        threading.Thread(target=target,args=(base,latest,stop,opener),daemon=True).start()
    GLib.timeout_add(66,update)
    Gtk.main()
    stop.set()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url',default='http://127.0.0.1:8081')
    args = parser.parse_args(argv)
    parsed = urlsplit(args.url)
    if parsed.scheme!='http' or parsed.hostname not in ('127.0.0.1','localhost') or parsed.username or parsed.password or parsed.path not in ('','/') or parsed.query or parsed.fragment:
        parser.error('Only the same-Pi localhost HTTP producer is supported')
    native_window(args.url.rstrip('/'))


if __name__=='__main__':
    main()
