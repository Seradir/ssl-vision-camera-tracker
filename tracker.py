from __future__ import annotations

import argparse, fractions, logging, signal, socket, sys, threading, time
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

import cv2
import numpy as np
from google.protobuf import descriptor_pb2, descriptor_pool, message_factory
from google.protobuf.message import DecodeError


def _build_proto_classes():
    fd = descriptor_pb2.FileDescriptorProto()
    fd.name = "ssl_tracked_minimal.proto"
    fd.package = "ssl_tracked_minimal"
    fd.syntax = "proto2"
    L = descriptor_pb2.FieldDescriptorProto

    def field(msg, name, no, label, typ, type_name=None):
        f = msg.field.add(); f.name = name; f.number = no; f.label = label; f.type = typ
        if type_name: f.type_name = type_name

    m = fd.message_type.add(); m.name = "Vector3"
    field(m, "x", 1, L.LABEL_REQUIRED, L.TYPE_FLOAT)
    field(m, "y", 2, L.LABEL_REQUIRED, L.TYPE_FLOAT)
    field(m, "z", 3, L.LABEL_REQUIRED, L.TYPE_FLOAT)

    m = fd.message_type.add(); m.name = "TrackedBall"
    field(m, "pos", 1, L.LABEL_REQUIRED, L.TYPE_MESSAGE, ".ssl_tracked_minimal.Vector3")
    field(m, "visibility", 3, L.LABEL_OPTIONAL, L.TYPE_FLOAT)

    m = fd.message_type.add(); m.name = "TrackedFrame"
    field(m, "frame_number", 1, L.LABEL_REQUIRED, L.TYPE_UINT32)
    field(m, "timestamp", 2, L.LABEL_REQUIRED, L.TYPE_DOUBLE)
    field(m, "balls", 3, L.LABEL_REPEATED, L.TYPE_MESSAGE, ".ssl_tracked_minimal.TrackedBall")

    m = fd.message_type.add(); m.name = "TrackerWrapperPacket"
    field(m, "uuid", 1, L.LABEL_REQUIRED, L.TYPE_STRING)
    field(m, "source_name", 2, L.LABEL_OPTIONAL, L.TYPE_STRING)
    field(m, "tracked_frame", 3, L.LABEL_OPTIONAL, L.TYPE_MESSAGE, ".ssl_tracked_minimal.TrackedFrame")

    pool = descriptor_pool.DescriptorPool(); pool.Add(fd)
    return {
        name: message_factory.GetMessageClass(pool.FindMessageTypeByName(f"ssl_tracked_minimal.{name}"))
        for name in ("Vector3", "TrackedBall", "TrackedFrame", "TrackerWrapperPacket")
    }


_PROTO = _build_proto_classes()
TrackedFrameMsg = _PROTO["TrackedFrame"]
TrackerWrapperPacketMsg = _PROTO["TrackerWrapperPacket"]


@dataclass
class BallState:
    world_x: float
    world_y: float
    source_timestamp: float
    received_monotonic: float
    frame_number: int


class TrackedPacketParser:
    def __init__(self):
        self.frame_cls = TrackedFrameMsg
        self.wrapper_cls = TrackerWrapperPacketMsg

    def _as_frame(self, payload: bytes):
        msg = self.frame_cls(); msg.ParseFromString(payload)
        return msg if msg.IsInitialized() else None

    def _as_wrapper(self, payload: bytes):
        msg = self.wrapper_cls(); msg.ParseFromString(payload)
        return msg if msg.IsInitialized() and msg.HasField("tracked_frame") and msg.tracked_frame.IsInitialized() else None

    def extract_primary_ball(self, payload: bytes) -> Optional[BallState]:
        frame = None
        try: frame = self._as_frame(payload)
        except DecodeError: pass
        if frame is None:
            try:
                wrapper = self._as_wrapper(payload)
                frame = wrapper.tracked_frame if wrapper else None
            except DecodeError:
                frame = None
        if frame is None or not frame.balls:
            return None
        ball = frame.balls[0]
        return BallState(float(ball.pos.x), float(ball.pos.y), float(frame.timestamp), time.monotonic(), int(frame.frame_number))


class LatestBallReceiver(threading.Thread):
    def __init__(self, host: str, port: int, multicast_group: str | None = None, multicast_interface: str = "0.0.0.0"):
        super().__init__(daemon=True)
        self.host, self.port = host, port
        self.multicast_group, self.multicast_interface = multicast_group, multicast_interface
        self.parser = TrackedPacketParser()
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.sock: socket.socket | None = None
        self._latest: BallState | None = None

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock = sock
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.host, self.port))
        if self.multicast_group:
            mreq = socket.inet_aton(self.multicast_group) + socket.inet_aton(self.multicast_interface)
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.settimeout(0.2)
        logging.info("Protobuf-UDP aktiv auf %s:%d%s", self.host, self.port, f" (Multicast {self.multicast_group})" if self.multicast_group else "")
        while not self.stop_event.is_set():
            try: payload, _ = sock.recvfrom(65535)
            except socket.timeout: continue
            except OSError: break
            try: ball = self.parser.extract_primary_ball(payload)
            except DecodeError:
                ball = None
            if ball is not None:
                with self.lock: self._latest = ball

    def latest(self) -> BallState | None:
        with self.lock: return self._latest

    def stop(self):
        self.stop_event.set()
        if self.sock is not None:
            try: self.sock.close()
            except OSError: pass


class AffineMapper:
    def __init__(self, matrix: np.ndarray):
        if matrix.shape != (2, 3): raise ValueError("Affine matrix must be 2x3")
        self.matrix = matrix.astype(np.float64)

    @classmethod
    def from_pairs(cls, pairs: Sequence[Tuple[float, float, float, float]]):
        A, bu, bv = [], [], []
        for vx, vy, bx, by in pairs:
            A.append([bx, by, 1.0]); bu.append(vx); bv.append(vy)
        A = np.asarray(A, dtype=np.float64)
        u, *_ = np.linalg.lstsq(A, np.asarray(bu, dtype=np.float64), rcond=None)
        v, *_ = np.linalg.lstsq(A, np.asarray(bv, dtype=np.float64), rcond=None)
        return cls(np.vstack([u, v]))

    def map_world_to_video(self, x: float, y: float) -> Tuple[float, float]:
        out = self.matrix @ np.array([x, y, 1.0], dtype=np.float64)
        return float(out[0]), float(out[1])


class EMAFilter:
    def __init__(self, alpha: float):
        if not (0.0 < alpha <= 1.0): raise ValueError("alpha must be in (0,1]")
        self.alpha = alpha; self.value: Tuple[float, float] | None = None

    def update(self, x: float, y: float) -> Tuple[float, float]:
        if self.value is None:
            self.value = (x, y)
        else:
            px, py = self.value
            self.value = (self.alpha * x + (1 - self.alpha) * px, self.alpha * y + (1 - self.alpha) * py)
        return self.value


class BallZoomRenderer:
    def __init__(self, zoom_factor: float, output_size: Tuple[int, int]):
        if zoom_factor < 1.0: raise ValueError("zoom_factor must be >= 1.0")
        self.zoom_factor, self.output_size = float(zoom_factor), output_size

    def render(self, frame: np.ndarray, center_x: float, center_y: float) -> np.ndarray:
        h, w = frame.shape[:2]
        out_w, out_h = self.output_size
        crop_w, crop_h = max(2, round(w / self.zoom_factor)), max(2, round(h / self.zoom_factor))
        crop_w, crop_h = min(crop_w, w), min(crop_h, h)
        x0 = int(round(center_x - crop_w / 2)); y0 = int(round(center_y - crop_h / 2))
        x0 = max(0, min(x0, w - crop_w)); y0 = max(0, min(y0, h - crop_h))
        crop = frame[y0:y0 + crop_h, x0:x0 + crop_w]
        if crop.size == 0: raise RuntimeError("Empty crop generated")
        return cv2.resize(crop, (out_w, out_h), interpolation=cv2.INTER_LINEAR)


class StopFlag:
    def __init__(self): self.stop = False
    def install(self):
        def handler(_sig, _frame): self.stop = True
        signal.signal(signal.SIGINT, handler); signal.signal(signal.SIGTERM, handler)


def opencv_has_gstreamer() -> bool:
    for line in cv2.getBuildInformation().splitlines():
        if "GStreamer" in line: return "YES" in line.upper()
    return False


def fps_fraction(fps: float) -> Tuple[int, int]:
    frac = fractions.Fraction(fps).limit_denominator(1001)
    return frac.numerator, frac.denominator


def build_capture_pipeline(device_number: int, connection: str, mode: str | None) -> str:
    parts = [f"decklinkvideosrc device-number={device_number}", f"connection={connection}"]
    if mode: parts.append(f"mode={mode}")
    parts += ["! queue leaky=downstream max-size-buffers=2", "! videoconvert", "! video/x-raw,format=BGR", "! appsink drop=true max-buffers=1 sync=false"]
    return " ".join(parts)


def build_output_pipeline(device: str, width: int, height: int, fps: float, output_format: str) -> str:
    num, den = fps_fraction(fps)
    return (
        "appsrc is-live=true format=time do-timestamp=true "
        f"! video/x-raw,format=BGR,width={width},height={height},framerate={num}/{den} "
        "! queue leaky=downstream max-size-buffers=2 ! videoconvert "
        f"! video/x-raw,format={output_format} ! v4l2sink device={device} sync=false"
    )


def parse_args(argv: Sequence[str] | None = None):
    p = argparse.ArgumentParser(description="Ball-Zoom-Stream aus DeckLink-SDI und SSL-Tracking-Protobuf")
    p.add_argument("--pb-host", default="0.0.0.0")
    p.add_argument("--pb-port", type=int, required=True)
    p.add_argument("--pb-multicast-group", default=None)
    p.add_argument("--pb-multicast-interface", default="0.0.0.0")
    p.add_argument("--decklink-device", type=int, default=0)
    p.add_argument("--decklink-connection", default="sdi")
    p.add_argument("--decklink-mode", default=None)
    p.add_argument("--input-pipeline", default=None)
    p.add_argument("--output-device", required=True)
    p.add_argument("--output-format", default="YUY2")
    p.add_argument("--output-width", type=int, default=None)
    p.add_argument("--output-height", type=int, default=None)
    p.add_argument("--output-fps", type=float, default=None)
    p.add_argument("--fallback-fps", type=float, default=25.0)
    p.add_argument("--zoom-factor", type=float, default=2.0)
    p.add_argument("--ball-timeout-ms", type=int, default=500)
    p.add_argument("--smooth-alpha", type=float, default=1.0)
    p.add_argument("--map", dest="maps", nargs=4, action="append", metavar=("VIDEO_X", "VIDEO_Y", "BALL_X", "BALL_Y"), required=True, help="Genau 4x: video_x video_y ball_x ball_y")
    p.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = p.parse_args(argv)
    if len(args.maps) != 4: p.error("Es müssen genau 4x --map angegeben werden.")
    args.maps = [tuple(float(v) for v in m) for m in args.maps]
    return args


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=getattr(logging, args.log_level), format="%(asctime)s %(levelname)s %(message)s")
    if not opencv_has_gstreamer():
        raise RuntimeError("Dieses OpenCV wurde ohne GStreamer gebaut.")

    mapper = AffineMapper.from_pairs(args.maps)
    receiver = LatestBallReceiver(args.pb_host, args.pb_port, args.pb_multicast_group, args.pb_multicast_interface)
    receiver.start()

    capture_pipeline = args.input_pipeline or build_capture_pipeline(args.decklink_device, args.decklink_connection, args.decklink_mode)
    cap = cv2.VideoCapture(capture_pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened(): raise RuntimeError(f"VideoCapture konnte nicht geöffnet werden: {capture_pipeline}")
    logging.info("Capture-Pipeline: %s", capture_pipeline)

    ok, frame = cap.read()
    if not ok or frame is None: raise RuntimeError("Konnte kein erstes Frame lesen.")
    in_h, in_w = frame.shape[:2]
    out_w, out_h = args.output_width or in_w, args.output_height or in_h
    fps = float(cap.get(cv2.CAP_PROP_FPS)); fps = fps if np.isfinite(fps) and fps > 0 else args.output_fps or args.fallback_fps
    if args.output_fps is not None: fps = args.output_fps

    output_pipeline = build_output_pipeline(args.output_device, out_w, out_h, fps, args.output_format)
    writer = cv2.VideoWriter(output_pipeline, cv2.CAP_GSTREAMER, 0, fps, (out_w, out_h), True)
    if not writer.isOpened(): raise RuntimeError(f"VideoWriter konnte nicht geöffnet werden: {output_pipeline}")
    logging.info("Output-Pipeline: %s", output_pipeline)

    renderer = BallZoomRenderer(args.zoom_factor, (out_w, out_h))
    smoother = EMAFilter(args.smooth_alpha)
    timeout_s = args.ball-timeout-ms / 1000.0
    stop = StopFlag(); stop.install()
    last_log = 0.0

    try:
        while not stop.stop:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.005); continue
            latest = receiver.latest()
            if latest and (time.monotonic() - latest.received_monotonic) <= timeout_s:
                cx, cy = mapper.map_world_to_video(latest.world_x, latest.world_y)
            else:
                cx, cy = frame.shape[1] / 2.0, frame.shape[0] / 2.0
            cx, cy = smoother.update(cx, cy)
            writer.write(renderer.render(frame, cx, cy))
            now = time.monotonic()
            if now - last_log > 1.0:
                logging.info("Ball-Zentrum bei x=%.1f y=%.1f | Output %dx%d @ %.3f fps", cx, cy, out_w, out_h, fps)
                last_log = now
    finally:
        receiver.stop(); receiver.join(timeout=1.0)
        cap.release(); writer.release()
    return 0


if __name__ == "__main__":
    sys.exit(main())
