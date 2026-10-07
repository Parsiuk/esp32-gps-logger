from machine import Pin, I2C, UART
import ssd1306, time, os, network, gc, math, bluetooth

# Collect garbage early and often. When an allocation doesn't fit, the ESP32
# port grows the Python heap with the largest free IDF block (~60 kB) instead,
# and TLS (mbedTLS lives in the IDF heap) then fails with ENOMEM.
gc.threshold(8192)

BAUD = 9600                              # NEO-6M/7M/8M default

i2c = I2C(0, scl=Pin(22), sda=Pin(21))
oled = ssd1306.SSD1306_I2C(128, 64, i2c)

uart = UART(2, baudrate=BAUD, tx=17, rx=16, rxbuf=4096)   # rides out HTTP calls

REFRESH = 500                            # ms between screen redraws
NO_DATA = 3000                           # ms of silence = "No data!"

SAMPLE = 1000                            # ms between sampled positions
MIN_MOVE = 10                            # m from the last recorded point
FLUSH = 60000                            # ms between batch writes to flash
NEW_FILE_GAP = 300                       # s without fix before a new file
STALE = 2000                             # ms: older position = no fix
LOG_DIR = "/gpx"
MIN_FREE = 32768                         # bytes of flash kept free
MAX_BATCH = 600                          # points kept in RAM if writes fail

ENV_FILE = "/.env"                       # SSID, WPA2, GEO_URL, API_KEY,
                                         # OBDII_ADDRESS, OBDII_PIN lines
WIFI_CHECK = FLUSH                       # ms between checks while connected
WIFI_RETRY = 15000                       # ms between reconnects while down

UPLOAD_BATCH = 50                        # points per POST to Dawarich
POINT_MAX = 192                          # bytes of JSON per point, at most
UPLOAD_GAP = 5000                        # ms between successful batches
UPLOAD_RETRY = 60000                     # ms to wait after a failed batch
HTTP_TIMEOUT = 15                        # s
DEVICE_ID = "esp32"

OBD_RETRY = 10000                        # ms between connect / 0100 attempts
OBD_CONNECT = 5000                       # ms to look for the adapter
OBD_TIMEOUT = 2000                       # ms to wait for a reply
OBD_SLOW = 10000                         # ms for ATZ and 0100 (protocol search)
OBD_STALE = 3000                         # ms: older values aren't logged
OBD_IDLE = 60000                         # ms without a fix before BLE is off

# --- NMEA parsing -----------------------------------------------------------

st = {}

def reset():
    st.clear()
    st.update(start=time.ticks_ms(), rx=None, spin=0, ok=0, err=0,
              status=None, quality=None, fixtype=None, valid=False,
              time=None, date=None, lat=None, lon=None, alt=None,
              speed=None, course=None, used=None, view={},
              hdop=None, pdop=None, ttff=None, pos_at=None)

def _num(s, conv=float):
    return conv(s) if s else None

def _deg(v, hemi):
    """NMEA ddmm.mmmm / dddmm.mmmm + N/S/E/W -> signed decimal degrees."""
    if not v or not hemi:
        return None
    i = v.find(".")
    d = (i if i >= 0 else len(v)) - 2
    deg = int(v[:d]) + float(v[d:]) / 60
    return -deg if hemi in "SW" else deg

def _position(f, i):
    lat, lon = _deg(f[i], f[i + 1]), _deg(f[i + 2], f[i + 3])
    if lat is not None and lon is not None:
        st["lat"], st["lon"] = lat, lon

def _time(s):
    if len(s) >= 6:
        st["time"] = "%s:%s:%s" % (s[0:2], s[2:4], s[4:6])

def _update_fix():
    st["valid"] = st["status"] == "A" or (st["quality"] or 0) > 0
    if st["valid"]:
        st["pos_at"] = time.ticks_ms()
        if st["ttff"] is None:
            st["ttff"] = time.ticks_diff(st["pos_at"], st["start"]) // 1000

def rmc(talker, f):
    _time(f[1])
    st["status"] = f[2]
    _position(f, 3)
    kn = _num(f[7])
    st["speed"] = None if kn is None else kn * 1.852
    st["course"] = _num(f[8])
    d = f[9]
    if len(d) == 6:
        st["date"] = "20%s-%s-%s" % (d[4:6], d[2:4], d[0:2])
    _update_fix()

def gga(talker, f):
    _time(f[1])
    _position(f, 2)
    st["quality"] = _num(f[6], int)
    st["used"] = _num(f[7], int)
    st["hdop"] = _num(f[8])
    st["alt"] = _num(f[9])
    _update_fix()

def gsa(talker, f):
    st["fixtype"] = _num(f[2], int)
    if len(f) > 15:
        st["pdop"] = _num(f[15])

def gsv(talker, f):
    n = _num(f[3], int)
    if n is not None:
        st["view"][talker] = n          # per constellation, summed on display

HANDLERS = {"RMC": rmc, "GGA": gga, "GSA": gsa, "GSV": gsv}

def checksum_ok(line):
    star = line.rfind(b"*")
    if not line.startswith(b"$") or star < 0:
        return False
    c = 0
    for b in line[1:star]:
        c ^= b
    try:
        return c == int(line[star + 1:star + 3], 16)
    except ValueError:
        return False

def handle(line):
    line = line.strip()
    if not line:
        return
    if not checksum_ok(line):
        st["err"] += 1
        return
    try:
        f = line[1:line.rfind(b"*")].decode().split(",")
        fn = HANDLERS.get(f[0][2:])
        if fn:
            fn(f[0][:2], f)
        st["ok"] += 1
    except (ValueError, IndexError, UnicodeError):
        st["err"] += 1

_buf = b""
_chunk = bytearray(256)                  # read in small pieces: one big
                                         # allocation grows the heap (see
                                         # Upload) and starves TLS

def read_uart():
    global _buf
    while uart.any():
        k = uart.readinto(_chunk)
        if not k:
            return
        st["rx"] = time.ticks_ms()
        st["spin"] += 1
        _buf += _chunk[:k]
        while True:
            i = _buf.find(b"\n")
            if i < 0:
                break
            line, _buf = _buf[:i], _buf[i + 1:]
            handle(line)
        if len(_buf) > 256:              # no newline in sight: garbage/baud
            _buf = b""

# --- WiFi -------------------------------------------------------------------
# Scan once at start and connect if our network is visible. Afterwards only
# the status is polled; a dropped link is reconnected every WIFI_RETRY ms
# without a (blocking) re-scan, so GPS reading isn't stalled. The driver
# reports a handshake timeout on a weak link as STAT_WRONG_PASSWORD and
# then gives up, hence "auth fail" and our own retries.
#
# The link picks the mode (see mode_tick): while connected, nothing is logged
# and finished GPX files are uploaded; the moment it drops, logging resumes
# into a new file. GPS is read all the time, so that switch is immediate.

def load_env(path):
    """KEY=VALUE lines -> dict. Surrounding quotes are stripped."""
    env = {}
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if not line or line[0] == "#" or "=" not in line:
                    continue
                k, v = line.split("=", 1)
                k, v = k.strip(), v.strip()
                if len(v) >= 2 and v[0] == v[-1] and v[0] in "\"'":
                    v = v[1:-1]
                env[k] = v
    except OSError:
        pass                             # no .env on the device
    return env

WIFI_STATES = {}
for _n, _s in (("STAT_IDLE", "idle"), ("STAT_CONNECTING", "connecting"),
               ("STAT_GOT_IP", "connected"), ("STAT_NO_AP_FOUND", "no AP"),
               ("STAT_WRONG_PASSWORD", "auth fail"),
               ("STAT_BEACON_TIMEOUT", "lost"), ("STAT_ASSOC_FAIL", "assoc fail"),
               ("STAT_HANDSHAKE_TIMEOUT", "handshake")):
    if hasattr(network, _n):
        WIFI_STATES[getattr(network, _n)] = _s

wifi = {}

def wifi_reset():
    wifi.clear()
    wifi.update(wlan=None, ssid=None, pw=None, state="off", ip=None,
                rssi=None, tries=0, last_check=time.ticks_ms())

def _connect():
    w = wifi["wlan"]
    try:
        w.disconnect()
    except OSError:
        pass
    try:
        w.connect(wifi["ssid"], wifi["pw"])
        wifi["state"] = "connecting"
    except OSError:
        wifi["state"] = "error"
    wifi["tries"] += 1

def wifi_start(env):
    wifi["ssid"], wifi["pw"] = env.get("SSID"), env.get("WPA2")
    up["url"] = env.get("GEO_URL", "").rstrip("/") or None
    up["key"] = env.get("API_KEY") or None
    if not wifi["ssid"] or wifi["pw"] is None:
        wifi["state"] = "no .env"
        return
    oled.fill(0)
    oled.text("WiFi scanning...", 0, 28)
    oled.show()
    try:
        w = wifi["wlan"] = network.WLAN(network.STA_IF)
        w.active(True)
        found = any(n[0] == wifi["ssid"].encode() for n in w.scan())
    except OSError:
        wifi["state"] = "error"
        return
    if found:
        _connect()
    else:
        wifi["state"] = "not found"
    wifi["last_check"] = time.ticks_ms()

def wifi_tick():
    w = wifi["wlan"]
    if w is None:
        return
    s = w.status()
    now = time.ticks_ms()
    every = WIFI_CHECK if s == network.STAT_GOT_IP else WIFI_RETRY
    check = time.ticks_diff(now, wifi["last_check"]) >= every
    if s == network.STAT_GOT_IP:
        if wifi["state"] != "connected" or check:
            wifi["state"] = "connected"
            wifi["ip"] = w.ifconfig()[0]
            try:
                wifi["rssi"] = w.status("rssi")
            except (OSError, ValueError):
                wifi["rssi"] = None
    elif s == network.STAT_CONNECTING:
        wifi["state"] = "connecting"
    elif wifi["state"] not in ("not found", "error"):
        wifi["state"] = WIFI_STATES.get(s, "lost")
    if not check:
        return
    wifi["last_check"] = now
    if s not in (network.STAT_GOT_IP, network.STAT_CONNECTING):
        wifi["ip"] = wifi["rssi"] = None
        _connect()

def wifi_stop():
    w = wifi["wlan"]
    if w is None:
        return
    try:
        w.disconnect()
        w.active(False)
    except OSError:
        pass
    wifi["state"] = "off"

def online():
    return wifi["state"] == "connected"

# --- GPX logger -------------------------------------------------------------
# The file is always a complete GPX document: each batch overwrites the
# closing tags with new points plus the closing tags again. LittleFS commits
# on close, so a power cut leaves either the previous or the new version.

HEADER = ('<?xml version="1.0" encoding="UTF-8"?>\n'
          '<gpx version="1.1" creator="esp32 gps_logger" '
          'xmlns="http://www.topografix.com/GPX/1/1">\n'
          '<trk><name>%s</name><trkseg>\n')
FOOTER = "</trkseg></trk></gpx>\n"
NEW_SEG = "</trkseg><trkseg>\n"

log = {}

def log_reset():
    log.clear()
    log.update(file=None, created=False, batch=[], points=0, err=0,
               fix=False, lost_at=None, new_seg=False, full=False, online=False,
               last_sample=None, last_flush=time.ticks_ms(), last_t=None,
               prev=None)

def free():
    try:
        s = os.statvfs(LOG_DIR)
        return s[0] * s[3]
    except OSError:
        return None

def has_fix(now):
    return (st["valid"] and st["date"] and st["time"] and st["lat"] is not None
            and st["pos_at"] is not None
            and time.ticks_diff(now, st["pos_at"]) < STALE)

def start_file():
    try:
        os.mkdir(LOG_DIR)
    except OSError:
        pass                             # already exists
    name = st["date"].replace("-", "") + "_" + st["time"].replace(":", "")
    log.update(file=LOG_DIR + "/" + name + ".gpx", created=False, points=0,
               new_seg=False, prev=None)
    flush()                              # create it right away

def flush():
    """Write pending points to the current file, opened only for the write."""
    log["last_flush"] = time.ticks_ms()
    path, batch = log["file"], log["batch"]
    if path is None or (log["created"] and not batch):
        return
    room = free()
    if room is not None and room < MIN_FREE:
        log["full"] = True
        del batch[:]
        return
    pts = "".join(batch)
    try:
        if log["created"]:
            f = open(path, "r+b")
            try:
                f.seek(-len(FOOTER), 2)
                f.write(((NEW_SEG if log["new_seg"] else "") + pts
                         + FOOTER).encode())
            finally:
                f.close()
        else:
            name = path[len(LOG_DIR) + 1:-4]
            f = open(path, "wb")
            try:
                f.write((HEADER % name + pts + FOOTER).encode())
            finally:
                f.close()
            log["created"] = True
    except OSError:
        log["err"] += 1                  # keep the batch, retry next FLUSH
        return
    log["points"] += len(batch)
    up["left"] += len(batch)
    del batch[:]
    log["new_seg"] = False

def _dist(lat1, lon1, lat2, lon2):
    """Metres between two nearby positions (equirectangular approximation)."""
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    y = math.radians(lat2 - lat1)
    return 6371000 * math.sqrt(x * x + y * y)

def log_tick():
    """Sample a position every SAMPLE ms and record it if it is more than
    MIN_MOVE m from the last recorded point; write a batch every FLUSH ms."""
    if log["full"] or log["online"]:
        return
    now = time.ticks_ms()
    if not has_fix(now):
        if log["fix"]:
            log["fix"], log["lost_at"] = False, now
            flush()
        return
    if not log["fix"]:
        log["fix"] = True
        if (log["file"] is None or time.ticks_diff(now, log["lost_at"])
                > NEW_FILE_GAP * 1000):
            flush()                      # leftovers of a failed write
            del log["batch"][:]
            start_file()
        else:
            log["new_seg"] = True
            log["prev"] = None           # start the segment with this fix
    if (log["last_sample"] is None
            or time.ticks_diff(now, log["last_sample"]) >= SAMPLE):
        log["last_sample"] = now
        t = st["date"] + "T" + st["time"] + "Z"
        prev = log["prev"]
        if t != log["last_t"] and (
                prev is None
                or _dist(prev[0], prev[1], st["lat"], st["lon"]) > MIN_MOVE):
            log["last_t"] = t
            log["prev"] = (st["lat"], st["lon"])
            ele = ("<ele>%.1f</ele>" % st["alt"]
                   if st["alt"] is not None else "")
            log["batch"].append(
                '<trkpt lat="%.6f" lon="%.6f">%s<time>%s</time>%s</trkpt>\n'
                % (st["lat"], st["lon"], ele, t, obd_ext(now)))
            if len(log["batch"]) > MAX_BATCH:
                log["batch"].pop(0)
    if time.ticks_diff(now, log["last_flush"]) >= FLUSH:
        flush()

def mode_tick():
    """Switch between logging (offline) and uploading (online)."""
    on = online()
    if on == log["online"]:
        return
    log["online"] = on
    if on:                               # close the file so it gets uploaded
        flush()
        del log["batch"][:]
        log.update(file=None, fix=False, new_seg=False, last_t=None,
                   prev=None)
        up["left"] = count_points()
        up["next_at"] = time.ticks_ms()
    else:                                # next fix starts a new file
        log.update(fix=False, lost_at=None)

# --- Upload ---------------------------------------------------------------
# Finished GPX files (never the one being recorded) are sent to Dawarich's
# Overland endpoint, oldest first, UPLOAD_BATCH points per POST. Accepted
# points are cut from the file by rewriting it to a .tmp and renaming it over
# the original; the empty file is then removed. A power cut between the POST
# and the rewrite only resends points, which Dawarich deduplicates.
#
# TLS (mbedTLS) needs ~40 kB contiguous from the ESP-IDF heap, outside the
# Python heap. If the Python heap ever grows (see gc.threshold at the top) it
# takes that block, so allocations here stay small: the request body goes
# into a buffer allocated once at import. For the same reason the module must
# be deployed precompiled (gps_logger.mpy): compiling the source on the
# device grows the heap for good.

_body = bytearray(UPLOAD_BATCH * POINT_MAX + 32)

up = {}

def up_reset():
    up.clear()
    up.update(url=None, key=None, state="idle", sent=0, err=0, files=0,
              left=0, next_at=time.ticks_ms())
    try:
        for n in os.listdir(LOG_DIR):
            if n.endswith(".tmp"):
                os.remove(LOG_DIR + "/" + n)   # rewrite cut by power loss
    except OSError:
        pass
    oled.fill(0)
    oled.text("Counting points", 0, 28)
    oled.show()
    up["left"] = count_points()

def count_points():
    """Points in all GPX files, the one being recorded included. Reads every
    file, so it runs only at boot and when going online; flush() and
    drop_points() keep up["left"] current in between."""
    try:
        names = [n for n in os.listdir(LOG_DIR) if n.endswith(".gpx")]
    except OSError:
        return 0
    total = 0
    for n in names:
        try:
            with open(LOG_DIR + "/" + n) as f:
                for line in f:
                    if line.startswith("<trkpt"):
                        total += 1
        except OSError:
            pass                         # unreadable: upload reports fs err
    return total

def pending():
    """Finished GPX files, oldest first (names are YYYYMMDD_HHMMSS)."""
    try:
        names = sorted(n for n in os.listdir(LOG_DIR) if n.endswith(".gpx"))
    except OSError:
        return []
    return [p for p in (LOG_DIR + "/" + n for n in names) if p != log["file"]]

def _attr(line, name):
    i = line.find(name + '="')
    if i < 0:
        return None
    i += len(name) + 2
    return line[i:line.find('"', i)]

def _tag(line, name):
    i = line.find("<" + name + ">")
    if i < 0:
        return None
    i += len(name) + 2
    return line[i:line.find("</" + name + ">", i)]

def _put(pos, b):
    _body[pos:pos + len(b)] = b
    return pos + len(b)

def read_batch(path):
    """Fill _body with the first UPLOAD_BATCH points of path as Overland
    JSON -> (points read, body length or 0 if all of them were malformed)."""
    n, pos, sep = 0, _put(0, b'{"locations":['), b""
    with open(path) as f:
        for line in f:
            if not line.startswith("<trkpt"):
                continue
            n += 1
            lat, lon, t = _attr(line, "lat"), _attr(line, "lon"), _tag(line, "time")
            if lat is not None and lon is not None and t is not None:
                ele = _tag(line, "ele")
                loc = ('{"type":"Feature","geometry":{"type":"Point",'
                       '"coordinates":[%s,%s]},"properties":{"timestamp":"%s",'
                       '%s"device_id":"%s"}}'
                       % (lon, lat, t, '"altitude":%s,' % ele if ele else "",
                          DEVICE_ID)).encode()
                if len(loc) < POINT_MAX:     # longer = garbage, skipped
                    pos = _put(_put(pos, sep), loc)
                    sep = b","
            if n >= UPLOAD_BATCH:
                break
    if not sep:
        return n, 0
    return n, _put(pos, b"]}")

def drop_points(path, n):
    """Remove the first n points from path; delete it if none are left."""
    tmp, left, cut = path + ".tmp", 0, n
    with open(path) as src, open(tmp, "w") as dst:
        for line in src:
            if line.startswith("<trkpt"):
                if n:
                    n -= 1
                    continue
                left += 1
            elif line == NEW_SEG and n:
                continue                 # segment break inside the cut part
            dst.write(line)
    if left:
        os.rename(tmp, path)
    else:
        os.remove(tmp)
        os.remove(path)
    up["left"] = max(up["left"] - (cut - n), 0)

def upload_tick():
    """Send one batch when online and due."""
    now = time.ticks_ms()
    if (wifi["state"] != "connected" or up["state"] in ("auth fail", "no .env")
            or time.ticks_diff(now, up["next_at"]) < 0):
        return
    if not up["url"] or not up["key"]:
        up["state"] = "no .env"
        return
    files = pending()
    up["files"] = len(files)
    if not files:
        up["left"] = 0
        up["state"] = "idle"
        up["next_at"] = time.ticks_add(now, UPLOAD_RETRY)
        return
    path = files[0]
    try:
        n, size = read_batch(path)
        if not n:                        # header/footer only
            os.remove(path)
            return
        if not size:                     # only malformed points: cut them
            drop_points(path, n)
            return
    except OSError:
        up["err"] += 1
        up["state"] = "fs err"
        up["next_at"] = time.ticks_add(now, UPLOAD_RETRY)
        return
    except MemoryError:
        _low_mem(now)
        return
    up["state"] = "sending"
    read_uart()                          # drain NMEA before blocking
    gc.collect()
    code = None
    try:
        import requests
        r = requests.post("%s/api/v1/overland/batches?api_key=%s"
                          % (up["url"], up["key"]),
                          data=memoryview(_body)[:size],
                          headers={"Content-Type": "application/json"},
                          timeout=HTTP_TIMEOUT)
        code = r.status_code
        r.close()
    except (OSError, ImportError) as e:
        print("upload", repr(e))
    except MemoryError:
        r = None
        _low_mem(time.ticks_ms())
        return
    r = None
    gc.collect()
    now = time.ticks_ms()
    print("upload", path, n, code, gc.mem_free())
    if code is not None and 200 <= code < 300:
        try:
            drop_points(path, n)
        except OSError:
            up["err"] += 1
            up["state"] = "fs err"
            up["next_at"] = time.ticks_add(now, UPLOAD_RETRY)
            return
        up["sent"] += n
        up["state"] = "ok"
        up["next_at"] = time.ticks_add(now, UPLOAD_GAP)
    elif code == 401:
        up["state"] = "auth fail"        # wrong API_KEY: stop for this run
    else:
        up["err"] += 1
        up["state"] = "err " + ("net" if code is None else str(code))
        up["next_at"] = time.ticks_add(now, UPLOAD_RETRY)

def _low_mem(now):
    gc.collect()
    up["err"] += 1
    up["state"] = "low mem"
    up["next_at"] = time.ticks_add(now, UPLOAD_RETRY)
    print("upload MemoryError, free", gc.mem_free())

# --- OBD-II -----------------------------------------------------------------
# An ELM327-style adapter over BLE (MicroPython has no Classic Bluetooth). It
# is used only while logging with a fix: then the adapter is connected, its
# serial characteristic found and subscribed to, the ELM327 initialised, and
# the Mode 01 PIDs it supports (per 0100) polled in turn, one request at a
# time. Each recorded point gets the values that are fresh. Without a fix
# nothing is sent; BLE is switched off after OBD_IDLE, and at once when going
# online, as NimBLE shares the IDF heap that TLS uploads need.
#
# The IRQ handler walks connect -> services -> characteristics -> CCCD ->
# subscribed, and collects the replies; obd_tick() sends the commands.

_IRQ_PERIPHERAL_CONNECT = 7
_IRQ_PERIPHERAL_DISCONNECT = 8
_IRQ_GATTC_SERVICE_RESULT = 9
_IRQ_GATTC_SERVICE_DONE = 10
_IRQ_GATTC_CHARACTERISTIC_RESULT = 11
_IRQ_GATTC_CHARACTERISTIC_DONE = 12
_IRQ_GATTC_DESCRIPTOR_RESULT = 13
_IRQ_GATTC_DESCRIPTOR_DONE = 14
_IRQ_GATTC_WRITE_DONE = 17
_IRQ_GATTC_NOTIFY = 18
_IRQ_GATTC_INDICATE = 19
_IRQ_ENCRYPTION_UPDATE = 28
_IRQ_PASSKEY_ACTION = 31
_PASSKEY_ACTION_INPUT = 2
_IO_KEYBOARD_ONLY = 2
_NO_CONN = 0xFFFF                        # connect attempt that timed out

_WRITE_NR, _WRITE, _NOTIFY, _INDICATE = 0x04, 0x08, 0x10, 0x20
_CCCD = bluetooth.UUID(0x2902)
_STD = (bluetooth.UUID(0x1800), bluetooth.UUID(0x1801),   # GAP, GATT,
        bluetooth.UUID(0x180A))                           # device info
_KNOWN = (bluetooth.UUID(0xFFF0), bluetooth.UUID(0xFFE0),  # tried first
          bluetooth.UUID("E7810A71-73AE-499D-8C15-FAA9AEF0C3F2"))  # Vgate

OBD_INIT = ("ATZ", "ATE0", "ATL0", "ATS0", "ATH0", "ATSP0", "0100")
OBD_PIDS = (("0C", "rpm", 2, lambda d: "%d" % ((d[0] * 256 + d[1]) // 4)),
            ("0D", "speed", 1, lambda d: "%d" % d[0]),                # km/h
            ("11", "throttle", 1, lambda d: "%.1f" % (d[0] * 100 / 255)))
_BUSY = ("connecting", "discover", "init", "ok", "no ECU", "no PID")

ble = None
obd = {}

def obd_reset(env):
    global ble
    obd.clear()
    obd.update(addr=None, addr_type=0, pin=None, ble=False, state="no .env",
               since=time.ticks_ms(), conn=None, svcs=[], chars=[], rx=None,
               tx=None, tx_mode=0, ind=False, cccd=None, paired=False,
               buf=b"", cmd=None, sent_at=0, queue=[], pids=[], i=0, vals={},
               next_at=time.ticks_ms(), idle_at=time.ticks_ms())
    try:
        a = bytes(int(x, 16) for x in env.get("OBDII_ADDRESS", "").split(":"))
    except ValueError:
        return
    if len(a) != 6:
        return
    try:
        obd["pin"] = int(env["OBDII_PIN"])
    except (KeyError, ValueError):
        pass
    obd.update(addr=a, state="off")
    ble = bluetooth.BLE()
    ble.irq(_obd_irq)

def _obd_state(s):
    obd["state"], obd["since"] = s, time.ticks_ms()

def _obd_fail(why):
    print("obd", why)
    _obd_state("err")
    obd["next_at"] = time.ticks_add(time.ticks_ms(), OBD_RETRY)
    try:
        if obd["conn"] is None:
            ble.gap_connect(None)        # cancel a pending connect
        else:
            ble.gap_disconnect(obd["conn"])
    except OSError:
        pass

def _obd_pick():
    """-> (notify char, write char, service end) of the adapter's serial
    service, or None. Chars are (value handle, properties)."""
    svcs = ([s for s in obd["svcs"] if s[2] in _KNOWN]
            + [s for s in obd["svcs"] if s[2] not in _KNOWN + _STD])
    for start, end, _ in svcs:
        rx = tx = None
        for c in obd["chars"]:
            if not start <= c[0] <= end:
                continue
            if rx is None and c[1] & (_NOTIFY | _INDICATE):
                rx = c
            if tx is None and c[1] & (_WRITE | _WRITE_NR):
                tx = c
        if rx is not None and tx is not None:
            if rx[1] & (_WRITE | _WRITE_NR):
                tx = rx
            return rx, tx, end
    return None

def _obd_subscribe():
    ble.gattc_write(obd["conn"], obd["cccd"],
                    b"\x02\x00" if obd["ind"] else b"\x01\x00", 1)

def _obd_irq(event, data):
    o = obd
    try:
        if event == _IRQ_PERIPHERAL_CONNECT:
            o.update(conn=data[0], svcs=[], chars=[], cccd=None, paired=False,
                     buf=b"", cmd=None)
            _obd_state("discover")
            ble.gattc_discover_services(data[0])
        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            o.update(conn=None, cmd=None)
            if data[0] == _NO_CONN:      # not found: try the other type
                o["addr_type"] ^= 1
                if o["state"] == "connecting":
                    _obd_state("not found")
            elif o["state"] in _BUSY:
                _obd_state("lost")
        elif event == _IRQ_GATTC_SERVICE_RESULT:
            o["svcs"].append((data[1], data[2], bluetooth.UUID(data[3])))
        elif event == _IRQ_GATTC_SERVICE_DONE:
            ble.gattc_discover_characteristics(o["conn"], 1, 0xFFFF)
        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            o["chars"].append((data[2], data[3]))
        elif event == _IRQ_GATTC_CHARACTERISTIC_DONE:
            p = _obd_pick()
            if p is None:
                _obd_fail("no serial characteristic")
                _obd_state("no serial")
                return
            rx, tx, end = p
            o.update(rx=rx[0], tx=tx[0], ind=not rx[1] & _NOTIFY,
                     tx_mode=0 if tx[1] & _WRITE_NR else 1)
            if rx[0] < end:
                ble.gattc_discover_descriptors(o["conn"], rx[0] + 1, end)
            else:
                o["cccd"] = rx[0] + 1
                _obd_subscribe()
        elif event == _IRQ_GATTC_DESCRIPTOR_RESULT:
            if o["cccd"] is None and bluetooth.UUID(data[2]) == _CCCD:
                o["cccd"] = data[1]
        elif event == _IRQ_GATTC_DESCRIPTOR_DONE:
            if o["cccd"] is None:
                o["cccd"] = o["rx"] + 1
            _obd_subscribe()
        elif event == _IRQ_GATTC_WRITE_DONE:
            status = data[2]
            if not status:
                if data[1] == o["cccd"] and o["state"] == "discover":
                    o.update(queue=list(OBD_INIT), next_at=time.ticks_ms())
                    _obd_state("init")
            elif (status & 0xFF) in (5, 15) and not o["paired"]:
                o["paired"] = True       # insufficient authentication or
                ble.gap_pair(o["conn"])  # encryption: pair, then retry
            else:
                _obd_fail("write status %d" % status)
        elif event == _IRQ_ENCRYPTION_UPDATE:
            if data[1] and o["state"] == "discover":
                _obd_subscribe()
        elif event in (_IRQ_GATTC_NOTIFY, _IRQ_GATTC_INDICATE):
            if data[1] == o["rx"] and len(o["buf"]) < 256:
                o["buf"] += bytes(data[2])
        elif event == _IRQ_PASSKEY_ACTION:
            if data[1] == _PASSKEY_ACTION_INPUT and o["pin"] is not None:
                ble.gap_passkey(data[0], data[1], o["pin"])
    except (OSError, ValueError) as e:
        _obd_fail("irq %d %r" % (event, e))

def _obd_parse(buf, pid, n):
    """ELM327 reply -> n data bytes of its first '41<pid>' line, or None
    (NO DATA, UNABLE TO CONNECT, ?, garbage...)."""
    try:
        s = buf.decode()
    except UnicodeError:
        return None
    want = "41" + pid
    for line in s.replace(" ", "").replace("\n", "\r").replace(">", "\r") \
            .split("\r"):
        if line.startswith(want) and len(line) >= 4 + 2 * n:
            try:
                return [int(line[4 + 2 * k:6 + 2 * k], 16) for k in range(n)]
            except ValueError:
                return None
    return None

def _obd_send(cmd, now):
    obd.update(buf=b"", cmd=cmd, sent_at=now)
    ble.gattc_write(obd["conn"], obd["tx"], (cmd + "\r").encode(),
                    obd["tx_mode"])

def _obd_reply(cmd, buf, now):
    o = obd
    if cmd == "0100":
        d = _obd_parse(buf, "00", 4)
        print("obd", cmd, buf)
        if d is None:                    # ignition off or no ECU yet
            _obd_state("no ECU")
            o["next_at"] = time.ticks_add(now, OBD_RETRY)
            return
        mask = d[0] << 24 | d[1] << 16 | d[2] << 8 | d[3]
        o.update(i=0, pids=[p for p in OBD_PIDS
                            if mask >> (32 - int(p[0], 16)) & 1])
        print("obd PIDs %08x" % mask, [p[1] for p in o["pids"]])
        _obd_state("ok" if o["pids"] else "no PID")
    elif o["state"] == "ok":
        for pid, name, n, fmt in o["pids"]:
            if cmd == "01" + pid:
                d = _obd_parse(buf, pid, n)
                if d is not None:
                    o["vals"][name] = (fmt(d), now)
    else:
        print("obd", cmd, buf)

def obd_tick():
    """Connect to the adapter and poll it while logging with a fix."""
    o = obd
    if o["addr"] is None:
        return
    now = time.ticks_ms()
    if log["online"] or log["full"] or not log["fix"]:
        if o["ble"] and (log["online"] or log["full"] or time.ticks_diff(
                now, o["idle_at"]) >= OBD_IDLE):
            obd_stop()
        return
    o["idle_at"] = now
    s = o["state"]
    if s in ("connecting", "discover"):
        if time.ticks_diff(now, o["since"]) > OBD_CONNECT + OBD_SLOW:
            _obd_fail("timeout " + s)
        return
    if o["conn"] is None:
        if time.ticks_diff(now, o["next_at"]) < 0:
            return
        o["next_at"] = time.ticks_add(now, OBD_RETRY)
        try:
            if not o["ble"]:
                ble.active(True)
                o["ble"] = True
                if o["pin"] is not None:
                    ble.config(io=_IO_KEYBOARD_ONLY, mitm=True)
            ble.gap_connect(o["addr_type"], o["addr"], OBD_CONNECT)
            _obd_state("connecting")
        except (OSError, ValueError) as e:
            _obd_fail("connect %r" % e)
        return
    cmd = o["cmd"]
    if cmd is not None:
        done = b">" in o["buf"]
        wait = OBD_SLOW if cmd in ("ATZ", "0100") else OBD_TIMEOUT
        if not done and time.ticks_diff(now, o["sent_at"]) < wait:
            return
        o["cmd"] = None
        _obd_reply(cmd, o["buf"] if done else b"", now)
    if time.ticks_diff(now, o["next_at"]) < 0:
        return
    try:
        if o["state"] == "init" and o["queue"]:
            _obd_send(o["queue"].pop(0), now)
        elif o["state"] == "no ECU":
            _obd_send("0100", now)
        elif o["state"] == "ok":
            o["i"] = (o["i"] + 1) % len(o["pids"])
            _obd_send("01" + o["pids"][o["i"] - 1][0], now)
    except OSError as e:
        _obd_fail("send %r" % e)

def obd_ext(now):
    """GPX <extensions> with the fresh OBD values, or ""."""
    s = ""
    for _, name, _, _ in OBD_PIDS:
        v = obd["vals"].get(name)
        if v is not None and time.ticks_diff(now, v[1]) < OBD_STALE:
            s += "<%s>%s</%s>" % (name, v[0], name)
    return "<extensions>" + s + "</extensions>" if s else ""

def obd_stop():
    o = obd
    if not o["ble"]:
        return
    o["ble"] = False
    try:
        if o["conn"] is not None:
            ble.gap_disconnect(o["conn"])
    except OSError:
        pass
    try:
        ble.active(False)
    except OSError:
        pass
    o.update(conn=None, cmd=None, next_at=time.ticks_ms())
    _obd_state("off")

# --- Screen -----------------------------------------------------------------

def status():
    if st["rx"] is None or time.ticks_diff(time.ticks_ms(), st["rx"]) > NO_DATA:
        return "No data!"
    if not st["valid"]:
        return "Waiting fix"
    s = {2: "2D fix", 3: "3D fix"}.get(st["fixtype"], "Fix")
    return s + " DGPS" if st["quality"] == 2 else s

def _short(n):
    """Fit a count into the 4 chars left after "Points left "."""
    return str(n) if n < 10000 else "%dk" % (n // 1000)

def draw():
    oled.fill(0)
    oled.fill_rect(0, 0, 128, 8, 1)
    oled.text("GPS Logger", 0, 0, 0)
    oled.text("|/-\\"[st["spin"] % 4], 120, 0, 0)
    for row, line in enumerate((
            "WiFi " + wifi["state"],
            "GPS " + status(),
            "Mode " + ("UPLOAD" if log["online"] else "LOG"),
            "BLE " + obd["state"],
            "Points left " + _short(up["left"]))):
        oled.text(line, 0, 10 + row * 11)
    oled.show()

# --- Main loop --------------------------------------------------------------

def run():
    """Show the logger state on the OLED, log GPX (with OBD-II data)
    offline, upload online."""
    env = load_env(ENV_FILE)
    reset()
    log_reset()
    wifi_reset()
    up_reset()
    obd_reset(env)
    wifi_start(env)
    try:
        _loop()
    finally:
        flush()                          # don't lose the pending batch
        obd_stop()
        wifi_stop()

def _loop():
    last = None
    while True:
        read_uart()
        log_tick()
        wifi_tick()
        mode_tick()
        obd_tick()                       # BLE off before an upload starts
        upload_tick()
        now = time.ticks_ms()
        if last is None or time.ticks_diff(now, last) >= REFRESH:
            draw()
            last = now
        time.sleep_ms(20)

run()
