from machine import Pin, I2C, UART, WDT
import machine, sys, time, os, network, gc, math

# Collect garbage early and often. When an allocation doesn't fit, the ESP32
# port grows the Python heap with the largest free IDF block (~60 kB) instead,
# and TLS (mbedTLS lives in the IDF heap) then fails with ENOMEM.
gc.threshold(8192)

BAUD = 9600                              # NEO-6M/7M/8M default

# Board pins. ESP32 (WROOM): UART2 on GPIO16/17. The ESP32-C6 has no UART2 and
# GPIO16/17 are its console UART, so it uses UART1 on GPIO4 (TX) / GPIO5 (RX).
# main.py keeps the matching boot-bypass pin.
if "ESP32C6" in getattr(sys.implementation, "_machine", ""):
    GPS_UART, GPS_TX, GPS_RX, I2C_SCL, I2C_SDA = 1, 4, 5, 7, 6
else:
    GPS_UART, GPS_TX, GPS_RX, I2C_SCL, I2C_SDA = 2, 17, 16, 22, 21

# OLED is optional: boards without a display (or a failed I²C probe) run
# headless. draw() and the boot banners become no-ops. Runtime I²C errors
# disable the display instead of resetting the chip.
oled = None
oled_err = 0
OLED_MAX_ERR = 3
try:
    import ssd1306
    _i2c = I2C(0, scl=Pin(I2C_SCL), sda=Pin(I2C_SDA))
    oled = ssd1306.SSD1306_I2C(128, 64, _i2c)
except Exception as e:
    print("oled", repr(e))

uart = UART(GPS_UART, baudrate=BAUD, tx=GPS_TX, rx=GPS_RX, rxbuf=4096)   # rides out HTTP calls

REFRESH = 500                            # ms between screen redraws
NO_DATA = 3000                           # ms of silence = "No data!"

SAMPLE = 1000                            # ms between sampled positions
MIN_MOVE = 3                             # m from the last recorded point
FLUSH = 60000                            # ms between batch writes to flash
NEW_FILE_GAP = 300                       # s without fix → new file; also WiFi max age
HOME_STOP_SPEED = 5                      # km/h below which home stop timer runs
HOME_STOP_DELAY = 120000                 # ms below HOME_STOP_SPEED before pausing
HOME_MOVE_CONFIRM = 30000                # ms above HOME_STOP_SPEED to resume
HOME_MOVE_DISTANCE = 60                  # m of confirmed displacement to resume
HOME_MOVE_MAX_WAIT = 120000              # ms before restarting a stalled move check
HOME_STILL_RESET = 10000                 # ms of sustained speed that restarts
HOME_STILL_RESET_DISTANCE = 15           # m  ... the stop timer (not paused)
HOME_PENDING_MAX = 90                    # samples kept while confirming a resume
STALE = 2000                             # ms: older position = no fix
LOG_DIR = "/gpx"
MIN_FREE = 32768                         # bytes of flash kept free
MAX_BATCH = 180                          # RAM cap while flash writes fail
MAX_HDOP = 5                             # skip samples with worse dilution
HDOP_M = 5                               # metres of UERE per HDOP unit
FULL_CHECK = 5000                        # ms between statvfs polls when full

ENV_FILE = "/.env"                       # SSID, WPA2, GEO_URL, API_KEY lines
WIFI_CHECK = FLUSH                       # ms between checks while connected
WIFI_RETRY = 15000                       # ms between reconnects while down
WIFI_BACKOFF_MAX = 600000                # ms cap on wifi reconnect backoff
CONNECT_TIMEOUT = 12000                  # ms to wait for GOT_IP per network
FAIL_HOLD = 120000                       # ms to demote a network that failed
IGNORE_STALE_MS = 1500                   # leftover fail reason after connect()
CLEAR_WAIT = 500                         # ms to wait for STAT_IDLE after clear
LOST_GRACE = 5000                        # ms to let IDF auto-reconnect after drop
ROAM_CHECK = 300000                      # ms between home-roam scans on hotspot
ROAM_BACKOFF_MAX = 3600000               # ms cap on roam retry (60 min)
ROAM_MIN_RSSI = -75                      # do not jump to a weak home AP
LINE_MAX = 512                           # bounded track-line readline
BOOT_OK_MS = 60000                       # clear boot-fail after this of _loop

UPLOAD_BATCH = 50                        # points per POST to Dawarich
BODY_CAP = UPLOAD_BATCH * 256 + 32       # JSON buffer; sized for long device_id
UPLOAD_GAP = 5000                        # ms between successful batches
UPLOAD_GAP_AWAY = 30000                  # ms between batches on a non-home net
BACKOFF_START = 15000                    # ms first retry after net/5xx/4xx
BACKOFF_MAX = 1800000                    # ms cap (30 min) on upload backoff
AUTH_BACKOFF = 1800000                   # ms retry after 401 (plan/key/limit)
HTTP_TIMEOUT = 15                        # s per socket op, not a wall clock
DEFAULT_DEVICE_ID = "Dawarich-GPS-tracker"
DEFAULT_HOSTNAME = "Dawarich-GPS-tracker"
DEVICE_ID = DEFAULT_DEVICE_ID            # Overland tracker id (overridable)
HOSTNAME = DEFAULT_HOSTNAME              # DHCP hostname (overridable)
ID_MAX = 32                              # DEVICE_ID / HOSTNAME length cap
MAX_BAD_BYTES = 65536                    # cap on appended <file>.bad
MAX_BAD_RETRY = 8                        # successful .bad batches per file per boot
MAX_GLOBAL_PAYLOAD = 3                   # consecutive 400/422 -> global backoff
WDT_TIMEOUT = 120000                     # ms; ESP32 WDT cannot be disarmed
KEEPALIVE_MAX = 600000                   # ms Ctrl-C timer may feed the WDT
PAYLOAD_ERR = (400, 413, 422)            # only these quarantine a batch
UP_WAIT_MAX = max(BACKOFF_MAX, AUTH_BACKOFF)  # longest upload/hold deadline

# --- Ticks ------------------------------------------------------------------
# ticks_ms wraps every 2**30 ms and ticks_diff is signed, so a timestamp older
# than ~6.2 days reads as being in the future. Ages and deadlines go through
# these helpers: a wrapped past timestamp counts as very old, a deadline that
# lies further ahead than it could ever have been scheduled counts as due.

TICKS_OLD = 1 << 29                      # "very old" for a wrapped age
TICKS_SKEW = 1000                        # ms a timestamp may be "ahead"

def _age(now, t):
    """ms since ticks t. A timestamp taken just after now (say read_uart()
    ran between the two ticks_ms() calls) is age 0; a wrapped (large
    negative) difference is TICKS_OLD."""
    d = time.ticks_diff(now, t)
    if d >= 0:
        return d
    return 0 if d > -TICKS_SKEW else TICKS_OLD

def _waiting(now, at, cap):
    """True while deadline at (scheduled at most cap ms ahead) is ahead."""
    d = time.ticks_diff(at, now)
    return 0 < d <= cap

# --- NMEA parsing -----------------------------------------------------------

st = {}

def reset():
    st.clear()
    st.update(start=time.ticks_ms(), rx=None, spin=0, ok=0, err=0,
              status=None, quality=None, fixtype=None, valid=False,
              time=None, date=None, lat=None, lon=None, alt=None,
              speed=None, course=None, used=None, view={},
              hdop=None, pdop=None, ttff=None, pos_at=None, hdop_at=None)

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
    """Store the sentence's position. False when its fields are empty."""
    lat, lon = _deg(f[i], f[i + 1]), _deg(f[i + 2], f[i + 3])
    if lat is None or lon is None:
        return False
    st["lat"], st["lon"] = lat, lon
    return True

def _time(s):
    if len(s) >= 6:
        st["time"] = "%s:%s:%s" % (s[0:2], s[2:4], s[4:6])

def _update_fix(ok, have_pos):
    """ok is this sentence's own verdict (RMC status / GGA quality), so a
    void RMC is not rescued by the previous epoch's GGA. pos_at only moves
    when the sentence actually carried a position."""
    st["valid"] = ok
    if ok and have_pos:
        st["pos_at"] = time.ticks_ms()
        if st["ttff"] is None:
            st["ttff"] = time.ticks_diff(st["pos_at"], st["start"]) // 1000

def rmc(talker, f):
    _time(f[1])
    st["status"] = f[2]
    have_pos = _position(f, 3)
    kn = _num(f[7])
    st["speed"] = None if kn is None else kn * 1.852
    st["course"] = _num(f[8])
    d = f[9]
    if len(d) == 6:
        st["date"] = "20%s-%s-%s" % (d[4:6], d[2:4], d[0:2])
    _update_fix(st["status"] == "A", have_pos)

def gga(talker, f):
    _time(f[1])
    have_pos = _position(f, 2)
    st["quality"] = _num(f[6], int)
    st["used"] = _num(f[7], int)
    st["hdop"] = _num(f[8])
    st["hdop_at"] = time.ticks_ms()
    st["alt"] = _num(f[9])
    _update_fix((st["quality"] or 0) > 0, have_pos)

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

# --- Watchdog ---------------------------------------------------------------
# Created only from run()/_loop(), so the GPIO25 boot-bypass REPL is not
# reset. ESP32 cannot disarm it: Ctrl-C starts a Timer that keeps feeding
# until KEEPALIVE_MAX, then the dog is allowed to reset.

_wdt = None
_keep = None
_keep_from = 0
_running = False
_boot_ok_done = False

def _boot_ok():
    """Clear the boot-fail counter after a stretch of healthy looping."""
    global _boot_ok_done
    if _boot_ok_done:
        return
    _boot_ok_done = True
    try:
        machine.RTC().memory(b"0")
    except Exception:
        pass
    try:
        os.remove("/boot_fail")
    except OSError:
        pass

def _boot_fail_bump():
    n = 0
    try:
        m = machine.RTC().memory()
        if m:
            if isinstance(m, bytes):
                m = m.split(b"\x00", 1)[0].split(b"\n", 1)[0].decode()
            s = str(m).strip()
            for c in s:
                if c < "0" or c > "9":
                    break
                n = n * 10 + (ord(c) - 48)
    except Exception:
        try:
            with open("/boot_fail") as f:
                n = int(f.read().strip() or "0")
        except Exception:
            n = 0
    n += 1
    raw = ("%d" % n).encode()
    try:
        machine.RTC().memory(raw)
    except Exception:
        try:
            f = open("/boot_fail", "w")
            f.write("%d" % n)
            f.close()
        except Exception:
            pass
    return n

def feed():
    if _wdt is not None:
        _wdt.feed()

def _keep_stop():
    global _keep
    t = _keep
    _keep = None
    if t is None:
        return
    try:
        t.deinit()
    except Exception:
        pass

def _keep_cb(timer):
    if _running:
        return
    if time.ticks_diff(time.ticks_ms(), _keep_from) > KEEPALIVE_MAX:
        _keep_stop()
        return                           # stop feeding: WDT resets the chip
    feed()

def _wdt_keepalive():
    """Feed the WDT after Ctrl-C, but only for KEEPALIVE_MAX ms."""
    global _keep, _keep_from
    if _wdt is None:
        return
    _keep_stop()
    try:
        t = machine.Timer(0)
        period = WDT_TIMEOUT // 3
        if period < 1000:
            period = 1000
        _keep_from = time.ticks_ms()
        t.init(period=period, mode=machine.Timer.PERIODIC, callback=_keep_cb)
        _keep = t
    except Exception as e:
        print("wdt timer", repr(e))

# --- WiFi -------------------------------------------------------------------
# Up to three networks. Network #1 is "home" by default and preferred
# for connections. Logging continues and finished files upload on all networks.
#
# Connect is a state machine in wifi_tick (one status poll per loop), so
# GPS sampling is not blocked. Scan once per round; try strongest visible
# known SSID (deduped by SSID). Remaining configured nets are tried in config
# order after visible ones, so a missed scan does not block fallback.
# v1.29 connect() may leave a stale status reason: the first IGNORE_STALE_MS
# of a try are not treated as final.

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

def _plain(v):
    """Value of a non-secret key without a trailing "# comment".

    Only for keys whose values can never contain "#" (flags, IPv4
    addresses, names). SSIDs, passwords, GEO_URL and API_KEY are verbatim."""
    if v is None:
        return None
    i = v.find("#")
    return (v if i < 0 else v[:i]).strip()

def _truthy(v):
    if v is None:
        return None
    s = _plain(str(v)).lower()
    if s in ("1", "true", "yes", "on"):
        return True
    if s in ("0", "false", "no", "off"):
        return False
    return None

def _ok_char(c, extra):
    """ASCII letter/digit (MicroPython 1.29 has no str.isalnum)."""
    return (("a" <= c <= "z") or ("A" <= c <= "Z") or ("0" <= c <= "9")
            or c in extra)

def _safe_name(s, default, host=False):
    """DEVICE_ID: [A-Za-z0-9._-]{1,32}. HOSTNAME: [A-Za-z0-9-]{1,32}."""
    if not s:
        return default
    if len(s) < 1 or len(s) > ID_MAX:
        print("bad name, using default")
        return default
    extra = "-" if host else "._-"
    for c in s:
        if _ok_char(c, extra):
            continue
        print("bad name, using default")
        return default
    return s

def _ipv4(s):
    """Dotted quad as given, or None when it is not a valid IPv4 address."""
    parts = (s or "").strip().split(".")
    if len(parts) != 4:
        return None
    for p in parts:
        if not p or len(p) > 3:
            return None
        for c in p:
            if c < "0" or c > "9":
                return None
        if int(p) > 255:
            return None
    return ".".join(parts)

def _static_ip(env, keys):
    """(ip, mask, gateway, dns) from IP/MASK/GW/DNS keys, or None for DHCP.

    A static IP needs a gateway; MASK defaults to 255.255.255.0 and DNS to the
    gateway. An invalid entry falls back to DHCP instead of leaving the board
    unreachable."""
    ipk, mk, gk, dk = keys
    raw = _plain(env.get(ipk))
    if not raw:
        return None
    ip = _ipv4(raw)
    gw = _ipv4(_plain(env.get(gk)))
    if not ip or not gw:
        print("static ip", ipk, "needs a valid", ipk, "and", gk, "- using DHCP")
        return None
    mask_raw, dns_raw = _plain(env.get(mk)), _plain(env.get(dk))
    mask, dns = _ipv4(mask_raw), _ipv4(dns_raw)
    if mask_raw and not mask:
        print("static ip", mk, "invalid - using 255.255.255.0")
    if dns_raw and not dns:
        print("static ip", dk, "invalid - using", gk)
    return (ip, mask or "255.255.255.0", gw, dns or gw)

def _load_nets(env):
    nets = []
    rows = (
        ("SSID", "WPA2", "HOME1", "SSID_HOME", "SSID_HIDDEN", True,
         ("IP", "MASK", "GW", "DNS")),
        ("SSID2", "WPA2_2", "HOME2", "SSID2_HOME", "SSID2_HIDDEN", False,
         ("IP2", "MASK2", "GW2", "DNS2")),
        ("SSID3", "WPA2_3", "HOME3", "SSID3_HOME", "SSID3_HIDDEN", False,
         ("IP3", "MASK3", "GW3", "DNS3")),
    )
    for sk, pk, hk, hk2, hidk, default_home, ipkeys in rows:
        s = env.get(sk)
        if not s:
            continue
        p = env.get(pk)
        home = default_home
        t = _truthy(env.get(hk2))
        if t is None:
            t = _truthy(env.get(hk))
        if t is not None:
            home = t
        hidden = bool(_truthy(env.get(hidk)))
        nets.append({"ssid": s, "pw": "" if p is None else p,
                     "home": home, "hidden": hidden, "bssid": None,
                     "ip": _static_ip(env, ipkeys)})
    return nets

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
    wifi.update(wlan=None, ssid=None, pw=None, nets=[], failed={},
                state="off", ip=None, rssi=None, tries=0, home=False,
                backoff=WIFI_RETRY, last_check=time.ticks_ms(),
                cand=[], cand_i=0, try_at=None, saw_connecting=False,
                roam_at=time.ticks_ms(), prev_status=None,
                roam_backoff=ROAM_CHECK, moved_at=None, roaming=False,
                blank_rssi=None, blank_bssid=None)

def _set_hostname(name):
    if not name:
        return
    try:
        network.hostname(name)
        return
    except (AttributeError, OSError, TypeError, ValueError):
        pass
    w = wifi.get("wlan")
    if w is None:
        return
    try:
        w.config(dhcp_hostname=name)
    except (AttributeError, OSError, TypeError, ValueError):
        pass

def _recently_failed(ssid, now):
    t = wifi["failed"].get(ssid)
    return t is not None and _age(now, t) < FAIL_HOLD

def _mark_failed(ssid):
    wifi["failed"][ssid] = time.ticks_ms()
    print("wifi fail", ssid)
    if wifi.get("roaming") and _net_home(ssid):
        # Hidden home: keep 5 min. A neighbour's empty-SSID AP can trigger
        # a try; growing backoff would delay a real visible-home switch.
        if not _net_hidden(ssid):
            b = wifi.get("roam_backoff") or ROAM_CHECK
            wifi["roam_backoff"] = min(max(b * 2, ROAM_CHECK), ROAM_BACKOFF_MAX)
        wifi["roaming"] = False

def _scan_known(w):
    """Visible known nets, strongest BSSID per SSID, or None if scan fails.
    Hidden APs are listed with an empty SSID; the strongest blank RSSI is
    a hint that *some* hidden AP is nearby (not necessarily ours)."""
    by = {}
    for n in wifi["nets"]:
        by[n["ssid"]] = n
    try:
        aps = w.scan()
    except OSError:
        wifi["blank_rssi"] = None
        wifi["blank_bssid"] = None
        return None
    best = {}
    blank_rssi, blank_bssid = None, None
    for rec in aps:
        raw = rec[0]
        ssid = raw.decode() if isinstance(raw, bytes) else raw
        rssi = rec[3]
        bssid = rec[1] if len(rec) > 1 else None
        if not ssid:
            if blank_rssi is None or rssi > blank_rssi:
                blank_rssi, blank_bssid = rssi, bssid
            continue
        if ssid not in by:
            continue
        prev = best.get(ssid)
        if prev is None or rssi > prev[0]:
            best[ssid] = (rssi, bssid, by[ssid])
    wifi["blank_rssi"] = blank_rssi
    wifi["blank_bssid"] = blank_bssid
    found = []
    items = list(best.values())
    items.sort(key=lambda x: -x[0])
    for rssi, bssid, net in items:
        n = dict(net)
        n["bssid"] = bssid
        n["rssi"] = rssi
        found.append(n)
    return found

def _blank_home_ok():
    """True if a hidden home is configured and a strong empty-SSID AP was seen."""
    r = wifi.get("blank_rssi")
    if r is None or r < ROAM_MIN_RSSI:
        return False
    for n in wifi.get("nets") or []:
        if n.get("home") and n.get("hidden"):
            return True
    return False

def _hidden_home_target(now, arrival=False):
    """Hidden home when a strong empty-SSID AP was seen.
    Always bssid=None so the IDF probes by SSID (never pin to the
    strongest blank AP — that may be a neighbour). A stop after driving
    must not treat an unverified blank AP as home."""
    if arrival:
        return None
    if not _blank_home_ok():
        return None
    for n in wifi.get("nets") or []:
        if (n.get("home") and n.get("hidden")
                and n["ssid"] != wifi.get("ssid")
                and not _recently_failed(n["ssid"], now)):
            target = dict(n)
            target["bssid"] = None
            return target
    return None

def _candidate_order(w):
    """Visible strongest first (deduped), then unscanned nets in config order."""
    now = time.ticks_ms()
    scanned = _scan_known(w)
    if scanned is None:
        cands = [dict(n) for n in wifi["nets"]]
    else:
        seen = set(n["ssid"] for n in scanned)
        cands = scanned + [dict(n) for n in wifi["nets"]
                           if n["ssid"] not in seen]
    good, bad = [], []
    for n in cands:
        (bad if _recently_failed(n["ssid"], now) else good).append(n)

    def _home_first(rows):
        homes, rest = [], []
        blank = _blank_home_ok()
        for n in rows:
            rssi = n.get("rssi")
            if n.get("home") and rssi is not None and rssi >= ROAM_MIN_RSSI:
                homes.append(n)
            elif n.get("home") and n.get("hidden") and rssi is None and blank:
                homes.append(n)
            else:
                rest.append(n)
        return homes + rest

    return _home_first(good) + _home_first(bad)

def _wifi_clear(w):
    """Drop the last association so the next status() is not a stale reason."""
    try:
        w.disconnect()
    except OSError:
        pass
    try:
        w.active(False)
        w.active(True)
        _set_hostname(HOSTNAME)
    except OSError:
        pass
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < CLEAR_WAIT:
        feed()
        try:
            s = w.status()
        except OSError:
            break
        if s == network.STAT_IDLE or s == network.STAT_CONNECTING:
            break
        time.sleep_ms(50)

def _wifi_ip(w, net):
    """Apply this network's static IP, or give a previous one back to DHCP.

    Only a network that configured a static IP changes the interface, so the
    DHCP networks behave exactly as before."""
    cfg = net.get("ip")
    try:
        if cfg:
            w.ifconfig(cfg)
            wifi["static"] = True
        elif wifi.get("static"):
            w.ifconfig("dhcp")
            wifi["static"] = False
    except (OSError, ValueError, TypeError) as e:
        print("wifi ip", repr(e))

def _wifi_connect(w, net):
    wifi["ssid"] = net["ssid"]
    wifi["pw"] = net["pw"]
    wifi["home"] = bool(net.get("home"))
    wifi["state"] = "connecting"
    wifi["try_at"] = time.ticks_ms()
    wifi["saw_connecting"] = False
    print("wifi try", net["ssid"])
    try:
        wifi["prev_status"] = w.status()
    except OSError:
        wifi["prev_status"] = None
    _wifi_clear(w)
    _wifi_ip(w, net)
    try:
        bssid = net.get("bssid")
        if bssid:
            try:
                w.connect(net["ssid"], net["pw"], bssid=bssid)
            except TypeError:
                w.connect(net["ssid"], net["pw"])
        else:
            w.connect(net["ssid"], net["pw"])
    except OSError:
        _mark_failed(net["ssid"])
        return _wifi_next(w)
    wifi["tries"] += 1
    return True

def _net_home(ssid):
    for n in wifi.get("nets") or []:
        if n["ssid"] == ssid:
            return bool(n.get("home"))
    return False

def _net_hidden(ssid):
    for n in wifi.get("nets") or []:
        if n["ssid"] == ssid:
            return bool(n.get("hidden"))
    return False

def _wifi_got_ip(w):
    wifi["state"] = "connected"
    try:
        wifi["ip"] = w.ifconfig()[0]
    except OSError:
        wifi["ip"] = None
    try:
        wifi["rssi"] = w.status("rssi")
    except (OSError, ValueError):
        wifi["rssi"] = None
    wifi["home"] = _net_home(wifi.get("ssid"))
    wifi["failed"].pop(wifi.get("ssid"), None)
    wifi["backoff"] = WIFI_RETRY
    wifi["roam_at"] = time.ticks_ms()
    wifi["roaming"] = False
    if wifi["home"]:
        wifi["roam_backoff"] = ROAM_CHECK
    print("wifi ok", wifi.get("ssid") or "")
    return True

def _wifi_next(w):
    i = wifi.get("cand_i", 0) + 1
    wifi["cand_i"] = i
    cands = wifi.get("cand") or []
    if i < len(cands):
        return _wifi_connect(w, cands[i])
    wifi["state"] = "not found"
    wifi["home"] = False
    wifi["backoff"] = min(max(wifi["backoff"] * 2, WIFI_RETRY), WIFI_BACKOFF_MAX)
    wifi["last_check"] = time.ticks_ms()
    return False

def _wifi_begin_round(w):
    wifi["cand"] = _candidate_order(w)
    wifi["cand_i"] = 0
    if not wifi["cand"]:
        wifi["state"] = "not found"
        wifi["home"] = False
        wifi["backoff"] = min(max(wifi["backoff"] * 2, WIFI_RETRY), WIFI_BACKOFF_MAX)
        wifi["last_check"] = time.ticks_ms()
        return False
    return _wifi_connect(w, wifi["cand"][0])

def _wifi_poll(w):
    """Advance a connecting attempt. Never sleeps. try_at is this attempt."""
    now = time.ticks_ms()
    started = wifi.get("try_at")
    if started is None:
        wifi["try_at"] = now
        started = now
    elapsed = time.ticks_diff(now, started)
    try:
        s = w.status()
    except OSError:
        return _wifi_next(w)
    if s == network.STAT_GOT_IP:
        return _wifi_got_ip(w)
    if s == network.STAT_CONNECTING or s == network.STAT_IDLE:
        wifi["saw_connecting"] = wifi["saw_connecting"] or (s == network.STAT_CONNECTING)
        if elapsed >= CONNECT_TIMEOUT:
            _mark_failed(wifi.get("ssid"))
            return _wifi_next(w)
        return False
    # v1.29 connect() may leave the previous SSID's fail reason. Do not
    # attribute it to this attempt until CONNECTING was seen, or the
    # whole CONNECT_TIMEOUT elapses with no GOT_IP.
    leftover = (s == wifi.get("prev_status") or not wifi["saw_connecting"])
    if leftover and elapsed < CONNECT_TIMEOUT:
        return False
    _mark_failed(wifi.get("ssid"))
    return _wifi_next(w)

def _try_roam_home(w, arrival=False):
    """If a home AP is visible and strong enough, switch; else stay put.
    Runs between upload batches (wifi_tick never runs during POST).
    arrival=True is a stop after driving: only a visible home SSID may
    reset backoff and trigger a switch (not an unverified blank AP)."""
    feed()
    read_uart()
    found = _scan_known(w)
    feed()
    now = time.ticks_ms()
    target = None
    for n in found or ():
        if (n.get("home") and n.get("rssi", -100) >= ROAM_MIN_RSSI
                and n["ssid"] != wifi.get("ssid")
                and not _recently_failed(n["ssid"], now)):
            target = n
            break
    if target is None:
        target = _hidden_home_target(now, arrival)
    if target is None:
        if not arrival:
            wifi["roam_backoff"] = ROAM_CHECK
        return
    if arrival:
        wifi["roam_backoff"] = ROAM_CHECK
        wifi["roam_at"] = now
    cur = None
    for x in wifi.get("nets") or []:
        if x["ssid"] == wifi.get("ssid"):
            cur = dict(x)
            break
    wifi["cand"] = [target] + ([cur] if cur else [])
    wifi["cand_i"] = 0
    wifi["roaming"] = True
    print("wifi roam", target["ssid"])
    _wifi_connect(w, target)

def wifi_start():
    global DEVICE_ID, HOSTNAME
    env = load_env(ENV_FILE)
    wifi["nets"] = _load_nets(env)
    if wifi["nets"]:
        wifi["ssid"] = wifi["nets"][0]["ssid"]
        wifi["pw"] = wifi["nets"][0]["pw"]
        wifi["home"] = wifi["nets"][0]["home"]
    up["url"] = env.get("GEO_URL", "").rstrip("/") or None
    up["key"] = env.get("API_KEY") or None
    DEVICE_ID = _safe_name(_plain(env.get("DEVICE_ID")), DEFAULT_DEVICE_ID)
    HOSTNAME = _safe_name(_plain(env.get("HOSTNAME"))
                          or _plain(env.get("DEVICE_ID")),
                          DEFAULT_HOSTNAME, host=True)
    if up["url"] and up["url"].startswith("http://"):
        print("WARNING: GEO_URL is http:// - API key and GPS data are unencrypted")
        print("WARNING: requests follows http redirects as GET; use https origin")
    if not wifi["nets"]:
        wifi["state"] = "no .env"
        return
    _oled_banner("WiFi scanning...")
    try:
        w = wifi["wlan"] = network.WLAN(network.STA_IF)
        _set_hostname(HOSTNAME)
        w.active(True)
    except OSError:
        wifi["state"] = "error"
        return
    _wifi_begin_round(w)
    if wifi["state"] == "connecting":
        _wifi_poll(w)
    wifi["last_check"] = time.ticks_ms()

def wifi_tick():
    w = wifi["wlan"]
    if w is None:
        return
    now = time.ticks_ms()
    try:
        s = w.status()
    except OSError:
        s = None
    if s == network.STAT_GOT_IP:
        if wifi["state"] != "connected":
            _wifi_got_ip(w)
        else:
            if _age(now, wifi["last_check"]) >= WIFI_CHECK:
                try:
                    wifi["rssi"] = w.status("rssi")
                except (OSError, ValueError):
                    wifi["rssi"] = None
                wifi["last_check"] = now
            interval = wifi.get("roam_backoff") or ROAM_CHECK
            if (not wifi.get("home")
                    and any(n.get("home") for n in wifi.get("nets") or [])):
                spd = st.get("speed")
                moving = has_fix(now) and spd is not None and spd >= 5
                if moving:
                    wifi["moved_at"] = now
                    arrival = False
                elif (wifi.get("moved_at") is not None
                      and _age(now, wifi["moved_at"]) >= 60000):
                    wifi["moved_at"] = None
                    arrival = True
                else:
                    arrival = False
                due = (not moving
                       and _age(now, wifi.get("roam_at") or now) >= interval)
                if due or arrival:
                    wifi["roam_at"] = now
                    _try_roam_home(w, arrival=arrival and not due)
        return
    if wifi["state"] == "connected":
        # v1.29 reports STAT_CONNECTING on beacon timeout while the IDF
        # auto-reconnects. Do not map that to "connecting" with a stale
        # try_at (that would immediately fail the current SSID).
        wifi["state"] = "lost"
        wifi["home"] = False
        wifi["ip"] = wifi["rssi"] = None
        wifi["try_at"] = now
        wifi["cand"] = []
        wifi["cand_i"] = 0
        print("wifi lost", wifi.get("ssid") or "")
        return
    if wifi["state"] == "lost":
        if (s == network.STAT_CONNECTING or s == network.STAT_IDLE
                or s is None):
            if _age(now, wifi["try_at"] or now) < LOST_GRACE:
                return
        wifi["state"] = "offline"
        wifi["last_check"] = time.ticks_add(now, -wifi["backoff"])
        # fall through to a fresh round
    if wifi["state"] == "connecting":
        _wifi_poll(w)
        return
    if _age(now, wifi["last_check"]) < wifi["backoff"]:
        return
    wifi["last_check"] = now
    _wifi_begin_round(w)

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
    wifi["home"] = False

def online():
    return wifi["state"] == "connected"

def at_home():
    """Whether the connected network is configured as a home network."""
    return wifi["state"] == "connected" and wifi.get("home")

# --- GPX logger -------------------------------------------------------------
# Format (.trk): one point per line, "lat,lon,ele,hdop,YYYYMMDDTHHMMSSZ"
# (~46 bytes; the old GPX XML was ~112). ele/hdop are empty when unknown.
# "-" starts a new segment. The file is always complete: each batch
# overwrites the footer with new points plus the footer again. LittleFS
# commits on close, so a power cut leaves either the previous or the new
# version. A line cut short has no newline and is skipped on read.
# Older .gpx files (XML) are still read and uploaded.

HEADER = "#TRK1\n"
FOOTER = "#END\n"
NEW_SEG = "-\n"

log = {}

def log_reset():
    log.clear()
    log.update(file=None, created=False, batch=[], points=0, err=0,
               fix=False, lost_at=None, new_seg=False, full=False,
               home_still_since=None, home_moving_since=None,
               home_move_origin=None, home_motion_active=False,
               home_paused=False, home_pending=None,
               last_sample=None, last_flush=time.ticks_ms(), last_t=None,
               prev=None, full_check=None, file_at=None, good_size=None)

def free():
    try:
        s = os.statvfs(LOG_DIR)
        return s[0] * s[3]
    except OSError:
        return None

def has_fix(now):
    return (st["valid"] and st["date"] and st["time"] and st["lat"] is not None
            and st["pos_at"] is not None
            and _age(now, st["pos_at"]) < STALE)

def _stamp():
    """YYYYMMDD_HHMMSS of the current fix (track file names)."""
    return st["date"].replace("-", "") + "_" + st["time"].replace(":", "")

def start_file(stamp=None):
    try:
        os.mkdir(LOG_DIR)
    except OSError:
        pass                             # already exists
    name = stamp or _stamp()
    log.update(file=LOG_DIR + "/" + name + ".trk", created=False, points=0,
               new_seg=False, file_at=time.ticks_ms(), good_size=None)
    # First write happens when there are points (parking must not create
    # an empty / one-point file just because a fix came back).

def flush():
    """Write pending points to the current file, opened only for the write.
    LittleFS commits on close: drop the RAM batch only after a successful
    close, or if the file size shows the new bytes actually landed."""
    log["last_flush"] = time.ticks_ms()
    path, batch = log["file"], log["batch"]
    if path is None or (log["created"] and not batch):
        return
    room = free()
    if room is not None and room < MIN_FREE:
        log["full"] = True
        del batch[:]
        return
    creating = not log["created"]
    old = log.get("good_size")
    if not creating:
        sz0 = _file_size(path)
        if old is None or (sz0 is not None and old > sz0):
            old = sz0
    f = None
    wrote = False
    close_ok = False
    expected = None
    try:
        extra = 0 if creating else (len(NEW_SEG) if log["new_seg"] else 0)
        pts = 0
        i = 0
        while i < len(batch):
            pts += len(batch[i])
            i += 1
        if creating:
            expected = len(HEADER) + pts + len(FOOTER)
            f = open(path, "wb")
            f.write(HEADER.encode())
        else:
            f = open(path, "r+b")
            _seek_footer(f, old)
            try:
                start = f.tell()
            except (OSError, AttributeError):
                start = (old or 0) - len(FOOTER)
                if start < 0:
                    start = 0
            expected = start + extra + pts + len(FOOTER)
            if log["new_seg"]:
                f.write(NEW_SEG.encode())
        i = 0
        while i < len(batch):
            f.write(batch[i].encode())
            i += 1
        f.write(FOOTER.encode())
        wrote = True
    except (OSError, MemoryError):
        log["err"] += 1
    if f is not None:
        try:
            f.close()
            close_ok = True
        except OSError:
            log["err"] += 1
    landed = wrote and close_ok
    if not landed:
        sz, gone = _stat_size(path)
        if expected is not None and sz == expected:
            landed = True
        else:
            if creating:
                log["created"] = False
                log["good_size"] = None
            elif gone or sz == 0:
                # Only rewrite if the file is definitely missing or empty.
                # A transient stat failure (sz is None) must not flip
                # created=False — the next wb would wipe the trip.
                log["created"] = False
                log["good_size"] = None
            return
    log["created"] = True
    log["good_size"] = expected
    log["points"] += len(batch)
    up["left"] += len(batch)
    del batch[:]
    log["new_seg"] = False

def _dist(lat1, lon1, lat2, lon2):
    """Metres between two nearby positions (equirectangular approximation)."""
    x = math.radians(lon2 - lon1) * math.cos(math.radians((lat1 + lat2) / 2))
    y = math.radians(lat2 - lat1)
    return 6371000 * math.sqrt(x * x + y * y)

def _fresh_hdop(now):
    """HDOP from a GGA in this epoch (not the previous fix)."""
    hdop, at = st.get("hdop"), st.get("hdop_at")
    if hdop is None or at is None:
        return None
    if _age(now, at) >= STALE:
        return None
    return hdop

def _home_pending_add(now):
    """Keep the samples of a paused track that is confirming movement, so a
    resume starts where the car started moving, not 30 s / 60 m later."""
    if (log["last_sample"] is not None
            and _age(now, log["last_sample"]) < SAMPLE):
        return
    log["last_sample"] = now
    pend = log["home_pending"]
    if pend is None:
        pend = log["home_pending"] = {"lines": [], "prev": None, "t": None,
                                      "stamp": None}
    s = _sample_line(now, pend["prev"], pend["t"])
    if s is None:
        return
    if not pend["lines"]:
        pend["stamp"] = _stamp()
    pend["t"], pend["prev"] = s[0], (st["lat"], st["lon"])
    pend["lines"].append(s[1])
    if len(pend["lines"]) > HOME_PENDING_MAX:
        pend["lines"].pop(0)

def _home_resume(now):
    """Leave the pause; the confirmation samples open the new track."""
    pend = log["home_pending"]
    log.update(home_paused=False, home_pending=None, lost_at=None)
    if pend and pend["lines"]:
        start_file(pend["stamp"])
        batch = log["batch"]
        batch.extend(pend["lines"])
        if len(batch) > MAX_BATCH:
            del batch[:len(batch) - MAX_BATCH]
        log.update(fix=True, prev=pend["prev"], last_t=pend["t"],
                   last_sample=now)
    else:
        log.update(fix=False, prev=None, last_t=None, last_sample=None)

def _home_stop_tick(now):
    """Pause after sustained low speed; require sustained motion to resume."""
    if not at_home():
        if log["home_paused"]:
            _home_resume(now)            # driving away: keep what was seen
        log.update(home_still_since=None, home_moving_since=None,
                   home_move_origin=None, home_motion_active=False,
                   home_pending=None)
        return False

    speed = st.get("speed")
    if not has_fix(now) or speed is None:
        log.update(home_moving_since=None, home_move_origin=None,
                   home_pending=None)
        return True if log["home_paused"] else False

    if speed >= HOME_STOP_SPEED:
        if log["home_motion_active"]:
            return False
        # Confirm real movement before the stop timer is cleared. Only a
        # paused track waits for it; an active one keeps recording, so the
        # arrival and every slow-down near home are not lost.
        paused = log["home_paused"]
        if log["home_moving_since"] is None:
            log["home_moving_since"] = now
            log["home_move_origin"] = (st["lat"], st["lon"])
        if paused:
            _home_pending_add(now)
        origin = log["home_move_origin"]
        moved = (0 if origin is None
                 else _dist(origin[0], origin[1], st["lat"], st["lon"]))
        elapsed = _age(now, log["home_moving_since"])
        # A few seconds of real driving restart the stop timer, so a stop
        # right after it does not pause on the time spent parked before.
        # A one-sample speed spike does neither.
        if (not paused and elapsed >= HOME_STILL_RESET
                and moved >= HOME_STILL_RESET_DISTANCE):
            log["home_still_since"] = None
        if moved >= HOME_MOVE_DISTANCE and elapsed >= HOME_MOVE_CONFIRM:
            log.update(home_motion_active=True, home_still_since=None,
                       home_moving_since=None, home_move_origin=None)
            if paused:
                _home_resume(now)
            return False
        if elapsed >= HOME_MOVE_MAX_WAIT:
            log.update(home_moving_since=now, home_pending=None,
                       home_move_origin=(st["lat"], st["lon"]))
        return paused

    log.update(home_moving_since=None, home_move_origin=None,
               home_motion_active=False, home_pending=None)
    if log["home_paused"]:
        return True
    if log["home_still_since"] is None:
        log["home_still_since"] = now
    if _age(now, log["home_still_since"]) < HOME_STOP_DELAY:
        return False

    if log["batch"]:
        flush()
    if log["batch"]:
        return True
    log.update(file=None, created=False, file_at=None, fix=False,
               new_seg=False, last_t=None, prev=None, home_paused=True)
    return True

def _sample_line(now, prev, last_t):
    """(time, track line) for the current fix, or None: HDOP too dilute
    (try again next SAMPLE), same second as last_t, or within MIN_MOVE of
    prev."""
    hdop = _fresh_hdop(now)
    if hdop is not None and hdop > MAX_HDOP:
        return None
    t = st["date"] + "T" + st["time"] + "Z"
    if t == last_t or (prev is not None and _dist(
            prev[0], prev[1], st["lat"], st["lon"]) <= MIN_MOVE):
        return None
    ele = ""
    if st["alt"] is not None and st.get("fixtype") != 2:
        ele = "%.1f" % st["alt"]
    hd = ("%.1f" % hdop) if hdop is not None else ""
    tc = st["date"].replace("-", "") + "T" + st["time"].replace(":", "") + "Z"
    return t, "%.6f,%.6f,%s,%s,%s\n" % (st["lat"], st["lon"], ele, hd, tc)

def log_tick():
    """Sample a position every SAMPLE ms and record it if it is more than
    MIN_MOVE m from the last recorded point; write a batch every FLUSH ms."""
    now = time.ticks_ms()
    if _home_stop_tick(now):
        return
    # Close a long-open file on any WiFi network so it can upload while
    # logging continues. Offline driving keeps one file.
    if (log["file"] is not None and log.get("file_at") is not None
            and wifi["state"] == "connected"
            and log["points"] + len(log["batch"]) > 0
            and _age(now, log["file_at"]) >= NEW_FILE_GAP * 1000
            and _age(now, log["last_flush"]) >= FLUSH):
        flush()
        if not log["batch"]:
            log.update(file=None, created=False, file_at=None)
    if log["full"]:
        last = log.get("full_check")
        if last is not None and _age(now, last) < FULL_CHECK:
            return
        log["full_check"] = now
        room = free()
        if room is None or room < MIN_FREE:
            return
        log["full"] = False
    if not has_fix(now):
        if log["fix"]:
            log["fix"], log["lost_at"] = False, now
            flush()
        return
    if not log["fix"]:
        log["fix"] = True
        if (log["file"] is not None and log.get("lost_at") is not None
                and _age(now, log["lost_at"]) > NEW_FILE_GAP * 1000):
            if _age(now, log["last_flush"]) >= FLUSH:
                flush()
            if not log["batch"]:
                log.update(file=None, created=False, file_at=None)
                log["prev"] = None       # first point of the new trip is kept
                log["new_seg"] = False
            else:
                log["new_seg"] = True    # write failed: new trip in same file
                log["prev"] = None
        elif log["file"] is not None:
            log["new_seg"] = True
            log["prev"] = None           # start the segment with this fix
    if (log["last_sample"] is None
            or _age(now, log["last_sample"]) >= SAMPLE):
        log["last_sample"] = now
        s = _sample_line(now, log["prev"], log["last_t"])
        if s is not None:
            if log["file"] is None:
                start_file()
            log["last_t"], log["prev"] = s[0], (st["lat"], st["lon"])
            log["batch"].append(s[1])
            if len(log["batch"]) > MAX_BATCH:
                log["batch"].pop(0)
    if _age(now, log["last_flush"]) >= FLUSH:
        flush()

# --- Upload ---------------------------------------------------------------
# Finished track files (never the one being recorded) are sent to Dawarich's
# Overland endpoint, oldest first, UPLOAD_BATCH points per POST. Progress is
# a <file>.pos sidecar (byte offset). LittleFS lfs_rename replaces the
# destination atomically; FAT vfs_fat does remove+rename (not atomic: a crash
# leaves no .pos, so the next boot resends from 0; Dawarich deduplicates via
# index_points_on_user_id_timestamp_lonlat / archival_safe_upsert_all).
#
# Offset advances only after a 2xx or a durable write to <file>.bad.
# If .bad is full, uploads of that file pause (state "bad full"). Three
# consecutive payload errors without a 2xx are treated as a server/config
# problem: backoff, no further quarantine. .bad retries that still 4xx
# never delete the quarantined data.
#
# TLS (mbedTLS) needs ~40 kB contiguous from the ESP-IDF heap, outside the
# Python heap. If the Python heap ever grows (see gc.threshold at the top) it
# takes that block, so allocations here stay small: the request body goes
# into a buffer allocated once at import. For the same reason the module must
# be deployed precompiled (gps_logger.mpy): compiling the source on the
# device grows the heap for good.

_body = bytearray(BODY_CAP)

up = {}

def _rm(path):
    try:
        os.remove(path)
    except OSError:
        pass

def _enoent(e):
    n = getattr(e, "errno", None)
    if n is None:
        try:
            n = e.args[0]
        except (AttributeError, IndexError):
            return False
    return n == 2

def _stat_size(path):
    """(size_or_None, missing). missing is True only on ENOENT."""
    try:
        return os.stat(path)[6], False
    except OSError as e:
        return None, _enoent(e)

def _file_size(path):
    try:
        return os.stat(path)[6]
    except OSError:
        return None

def _seek_footer(f, old):
    """Seek to the last known FOOTER, or after the last newline if truncated.

    A partial append leaves extra bytes after good_size; overwrite those
    from the last good footer. An externally truncated file is shorter
    and does not end with FOOTER — resume after the last newline."""
    n = len(FOOTER)
    foot = FOOTER.encode()
    if old is not None and old >= n:
        pos = old - n
        f.seek(pos, 0)
        if f.read(n) == foot:
            f.seek(pos, 0)
            return
        f.seek(0, 2)
        if f.tell() > old:
            f.seek(pos, 0)
            return
    else:
        f.seek(-n, 2)
        pos = f.tell()
        if f.read(n) == foot:
            f.seek(pos, 0)
            return
    f.seek(0, 2)
    end = f.tell()
    win = end if end < LINE_MAX else LINE_MAX
    if win <= 0:
        f.seek(0, 0)
        return
    start = end - win
    f.seek(start, 0)
    chunk = f.read(win)
    i = chunk.rfind(b"\n")
    if i < 0:
        f.seek(0, 0)
        return
    f.seek(start + i + 1, 0)

def up_reset():
    up.clear()
    up.update(url=None, key=None, state="idle", sent=0, err=0, files=0,
              left=0, fail4=0, backoff=BACKOFF_START, allow_bad=True,
              bad_try={}, skip=None, hold={}, bad_full={},
              next_at=time.ticks_ms())
    try:
        names = os.listdir(LOG_DIR)
    except OSError:
        names = []
    have = set(names)
    for n in names:
        if n.endswith(".tmp"):
            _rm(LOG_DIR + "/" + n)
        elif n.endswith(".pos") and n[:-4] not in have:
            _rm(LOG_DIR + "/" + n)       # orphan .pos or .bad.pos
    _oled_banner("Counting points")
    try:
        up["left"] = count_points()
    except MemoryError:
        gc.collect()
        up["left"] = 0
        print("count MemoryError")

def read_offset(path):
    """Byte offset to resume from. Corrupt / past-EOF sidecars reset to 0
    (Dawarich dedup handles the resend). Never treat a bad .pos as EOF."""
    size = _file_size(path)
    if size is None:
        return 0
    try:
        with open(path + ".pos") as f:
            off = int(f.read().strip() or 0)
    except (OSError, ValueError):
        return 0
    if off < 0 or off > size:
        print("pos corrupt", path, off, size)
        return 0
    if off == 0:
        return 0
    try:
        with open(path, "rb") as f:
            f.seek(off - 1)
            b = f.read(1)
        if b != b"\n":
            print("pos misaligned", path, off)
            return 0
    except OSError:
        return 0
    return off

def write_offset(path, off):
    """Sidecar update: write <file>.pos.tmp, then rename over .pos.
    Atomic on LittleFS; on FAT a crash may drop the .pos (resend from 0)."""
    tmp = path + ".pos.tmp"
    with open(tmp, "w") as f:
        f.write("%d\n" % off)
    os.rename(tmp, path + ".pos")

def _readline(f):
    """One track line, at most LINE_MAX bytes. Over-long lines are drained to
    the next newline and returned as b'' (skip). None means true EOF."""
    try:
        line = f.readline(LINE_MAX)
    except MemoryError:
        gc.collect()
        raise
    if not line:
        return None
    if len(line) == LINE_MAX and not line.endswith(b"\n"):
        while True:
            try:
                chunk = f.readline(LINE_MAX)
            except MemoryError:
                gc.collect()
                break
            if not chunk or chunk.endswith(b"\n"):
                break
        return b""
    return line

def count_points():
    """Points still to upload in all track files, the one being recorded
    included. Counts point lines from each file's sidecar offset. Runs at
    boot; flush() and commit_offset() keep up["left"] current in between.
    Feeds the WDT every 64 lines so a full flash cannot starve it."""
    try:
        names = [n for n in os.listdir(LOG_DIR)
                 if n.endswith((".gpx", ".trk"))]
    except OSError:
        return 0
    total = 0
    nlines = 0
    for n in names:
        path = LOG_DIR + "/" + n
        gpx = not _is_trk(path)
        try:
            off = read_offset(path)
            with open(path, "rb") as f:
                if off:
                    f.seek(off)
                while True:
                    line = _readline(f)
                    if line is None:
                        break
                    nlines += 1
                    if nlines & 63 == 0:
                        feed()
                    if _is_point(line, gpx):
                        total += 1
        except (OSError, MemoryError) as e:
            print("count", path, repr(e))
            gc.collect()
    return total

def pending():
    """Finished track files, oldest first (names are YYYYMMDD_HHMMSS).
    Sidecars (.pos) and quarantined (.bad) files are not listed."""
    try:
        names = sorted(n for n in os.listdir(LOG_DIR)
                       if n.endswith((".gpx", ".trk")))
    except OSError:
        return []
    return [p for p in (LOG_DIR + "/" + n for n in names) if p != log["file"]]

def pending_bad():
    """Rejected-batch files left for later recovery."""
    try:
        names = sorted(n for n in os.listdir(LOG_DIR)
                       if n.endswith((".gpx.bad", ".trk.bad")))
    except OSError:
        return []
    return [LOG_DIR + "/" + n for n in names]

def _attr(line, name):
    i = line.find(name + '="')
    if i < 0:
        return None
    i += len(name) + 2
    j = line.find('"', i)
    if j < 0:
        return None
    return line[i:j]

def _tag(line, name):
    i = line.find("<" + name + ">")
    if i < 0:
        return None
    i += len(name) + 2
    j = line.find("</" + name + ">", i)
    if j < 0:
        return None
    return line[i:j]

def _float(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None

def _finite(x):
    try:
        return math.isfinite(x)
    except AttributeError:
        return x == x and x != float("inf") and x != float("-inf")

def _valid_time(t):
    """ISO UTC `YYYY-MM-DDTHH:MM:SSZ` with year 2020-2099."""
    if not t or len(t) != 20:
        return False
    if (t[4] != "-" or t[7] != "-" or t[10] != "T"
            or t[13] != ":" or t[16] != ":" or t[19] != "Z"):
        return False
    for i in (0, 1, 2, 3, 5, 6, 8, 9, 11, 12, 14, 15, 17, 18):
        if t[i] < "0" or t[i] > "9":
            return False
    year = int(t[0:4])
    return 2020 <= year <= 2099

def _put(pos, b):
    _body[pos:pos + len(b)] = b
    return pos + len(b)

def _is_trk(path):
    return path.endswith(".trk") or path.endswith(".trk.bad")

def _is_point(line, gpx):
    """One point line: <trkpt> in old .gpx, "lat,lon,ele,hdop,time" in .trk.
    A line without its newline (power cut mid-write) is not a point."""
    if gpx:
        return line.startswith(b"<trkpt")
    return (line[-1:] == b"\n" and line.count(b",") == 4
            and line[:1] != b"#")

def _point(line, gpx):
    """(lat, lon, time, ele, hdop) as strings, time as ISO UTC, or None."""
    if gpx:
        s = line.decode()
        return (_attr(s, "lat"), _attr(s, "lon"), _tag(s, "time"),
                _tag(s, "ele"), _tag(s, "hdop"))
    f = line[:-1].decode().split(",")
    tc = f[4]
    if len(tc) != 16 or tc[8] != "T" or tc[15] != "Z":
        return None
    t = "%s-%s-%sT%s:%s:%sZ" % (tc[0:4], tc[4:6], tc[6:8],
                                tc[9:11], tc[11:13], tc[13:15])
    return (f[0], f[1], t, f[2], f[3])

def read_batch(path, off=0):
    """Fill _body with up to UPLOAD_BATCH valid points from byte offset off
    as Overland JSON -> (n, n_ok, body_len, new_off, eof).
    Truncated / non-numeric / inf / over-long points are skipped.
    A valid point that does not fit is left unread (seek back, eof=False)."""
    gpx = not _is_trk(path)
    n, n_ok, pos, sep = 0, 0, _put(0, b'{"locations":['), b""
    nlines, eof = 0, False
    with open(path, "rb") as f:
        if off:
            f.seek(off)
        while True:
            line_start = f.tell()
            line = _readline(f)
            if line is None:
                eof = True
                break
            nlines += 1
            if nlines & 63 == 0:
                feed()
            if not line or not _is_point(line, gpx):
                continue
            n += 1
            try:
                p = _point(line, gpx)
            except UnicodeError:
                p = None
            lat_f = lon_f = None
            if p is not None:
                lat_f, lon_f = _float(p[0]), _float(p[1])
            ok = (p is not None and lat_f is not None and lon_f is not None
                  and _finite(lat_f) and _finite(lon_f)
                  and abs(lat_f) <= 90 and abs(lon_f) <= 180
                  and _valid_time(p[2]))
            loc = None
            if ok:
                ele_f = _float(p[3]) if p[3] else None
                if ele_f is not None and not _finite(ele_f):
                    ele_f = None
                hdop_f = _float(p[4]) if p[4] else None
                extra = ""
                if ele_f is not None:
                    extra += '"altitude":%.1f,' % ele_f
                if hdop_f is not None and _finite(hdop_f):
                    extra += '"horizontal_accuracy":%.1f,' % (hdop_f * HDOP_M)
                loc = ('{"type":"Feature","geometry":{"type":"Point",'
                       '"coordinates":[%.6f,%.6f]},"properties":{'
                       '"timestamp":"%s",%s"device_id":"%s"}}'
                       % (lon_f, lat_f, p[2], extra, DEVICE_ID)).encode()
                need = len(sep) + len(loc)
                if pos + need + 2 > len(_body):
                    if n_ok == 0:
                        print("point too large", len(loc))
                        loc = None
                    else:
                        f.seek(line_start)
                        n -= 1
                        eof = False
                        break
            if loc:
                pos = _put(_put(pos, sep), loc)
                sep = b","
                n_ok += 1
            if n >= UPLOAD_BATCH:
                break
        new_off = f.tell()
    if not sep:
        return n, 0, 0, new_off, eof
    return n, n_ok, _put(pos, b"]}"), new_off, eof

def commit_offset(path, n, new_off, eof=False):
    """Record that n points are done. Delete only at true EOF.
    Crash between POST and this call resends (at-least-once)."""
    write_offset(path, new_off)
    up["left"] = max(up["left"] - n, 0)
    if not eof:
        return
    _rm(path)
    _rm(path + ".pos")
    _clear_bad_full(path)

def _append_bad(path, off, new_off):
    """Copy consumed point lines to <path>.bad. False if not durable."""
    if path.endswith(".bad"):
        return False
    bad = path + ".bad"
    sz = _file_size(bad) or 0
    need = max(new_off - off, 0)
    if sz >= MAX_BAD_BYTES or sz + need > MAX_BAD_BYTES:
        print("bad file full", bad)
        return False
    gpx = not _is_trk(path)
    try:
        with open(path, "rb") as src, open(bad, "ab") as dst:
            if off:
                src.seek(off)
            while src.tell() < new_off:
                line = _readline(src)
                if line is None:
                    break
                if _is_point(line, gpx):
                    dst.write(line)
    except (OSError, MemoryError) as e:
        print("quarantine", repr(e))
        gc.collect()
        return False
    return True

def _quarantine_batch(path, off, n, new_off, eof):
    """Park one rejected batch only if the copy landed on flash."""
    if path.endswith(".bad"):
        up["state"] = "err 422"
        return False
    if not _append_bad(path, off, new_off):
        up["state"] = "bad full"
        bf = up.get("bad_full") or {}
        bf[path] = True
        up["bad_full"] = bf
        return False
    commit_offset(path, n, new_off, eof)
    up["allow_bad"] = False
    up["state"] = "quarantine"
    return True

def _retry_after_ms(r):
    hdrs = getattr(r, "headers", None) or {}
    v = None
    if isinstance(hdrs, dict):
        for k in hdrs:
            if str(k).lower() == "retry-after":
                v = hdrs[k]
                break
    if v is None:
        return None
    try:
        ms = int(v) * 1000
    except (TypeError, ValueError):
        return None
    return min(max(ms, BACKOFF_START), BACKOFF_MAX)

def _schedule(now, ms=None):
    if ms is None:
        b = up.get("backoff", BACKOFF_START) or BACKOFF_START
        ms = b
        up["backoff"] = min(max(b * 2, BACKOFF_START), BACKOFF_MAX)
    up["next_at"] = time.ticks_add(now, ms)

def _hold_file(path, now, ms):
    """Backoff one file so a stuck oldest track does not block the rest."""
    h = up.get("hold") or {}
    h[path] = time.ticks_add(now, ms)
    up["hold"] = h

def _bad_full_left():
    """True while a .bad that hit MAX_BAD_BYTES still exists on flash."""
    bf = up.get("bad_full") or {}
    if not bf:
        return False
    keep = {}
    for p in bf:
        if _file_size(p) is not None:
            keep[p] = True
    up["bad_full"] = keep
    return bool(keep)

def _clear_bad_full(path):
    bf = up.get("bad_full") or {}
    if path in bf:
        bf.pop(path, None)
        up["bad_full"] = bf

def _http_post(size):
    """POST _body[:size]. Returns (status_code or None, retry_after_ms or None).
    Always closes the response so a non-OSError does not leak the TLS socket
    until the GC finalizer (micropython-lib requests)."""
    feed()
    r = None
    code, retry = None, None
    try:
        import requests
        r = requests.post("%s/api/v1/overland/batches" % up["url"],
                          data=memoryview(_body)[:size],
                          headers={"Content-Type": "application/json",
                                   "Authorization": "Bearer %s" % up["key"]},
                          timeout=HTTP_TIMEOUT)
        code = r.status_code
        retry = _retry_after_ms(r)
    except MemoryError:
        raise
    except Exception as e:
        print("upload", repr(e))
    finally:
        if r is not None:
            try:
                r.close()
            except Exception:
                pass
        feed()
        gc.collect()
    return code, retry

def upload_tick():
    """Send one batch when WiFi is up and due. Runs on home and hotspot."""
    now = time.ticks_ms()
    if (wifi["state"] != "connected" or up["state"] == "no .env"
            or _waiting(now, up["next_at"], UP_WAIT_MAX)):
        return
    if not up["url"] or not up["key"]:
        up["state"] = "no .env"
        return
    files = pending()
    up["files"] = len(files)
    skip = up.get("skip")
    if skip and skip in files and len(files) > 1:
        files = [p for p in files if p != skip]
    held = up.get("hold") or {}
    due = []
    for p in files:
        t = held.get(p)
        if t is None or not _waiting(now, t, UP_WAIT_MAX):
            due.append(p)
    files = due
    from_bad = False
    if not files and up.get("allow_bad"):
        files = pending_bad()
        from_bad = bool(files)
        files = [p for p in files
                 if (up.get("bad_try") or {}).get(p, 0) < MAX_BAD_RETRY]
        due = []
        for p in files:
            t = held.get(p)
            if t is None or not _waiting(now, t, UP_WAIT_MAX):
                due.append(p)
        files = due
    if not files:
        if _bad_full_left():
            up["state"] = "bad full"
        else:
            up["state"] = "idle"
        up["next_at"] = time.ticks_add(now, BACKOFF_START)
        return
    path = files[0]
    off = read_offset(path)
    try:
        n, n_ok, size, new_off, eof = read_batch(path, off)
        feed()
        fsize = _file_size(path) or 0
        if not n:
            if eof or fsize == 0:
                _rm(path)                # empty / EOF track and .bad
                _rm(path + ".pos")
                _clear_bad_full(path)
            else:
                print("pos skip", path, off, new_off, fsize)
                write_offset(path, 0)
            up["next_at"] = time.ticks_add(now, UPLOAD_GAP)
            return
        if not size:
            commit_offset(path, n, new_off, eof)
            up["next_at"] = time.ticks_add(now, UPLOAD_GAP)
            return
    except OSError:
        up["err"] += 1
        up["state"] = "fs err"
        _schedule(now)
        return
    except MemoryError:
        up["skip"] = path
        _low_mem(now)
        return
    if not _bad_full_left():
        up["state"] = "sending"
    read_uart()                          # drain NMEA before blocking
    gc.collect()
    try:
        code, retry = _http_post(size)
    except MemoryError:
        up["skip"] = path
        _low_mem(time.ticks_ms())
        return
    read_uart()                          # catch up after a multi-second POST
    now = time.ticks_ms()
    print("upload", path, n, n_ok, code, gc.mem_free())
    gap = UPLOAD_GAP_AWAY if not at_home() else UPLOAD_GAP
    if code is not None and 200 <= code < 300:
        try:
            commit_offset(path, n, new_off, eof)
        except OSError:
            up["err"] += 1
            up["state"] = "fs err"
            _schedule(now)
            return
        up["sent"] += n_ok
        up["fail4"] = 0
        up["backoff"] = BACKOFF_START
        up["allow_bad"] = True
        up["skip"] = None
        held = up.get("hold") or {}
        held.pop(path, None)
        up["hold"] = held
        _clear_bad_full(path)
        if from_bad:
            bt = up.get("bad_try") or {}
            bt[path] = bt.get(path, 0) + 1
            up["bad_try"] = bt
        if not _bad_full_left():
            up["state"] = "ok"
        up["next_at"] = time.ticks_add(now, gap)
    elif code in PAYLOAD_ERR:
        up["err"] += 1
        up["fail4"] = up.get("fail4", 0) + 1
        if path.endswith(".bad") or up["fail4"] >= MAX_GLOBAL_PAYLOAD:
            up["state"] = "err payload"
            _hold_file(path, now, retry if retry else BACKOFF_MAX)
            _schedule(now, gap)
            return
        if not _quarantine_batch(path, off, n, new_off, eof):
            _hold_file(path, now, BACKOFF_MAX)
            _schedule(now, gap)
            return
        up["next_at"] = time.ticks_add(now, gap)
    elif code == 401:
        up["err"] += 1
        up["state"] = "err 401"
        _schedule(now, retry if retry else AUTH_BACKOFF)
    else:
        up["err"] += 1
        up["state"] = "err " + ("net" if code is None else str(code))
        _schedule(now, retry)

def _low_mem(now):
    gc.collect()
    up["err"] += 1
    up["state"] = "low mem"
    _schedule(now)
    print("upload MemoryError, free", gc.mem_free())

# --- Screen -----------------------------------------------------------------

def status():
    if st["rx"] is None or _age(time.ticks_ms(), st["rx"]) > NO_DATA:
        return "No data!"
    if not st["valid"]:
        return "Waiting fix"
    s = {2: "2D fix", 3: "3D fix"}.get(st["fixtype"], "Fix")
    return s + " DGPS" if st["quality"] == 2 else s

def _fmt(fmt, v):
    return fmt % v if v is not None else "--"

def _short(n):
    """Fit a count into the 4 chars left after "Points left "."""
    return str(n) if n < 10000 else "%dk" % (n // 1000)

def _wifi_line():
    if wifi["state"] == "connected" and wifi.get("ssid"):
        ssid = wifi["ssid"]
        if len(ssid) > 11:
            ssid = ssid[:11]
        return "WiFi " + ssid
    return "WiFi " + wifi["state"]

def _mode_line():
    if _bad_full_left() or up.get("state") == "bad full":
        return "Mode BAD FULL"
    if wifi["state"] == "connected":
        return "Mode LOG+UP"
    return "Mode LOG"

def _oled_ok(fn):
    global oled, oled_err
    if oled is None:
        return
    try:
        fn()
        oled_err = 0
    except OSError as e:
        oled_err += 1
        print("oled", repr(e))
        if oled_err >= OLED_MAX_ERR:
            oled = None

def _oled_banner(msg):
    def _do():
        oled.fill(0)
        oled.text(msg, 0, 28)
        oled.show()
    _oled_ok(_do)

def draw():
    def _do():
        oled.fill(0)
        oled.fill_rect(0, 0, 128, 8, 1)
        oled.text("GPS Logger", 0, 0, 0)
        oled.text("|/-\\"[st["spin"] % 4], 120, 0, 0)
        room = free()
        for row, line in enumerate((
                _wifi_line(),
                "GPS " + status(),
                _mode_line(),
                "Flash " + _fmt("%d kB", None if room is None else room // 1024),
                "Points left " + _short(up["left"]))):
            oled.text(line, 0, 10 + row * 11)
        oled.show()
    _oled_ok(_do)

# --- Main loop --------------------------------------------------------------

def run():
    """Show the logger state on the OLED, logging and uploading when possible."""
    global _wdt, _running, _boot_ok_done
    _keep_stop()
    _running = False
    _boot_ok_done = False
    try:
        # Arm the dog before any blocking init (count_points, WiFi scan).
        # GPIO25 boot-bypass never calls run(), so the REPL stays free.
        if _wdt is None:
            _wdt = WDT(timeout=WDT_TIMEOUT)
        feed()
        reset()
        log_reset()
        wifi_reset()
        up_reset()
        wifi_start()
        _running = True
        _loop()
    except KeyboardInterrupt:
        _running = False
        print("WDT armed - fed by timer; hold the boot-bypass pin and reset for mpremote")
        try:
            flush()
        except Exception:
            pass
        wifi_stop()
        _wdt_keepalive()
        raise
    except Exception as e:
        _running = False
        if hasattr(sys, "print_exception"):
            sys.print_exception(e)
        else:
            print("crash", e)
        try:
            flush()
        except Exception:
            pass
        _boot_fail_bump()
        time.sleep(1)
        machine.reset()
    finally:
        _running = False
        try:
            flush()
        except Exception:
            pass
        wifi_stop()

def _loop():
    global _wdt
    if _wdt is None:
        _wdt = WDT(timeout=WDT_TIMEOUT)
    loop_start = time.ticks_ms()
    last = None
    while True:
        feed()
        read_uart()
        log_tick()
        wifi_tick()
        feed()
        upload_tick()
        now = time.ticks_ms()
        if (not _boot_ok_done
                and time.ticks_diff(now, loop_start) >= BOOT_OK_MS):
            _boot_ok()
        if last is None or time.ticks_diff(now, last) >= REFRESH:
            draw()
            last = now
        time.sleep_ms(20)
