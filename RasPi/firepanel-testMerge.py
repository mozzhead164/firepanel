#!/home/Dale/firepanel/RasPi/venv/bin/python

# ──────── IMPORTS ────────
import os                       # for filesystem paths, existence checks, etc.
import sys                      # for stderr/stdout, exiting, etc.
import time                     # for timestamps, sleep(), monotonic(), etc.
from time import sleep         # convenience: use sleep(x) instead of time.sleep(x)
import json                     # (de)serializing system_status to/from disk (once/minute)
import threading                # for spawning the Arduino/Node-RED monitor thread
from threading import Event     # used in firepanel.py as a stop_event (and possibly in LCD logic)
import serial                   # to talk to Arduino over UART
import socket                   # (if firepanel uses UDP/TCP anywhere)
import datetime                 # for log‐rotation timestamp helpers
import logging                  # for console/file logging in firepanel
from logging.handlers import TimedRotatingFileHandler
                              # for rotating firepanel.log every N weeks
import smbus2 as smbus          # for low‐level I²C operations (LCD may use this under the hood)
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
                              # (lcd_controller.py watched STATUS_FILE for changes; may keep if we still want FS notifications)
from RPLCD.i2c import CharLCD   # to drive the 20×4 I²C LCD display

# ──────── GLOBAL CONSTANTS ────────

# I²C device path and address (comes from lcd_controller.py)
I2C_DEV = "/dev/i2c-1"

# JSON‐status file paths (from lcd_controller.py)
STATUS_FILE   = "/home/Dale/firepanel/RasPi/system_status.json"
WATCHDOG_FILE = "/home/Dale/firepanel/RasPi/watchdog_status.json"

# Logging directory/file (from firepanel.py)
LOG_DIR  = os.path.expanduser("/home/Dale/firepanel/RasPi/logs")
LOG_FILE = os.path.join(LOG_DIR, "firepanel.log")

# ──────── INITIAL “IN‐MEMORY” STATE ────────

# This dict will be the single source of truth once we merge both scripts.
# Instead of having lcd_controller.py re‐read STATUS_FILE repeatedly,
# we’ll keep everything in this structure and only dump it to disk once per minute.
current_data = {
    'stage':   'booting',       # e.g. "booting", "initializing", "connected"
    'mode':    'BOOTING',       # e.g. operational mode string from Node-RED
    'conn':    [False]*8,       # 8‐element list: whether each channel is “connected” (Arduino)
    'trig':    [False]*8,       # 8‐element list: whether each channel is “triggered”
    'thermal': [False]*8,       # 8‐element list: whether each channel is in “thermal alarm”
    'avgTemp': None             # average temperature float, or None until known
}

# LCD “cache” of what’s currently displayed, so we only rewrite changed characters:
# This was in lcd_controller.py as well.
lcd_cache = [[' ' for _ in range(20)] for _ in range(4)]

# ──────── FIREPANEL‐SPECIFIC GLOBALS ────────

# How long to stay on the “booting” page before switching to “initializing”
BOOT_DURATION = 4.0           # seconds
start_time    = time.time()   # stamp when this script began (used for animations)

heartbeat_received = False    # once we see the first Arduino heartbeat, flip to “connected”

# Track previous trigger states (from firepanel.py) so we know when to start a new blink:
prev_trig    = [False] * 8
prev_thermal = [False] * 8

# When each channel’s blink began (so we can flash it for a fixed duration)
blink_start = [0.0] * 8       # float timestamps
blink_type  = [None] * 8      # either 'trig' or 'thermal' or None

# How long to hold a camera/trigger indication or thermal alarm indication
TRIG_HOLD_TIME    = 20       # seconds (firepanel.py used this for “camera” blinking)
THERMAL_HOLD_TIME = 20       # seconds

# Dictionary or other structures firepanel.py used for “trig”/“camera” metadata:
trig = {}                    # (populated when Arduino/Node-RED talk to us)

# For measuring missed‐heartbeat timeouts:
missed_heartbeats = 0        # count of consecutive loops without a heartbeat

# Watchdog / control event (from firepanel.py):
stop_event = threading.Event()

# ──────── LCD‐SPECIFIC GLOBALS (CUSTOM ICON BITMAPS) ────────

# These custom character bitmaps came from the top of lcd_controller.py.
# Each is an 8×5 pixel pattern that CharLCD.load_char can register.
tick_char     = [0b00000,0b00001,0b00010,0b10100,0b01000,0b00000,0b00000,0b00000]
cross_char    = [0b00000,0b10001,0b01010,0b00100,0b01010,0b10001,0b00000,0b00000]
therm_char    = [0B11111,0B11011,0B11011,0B11011,0B11011,0B11111,0B11011,0B11111]
flame_outline = [0B00100,0B00110,0B00110,0B01011,0B01001,0B10101,0B10101,0B01110]
flame_filled  = [0B00100,0B00110,0B00110,0B01111,0B01111,0B11111,0B11111,0B01110]

# ──────── LCD ANIMATION GLOBALS ────────
last_stage = None
booting_idx = 0
arrow_pos   = 0
handshake_left = 0
handshake_right = 19
handshake_previous_left = None
handshake_previous_right = None

# ──────── LCD TIMING CONSTANTS (from lcd_controller.py) ────────
FLASH_INTERVAL = 0.33        # how often (in seconds) to toggle “flash_on” for blinking icons
POLL_INTERVAL  = 0.2        # how often (in seconds) the LCD loop will rebuild/update the screen

# ──────── SERIAL CONSTANTS ────────
FRAME_TIMEOUT    = 1.0
MAX_FRAME_LENGTH = 1024


# ──────── ARDUINO SPECIFIC GLOBALS ────────
handshake_complete = False
last_heartbeat     = 0.0
missed_heartbeats  = 0


# ──────── CAMERA & THERMAL ALARM TRACKING ────────
camera_trigger_times = {}     # e.g. { channel_idx: timestamp, … }
thermal_alarm_times  = {}     # e.g. { channel_idx: timestamp, … }

# In the original, they had:
socket_path = "/home/Dale/firepanel/RasPi/firepanel.sock"


# ──────── LOG‐ROTATION SETTINGS (from firepanel.py) ────────
# Note: we’ll hook these into the logging setup in Step 2.
logger = logging.getLogger("firepanel")
logger.setLevel(logging.DEBUG)  # capture everything at DEBUG+; handlers can filter

# File‐handler: Rotate firepanel.log every 4 weeks, keep 12 backups (~48 weeks)
file_handler = TimedRotatingFileHandler(
    LOG_FILE,
    when="W0",            # “W0” means “every Monday” – though firepanel.py used this to approximate 4 weeks
    interval=4,           # interval of 4 weeks
    backupCount=12,       # keep last 12 logs (≈11 months)
    encoding="utf-8",
    utc=False,
    atTime=datetime.time(hour=0, minute=1)
)
file_handler.namer = lambda name: name + ".gz"  # compress rotated logs to .gz
file_handler.setLevel(logging.DEBUG)
file_formatter = logging.Formatter(
    "%(asctime)s %(levelname)-5s %(message)s",
    datefmt="%H:%M:%S"
)
file_handler.setFormatter(file_formatter)
logger.addHandler(file_handler)

# Console‐handler: only INFO+ to stdout (so debug details go to file, but not console)
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)
console_handler.stream = open(sys.stdout.fileno(), 'w', encoding='utf-8', closefd=False)
console_handler.setFormatter(
    logging.Formatter("%(asctime)s %(levelname)-5s %(message)s",
                      datefmt="%H:%M:%S")
)
logger.addHandler(console_handler)

# ──────── END OF MERGED HEADER ────────




# ─────────────────────────────────────────────────────────────────────────────
#                    SERIAL & NODE-RED MONITORING LOGIC
# ─────────────────────────────────────────────────────────────────────────────

def init_serial(port="/dev/ttyS0", baudrate=115200, timeout=0.5):
    """
    Open and return a serial.Serial object to communicate with the Arduino.
    Logs an error and returns None if it fails.
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=timeout)
        logger.info(f"Opened serial port {port} @ {baudrate} baud")
        return ser
    except Exception as e:
        logger.error(f"Failed to open serial port {port}: {e}")
        return None

# ─────────────────────────────────────────────────────────────────────────────
#                    send_json() & read_from_serial() (from backup)
# ─────────────────────────────────────────────────────────────────────────────

def send_json(ser, obj):
    """
    Wrap a JSON‐serializable dict in <…> and send it to the Arduino.
    Used for handshakes, get_data requests, fire/camera triggers, etc.
    """
    try:
        msg = "<" + json.dumps(obj) + ">"
        ser.write(msg.encode("ascii"))
        logger.debug(f"Sent → Arduino: {msg}")
    except Exception as e:
        logger.error(f"Failed to send JSON to Arduino: {e}")

# —– Read from Serial Port ———
def read_from_serial(ser):
    buffer = []               # list of chars inside <…>
    in_frame = False
    last_byte_time = time.monotonic()

    while not stop_event.is_set():
        try:
            # read up to whatever’s available, or at least 1 byte
            to_read = ser.in_waiting or 1
            data = ser.read(to_read)
            if not data:
                # timeout: if we were mid-frame and nothing’s changed for FRAME_TIMEOUT, resync
                if in_frame and (time.monotonic() - last_byte_time) > FRAME_TIMEOUT:
                    logger.warning("Serial frame timeout – discarding partial buffer")
                    buffer.clear()
                    in_frame = False
                continue

            last_byte_time = time.monotonic()

            for b in data:
                ch = chr(b)

                if ch == "<":
                    # start marker: reset buffer
                    in_frame = True
                    buffer.clear()
                elif ch == ">" and in_frame:
                    # end marker: process complete frame
                    frame = "".join(buffer).strip()
                    in_frame = False
                    buffer.clear()
                    if frame:
                        handle_frame(frame)
                elif in_frame:
                    buffer.append(ch)

                    # guard against runaway frames
                    if len(buffer) > MAX_FRAME_LENGTH:
                        logger.warning(
                            "Serial frame exceeded %d bytes—resyncing",
                            MAX_FRAME_LENGTH
                        )
                        buffer.clear()
                        in_frame = False

        except serial.SerialException:
            logger.exception("Serial port exception—trying to reopen in 1s")
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(1)
            try:
                ser.open()
                ser.reset_input_buffer()
                logger.info("Serial port reopened and input buffer flushed")
            except Exception as reopen_err:
                logger.exception("Failed to reopen serial port")
        except Exception:
            logger.exception("Unexpected error in serial read loop")
            time.sleep(0.1)

# ─────────────────────────────────────────────────────────────────────────────
#                    socket_command_listener() (from backup)
# ─────────────────────────────────────────────────────────────────────────────

def socket_command_listener(ser):
    """
    Bind to a Unix socket (firepanel.sock) and listen for commands from Node-RED.
    Each line is a UTF-8 string: e.g. "ping", "self-test", "thermal_trigger:3"
    We parse and forward JSON to the Arduino via send_json(ser, {...}).
    """
    # Remove old socket file if it exists
    try:
        os.remove(socket_path)
    except OSError:
        pass

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(socket_path)
    server.listen(1)
    logger.info(f"Socket listener up on {socket_path}")

    while not stop_event.is_set():
        try:
            conn, _ = server.accept()
            data = conn.recv(1024).decode("utf-8").strip()
            if data == "ping":
                send_json(ser, {"type": "ping"})
            elif data == "self-test":
                send_json(ser, {"type": "self_test"})
            elif data.startswith("thermal_trigger:"):
                idx = int(data.split(":")[1])
                send_json(ser, {"type": "trigger_thermal", "channel": idx})
                write_watchdog_status({"socket_cmd": data})
            elif data.startswith("camera_trigger:"):
                idx = int(data.split(":")[1])
                send_json(ser, {"type": "trigger_camera", "channel": idx})
                write_watchdog_status({"socket_cmd": data})
            else:
                logger.warning(f"Unknown socket command: {data}")

            conn.close()
        except Exception as e:
            logger.error(f"Socket listener error: {e}")
            time.sleep(0.5)


# ─────────────────────────────────────────────────────────────────────────────
#                       handle_frame() (from backup)
# ─────────────────────────────────────────────────────────────────────────────

def handle_frame(frame):
    """
    Parse one JSON string (no '<' or '>') from the Arduino, update current_data
    and any on‐disk status files, and perform handshaking logic.
    This is an unmodified copy from your original backup.
    """
    print(f"[HB-DEBUG] Received raw frame: {repr(frame)}")

    global last_heartbeat, handshake_complete, heartbeat_received

    try:
        j = json.loads(frame)
    except Exception:
        logger.warning(f"Malformed JSON frame: {frame}")
        print(f"[HB-DEBUG] Malformed JSON (not even valid JSON): {repr(frame)}")
        return

    # Example: handle an initial handshake response
    if j.get("type") == "handshake":
        if not handshake_complete:
            # Send back the same handshake so Arduino knows we’re here
            send_json(ser, {"type": "ack", "msg": "handshake"})
            handshake_complete = True
            last_heartbeat = time.time()
            current_data["stage"] = "connected"
            update_status_fields(stage="connected")
            logger.info("Handshake complete with Arduino")
        return

    # If this is a “data” frame, it could contain conn/trig/thermal/avgTemp, etc.
    if j.get("type") == "data":

        last_heartbeat = time.time()

        # Example fields: j = { "type":"data", "conn":[1,0,…], "trig":[…], "thermal":[…], "avgTemp":23.4 }
        mode_new = [bool(x) for x in j.get("mode", current_data["mode"])]
        conn_new = [bool(x) for x in j.get("conn", current_data["conn"])]
        trig_new = [bool(x) for x in j.get("trig", current_data["trig"])]
        ther_new = [bool(x) for x in j.get("thermal", current_data["thermal"])]
        temp_new = j.get("avgTemp", current_data["avgTemp"])

        to_write = {} # Fields to write to disk if they changed

        # Update current_data with new values, if they differ
        if mode_new != current_data["mode"]:
            current_data["mode"] = mode_new
            to_write["mode"] = mode_new

        if conn_new != current_data["conn"]:
            current_data["conn"] = conn_new
            to_write["conn"] = conn_new

        if trig_new != current_data["trig"]:
            current_data["trig"] = trig_new
            to_write["trig"] = trig_new

        if thermal_new != current_data["thermal"]:
            current_data["thermal"] = thermal_new
            to_write["thermal"] = ther_new

        if to_write:
            update_status_fields(**to_write)

        logger.debug(f"Data frame → current_data updated: {j}")
        return

    # If this is a “alert” (trigger_thermal), etc., update those fields then write disk
    if j.get("type") == "alert":
        idx = j.get("channel")
        if j.get("alertType") == "thermal":
            current_data["thermal"][idx] = True
            update_status_fields(thermal=current_data["thermal"])
        elif j.get("alertType") == "trigger":
            current_data["trig"][idx] = True
            update_status_fields(trig=current_data["trig"])
        logger.warning(f"Received ALERT from Arduino: {j}")
        return

    # If this is a “heartbeat” frame (solo), update last_heartbeat
    if j.get("type") == "heartbeat":
        last_heartbeat = time.time()
        heartbeat_received = True
        current_data["stage"] = "connected"
        update_status_fields(stage="connected")
        logger.debug("Heartbeat frame received")
        return

    # If this is an “ack” to some command we sent (e.g. for self_test or fire), ignore or log
    if j.get("type") == "ack":
        logger.info(f"Arduino ACK: {j.get('msg','')}")
        return

    # Handle other frame types (“self_test”, “output_confirm”, etc.) exactly as in backup
    # …
    # If none of the above, just log it
    logger.debug(f"Unhandled frame type: {j}")

# ─────────────────────────────────────────────────────────────────────────────
#                    load_status_file() and update_status_fields()
# ─────────────────────────────────────────────────────────────────────────────

def load_status_file():
    """
    Read the on‐disk system_status.json (if it exists), else return a default.
    """
    default = {
        "stage":    "booting",
        "mode":     "BOOTING",
        "conn":     [False]*8,
        "trig":     [False]*8,
        "thermal":  [False]*8,
        "avgTemp":  None
    }
    try:
        with open(STATUS_FILE, "r") as f:
            return json.load(f)
    except Exception:
        return default

def update_status_fields(**kwargs):
    """
    Merge the keyword args into the on‐disk system_status.json atomically.
    E.g. update_status_fields(stage="connected", conn=[…], avgTemp=23.4).
    """
    data = load_status_file()
    data.update(kwargs)
    try:
        tmp = STATUS_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, STATUS_FILE)
    except Exception as e:
        logger.warning(f"Could not write {STATUS_FILE}: {e}")

def periodic_status_dump():
    """
    Once a minute, write all fields of current_data into system_status.json.
    """
    while not stop_event.is_set():
        # Copy out the fields you want persisted
        update_status_fields(
            stage=current_data.get("stage", "unknown"),
            mode=current_data.get("mode", "UNKNOWN"),
            conn=current_data.get("conn", [False]*8),
            trig=current_data.get("trig", [False]*8),
            thermal=current_data.get("thermal", [False]*8),
            avgTemp=current_data.get("avgTemp", None)
        )
        # Sleep 60 seconds. If stop_event is set, this will exit promptly.
        for _ in range(60):
            if stop_event.is_set():
                return
            sleep(1)


# ─────────────────────────────────────────────────────────────────────────────
#                    Trig and Thermal Cleanup Loops (from backup)
# ─────────────────────────────────────────────────────────────────────────────

def trig_cleanup_loop():
    """
    Periodically clear any camera‐trigger flags older than TRIG_HOLD_TIME.
    """
    while not stop_event.is_set():
        status = load_status_file()
        ct = status.get("trig", [False]*8)
        updated = False
        now = time.time()
        for idx, ts in list(camera_trigger_times.items()):
            if now - ts > TRIG_HOLD_TIME:
                ct[idx] = False
                del camera_trigger_times[idx]
                updated = True
        if updated:
            update_status_fields(trig=ct)
        time.sleep(10)

def thermal_cleanup_loop():
    """
    Periodically clear any thermal‐alarm flags older than THERMAL_HOLD_TIME.
    """
    while not stop_event.is_set():
        status = load_status_file()
        th = status.get("thermal", [False]*8)
        updated = False
        now = time.time()
        for idx, ts in list(thermal_alarm_times.items()):
            if now - ts > THERMAL_HOLD_TIME:
                th[idx] = False
                del thermal_alarm_times[idx]
                updated = True
        if updated:
            update_status_fields(thermal=th)
        time.sleep(10)

# ─────────────────────────────────────────────────────────────────────────────
#                          write_watchdog_status() & watchdog_loop()
# ─────────────────────────────────────────────────────────────────────────────

def write_watchdog_status(payload):
    """
    Write a small JSON to watchdog_status.json containing payload merged with
    missed_heartbeats and any other fields desired.
    """
    try:
        status = load_status_file()
        watcher = {
            "timestamp": int(time.time()),
            "missed_heartbeats": missed_heartbeats
        }
        data = {**status, **watcher, **payload}
        tmp = WATCHDOG_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump(data, f, indent=2)
        os.replace(tmp, WATCHDOG_FILE)
    except Exception as e:
        logger.warning(f"Could not write {WATCHDOG_FILE}: {e}")

def watchdog_loop():
    """
    Every second, compare time.time() – last_heartbeat against various thresholds.
    If crossing WARN or CRIT thresholds, log and write_watchdog_status(...).
    """
    WARN_THRESHOLD = 15    # seconds without heartbeat before WARN
    CRIT_THRESHOLD = 30   # seconds without heartbeat before CRIT

    state = "OK"
    while not stop_event.is_set():
        now = time.time()
        delta = now - last_heartbeat
        if delta > CRIT_THRESHOLD and state != "CRIT":
            state = "CRIT"
            logger.critical("Watchdog: no heartbeat → CRITICAL")
            write_watchdog_status({"watchdog_state": "CRIT"})
        elif delta > WARN_THRESHOLD and state != "WARN":
            state = "WARN"
            logger.warning("Watchdog: no heartbeat → WARNING")
            write_watchdog_status({"watchdog_state": "WARN"})
        elif delta <= WARN_THRESHOLD and state != "OK":
            state = "OK"
            logger.info("Watchdog: heartbeat OK")
            write_watchdog_status({"watchdog_state": "OK"})

        time.sleep(1)


# ─────────────────────────────────────────────────────────────────────────────────
#                      3A. INIT LCD + REGISTER CUSTOM ICONS
# ─────────────────────────────────────────────────────────────────────────────────

lcd = None  # will hold the CharLCD instance

def init_lcd():
    """
    Wait for the I²C device node (e.g. "/dev/i2c-1") to appear, then create
    a CharLCD object and register custom characters (tick, cross, etc.).
    Returns True on success, False on failure.
    """
    global lcd
    start = time.time()
    # Poll until /dev/i2c-1 exists or 15 seconds have passed
    while not os.path.exists(I2C_DEV):
        if time.time() - start > 15:
            logger.error(f"I2C device {I2C_DEV} not found after 15 s")
            return False
        sleep(0.1)

    try:
        lcd = CharLCD(
            i2c_expander='PCF8574',
            address=0x27,
            port=1,
            cols=20,
            rows=4,
            charmap='A00',
            auto_linebreaks=False
        )
        # Register each custom character at its own index (1–5)
        lcd.create_char(1, tick_char)
        lcd.create_char(2, cross_char)
        lcd.create_char(3, therm_char)
        lcd.create_char(4, flame_outline)
        lcd.create_char(5, flame_filled)
        logger.info("LCD initialized and custom icons registered")
        return True
    except Exception as e:
        logger.error(f"LCD initialization failed: {e}")
        return False

# ─────────────────────────────────────────────────────────────────────────────────
#                      3B. LCD HELPER FUNCTIONS (CLEAR & WRITE DIFF)
# ─────────────────────────────────────────────────────────────────────────────────

def clear_screen_full():
    """
    Clear the entire LCD display and reset lcd_cache to spaces. Use this
    whenever we switch pages (boot → init → connected).
    """
    global lcd_cache
    if lcd:
        lcd.clear()
    lcd_cache = [[' ' for _ in range(20)] for _ in range(4)]

def update_screen(full_screen):
    """
    Compare the desired 4×20 character buffer (full_screen: list[str] of length 4)
    against lcd_cache, and write only those characters that differ.
    """
    for row in range(4):
        for col in range(20):
            target_char = full_screen[row][col]
            if lcd_cache[row][col] != target_char:
                try:
                    lcd.cursor_pos = (row, col)
                    lcd.write_string(target_char)
                    lcd_cache[row][col] = target_char
                except Exception as e:
                    logger.warning(f"I²C write error at ({row},{col}): {e}")
                    # Optionally, attempt a re-init if e.g. bus resets
                    # For now, we skip and hope next cycle recovers

def lcd_write_safe(pos, text):
    try:
        if lcd:
            lcd.cursor_pos = pos
            lcd.write_string(text)
    except Exception as e:
        print(f"[WARN] LCD write failed: {e}", file=sys.stderr)
        init_lcd()

# ─────────────────────────────────────────────────────────────────────────────────
#                      3C. BUILDING THE SCREEN FROM current_data
# ─────────────────────────────────────────────────────────────────────────────────

def build_trig_line(trig_states, thermal_states):
    """
    Construct a 20-char “TRIG …” line with one icon per channel:
      • chr(3) (flame) if trig_states[i] is True
      • chr(2) (therm)  if thermal_states[i] is True and trig is False
      • chr(1) (cross) if neither is True
    Then center the result in 20 columns.
    """
    # Start with “TRIG ” and one space between each icon
    # e.g. “TRIG X X X X X X X X”
    line = "TRIG "
    for i in range(8):
        if trig_states[i]:
            icon = chr(4)      # flame
        elif thermal_states[i]:
            icon = chr(2)      # thermometer
        else:
            icon = chr(3)      # cross
        line += icon + " "
    # Strip trailing space and center in 20 chars
    return line.strip().center(20)



def build_screen_from_data(system_data, flash_on=False):
    """
    Returns a list of four 20‐char strings:
      • Line 1: Mode left, Temperature right
      • Line 2: “CHAN 1 2 3 4 5 6 7 8” centered
      • Line 3: “CONN <8 icons>” centered
      • Line 4: Trigger/thermal icons via build_trig_line()
    """
    # Line 1: mode and temperature
    try:
        avg_temp = float(system_data.get("avgTemp", None))
        temp_str = f"{avg_temp:.1f}C"
    except (TypeError, ValueError):
        temp_str = "--.-C"

    mode_text = system_data.get("mode", "UNKNOWN")
    if mode_text == "ARMED":
        mode_str = "Mode: ARMED"
    elif mode_text == "TEST":
        mode_str = "Mode: TEST"
    elif mode_text == "NO_OUTPUT":
        mode_str = "Mode: No Suppr"
    else:
        mode_str = f"Mode: {mode_text}"

    # Pad/truncate to 20 chars: mode_str on left, temp_str on right
    line1 = (mode_str + temp_str.rjust(20 - len(mode_str)))[:20]

    # Line 2: channel labels
    line2 = "CHAN 1 2 3 4 5 6 7 8".center(20)

    # Line 3: connection icons (0 vs tick char '\x00')
    conn_states = system_data.get("conn", [False]*8)
    # Use '\x00' for ON, '\x01' (or '0') for OFF—pick the custom‐icon indices you preloaded
    conn_icons = [('\x00' if c else '\x01') for c in conn_states]
    line3 = ("CONN " + " ".join(conn_icons)).center(20)

    # Line 4: build trigger/thermal line with your existing helper
    trig_states = system_data.get("trig", [False]*8)
    thermal_states = system_data.get("thermal", [False]*8)
    # If lengths are wrong, fallback to all False
    if len(trig_states) != 8:
        trig_states = [False]*8
    if len(thermal_states) != 8:
        thermal_states = [False]*8

    line4 = build_trig_line(system_data.get("trig", [False]*8),
                        system_data.get("thermal", [False]*8))

    return [line1, line2, line3, line4]


# ─────────────────────────────────────────────────────────────────────────────────
#                      3D. PAGE SELECTION & ANIMATIONS
# ─────────────────────────────────────────────────────────────────────────────────

def determine_page():
    """
    Decide which “page” to display:
     - If time since start < BOOT_DURATION → "booting"
     - Else if we’ve never seen a heartbeat → "initializing"
     - Else → "connected"
    """
    global heartbeat_received, start_time
    now = time.time()
    elapsed = now - start_time

    if elapsed < BOOT_DURATION:
        return "booting"
    if heartbeat_received:
        return "connected"
    return "initializing"


def booting_animation_frame():
    global booting_idx

    border_coords = ([(0, i) for i in range(20)] + [(i, 19) for i in range(1, 4)] +
                     [(3, i) for i in reversed(range(20))] + [(i, 0) for i in reversed(range(1, 3))])
    total = len(border_coords)
    half = total // 2

    # Erase previous asterisks
    pos1 = border_coords[booting_idx % total]
    pos2 = border_coords[(booting_idx + half) % total]
    lcd_write_safe(pos1, " ")
    lcd_write_safe(pos2, " ")

    # Advance position
    booting_idx = (booting_idx + 1) % total

    # Draw new asterisks
    pos1 = border_coords[booting_idx % total]
    pos2 = border_coords[(booting_idx + half) % total]
    lcd_write_safe(pos1, "*")
    lcd_write_safe(pos2, "*")

    # Force overwrite full line with padded string
    line1 = "System Booting".center(20)
    line2 = "Please Wait".center(20)
    lcd_write_safe((1, 0), line1)
    lcd_write_safe((2, 0), line2)

    # Update cache manually so next redraws are clean
    for i in range(20):
        lcd_cache[1][i] = line1[i]
        lcd_cache[2][i] = line2[i]




def handshake_animation_frame():
    global handshake_left, handshake_right, handshake_previous_left, handshake_previous_right

    # Write static text on line 1 (row index 0-based = 1)
    line1 = "Initialising".center(20)
    lcd_write_safe((1, 0), line1)
    for i in range(20):
        lcd_cache[1][i] = line1[i]

    # Clear previous animation arrows (line 3 / row 2)
    if handshake_previous_left is not None and handshake_previous_left not in [9, 10]:
        lcd_write_safe((2, handshake_previous_left), " ")
        lcd_cache[2][handshake_previous_left] = " "
    if handshake_previous_right is not None and handshake_previous_right not in [9, 10]:
        lcd_write_safe((2, handshake_previous_right), " ")
        lcd_cache[2][handshake_previous_right] = " "

    # Draw new animation arrows
    if handshake_left not in [9, 10]:
        lcd_write_safe((2, handshake_left), ">")
        lcd_cache[2][handshake_left] = ">"
    if handshake_right not in [9, 10]:
        lcd_write_safe((2, handshake_right), "<")
        lcd_cache[2][handshake_right] = "<"

    lcd_write_safe((2, 9), "|")
    lcd_write_safe((2, 10), "|")
    lcd_cache[2][9] = "|"
    lcd_cache[2][10] = "|"

    # Save current positions
    handshake_previous_left = handshake_left
    handshake_previous_right = handshake_right

    handshake_left += 1
    handshake_right -= 1

    if handshake_left >= 9:
        sleep(0.5)

        # Clear >||< before restarting
        for col in range(20):
            lcd_write_safe((2, col), " ")
            lcd_cache[2][col] = " "

        handshake_left = 0
        handshake_right = 19
        handshake_previous_left = None
        handshake_previous_right = None



# ─────────────────────────────────────────────────────────────────────────────────
#                      3E. MAIN LCD‐UPDATE LOOP
# ─────────────────────────────────────────────────────────────────────────────────

def lcd_main_loop():
    """
    This is the core loop that:
     1. Initializes the LCD (init_lcd)
     2. Clears it once
     3. Every POLL_INTERVAL seconds:
        a. Toggles flash_on every FLASH_INTERVAL
        b. Checks if the “page” changed (booting → init → connected)
           → If so, clear_screen_full() and draw initial frame
        c. If on “booting” or “initializing”, run the corresponding animation frame
        d. If on “connected”, build_screen_from_data(current_data, flash_on)
           and update_screen(...) to diff vs. lcd_cache
    """
    global start_time, heartbeat_received

    # 1) Init LCD hardware
    if not init_lcd():
        logger.error("lcd_main_loop: Failed to initialize LCD; exiting")
        return
    clear_screen_full()

    last_stage = None
    flash_on   = True
    last_flash = time.time()

    while True:
        now = time.time()

        # a) toggle flash_on every FLASH_INTERVAL
        if now - last_flash >= FLASH_INTERVAL:
            last_flash = now
            flash_on = not flash_on

        # b) decide which “page” we want
        page = determine_page()

        # If the page changed (e.g. from booting → initializing), do a full clear + first draw
        if page != last_stage:
            clear_screen_full()
            if page == "booting":
                booting_animation_frame()
            elif page == "initializing":
                handshake_animation_frame()
            elif page == "connected":
                screen = build_screen_from_data(current_data, flash_on)
                update_screen(screen)
            last_stage = page

        # c) run per‐frame animation or screen update
        if page == "booting":
            booting_animation_frame()
        elif page == "initializing":
            handshake_animation_frame()
        else:  # connected
            screen = build_screen_from_data(current_data, flash_on)
            update_screen(screen)

        sleep(POLL_INTERVAL)

# ─────────────────────────────────────────────────────────────────────────────────
#                      3F. GLUE EVERYTHING IN __main__
# ─────────────────────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    # 1) Open the serial port to the Arduino
    ser = init_serial("/dev/ttyS0", baudrate=115200, timeout=0.1)
    if not ser:
        logger.error("Could not open serial port; exiting")
        sys.exit(1)

    # Initialise last_heartbeat with current time
    last_heartbeat = time.time()

    # 2) Start the framed-JSON reader thread instead of monitor_arduino_and_nodered
    t_serial = threading.Thread(target=read_from_serial, args=(ser,), daemon=True)
    t_serial.start()
    logger.info("Started read_from_serial thread")

    # 3) Start any other background threads (trig_cleanup, thermal_cleanup, watchdog, socket_listener)
    t_trig = threading.Thread(target=trig_cleanup_loop, daemon=True)
    t_trig.start()
    logger.info("Started trig_cleanup_loop thread")

    t_thermal = threading.Thread(target=thermal_cleanup_loop, daemon=True)
    t_thermal.start()
    logger.info("Started thermal_cleanup_loop thread")

    t_watchdog = threading.Thread(target=watchdog_loop, daemon=True)
    t_watchdog.start()
    logger.info("Started watchdog_loop thread")

    t_socket = threading.Thread(target=socket_command_listener, args=(ser,), daemon=True)
    t_socket.start()
    logger.info("Started socket_command_listener thread")

    t_periodic = threading.Thread(target=periodic_status_dump, daemon=True)
    t_periodic.start()
    logger.info("Started periodic_status_dump thread")

    # 4) Finally, run the LCD update loop in the main thread
    try:
        lcd_main_loop()
    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt: shutting down")
        stop_event.set()
        # Give background threads a moment to exit
        t_trig.join(timeout=2.0)
        t_thermal.join(timeout=2.0)
        t_watchdog.join(timeout=2.0)
        t_socket.join(timeout=2.0)
        t_periodic.join(timeout=2.0)
        logger.info("Exited cleanly")

        logger.info("KeyboardInterrupt caught: shutting down")
        stop_event.set()
        # Give background thread a moment to exit
        logger.info("Exited cleanly")
    finally:
        # Give threads a moment and then close serial
        t_serial.join(timeout=1.0)
        if ser and ser.is_open:
            ser.close()
        logger.info("Serial port closed, exiting")