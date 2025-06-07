import os
import time
import smbus
import sys
import json
from time import sleep
from threading import Event
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from RPLCD.i2c import CharLCD


I2C_DEV = "/dev/i2c-1"

STATUS_FILE = "/home/Dale/firepanel/RasPi/system_status.json"
WATCHDOG_FILE = "/home/Dale/firepanel/RasPi/watchdog_status.json"

lcd_cache = [[' ' for _ in range(20)] for _ in range(4)]
current_data = {
    'stage': 'booting',
    'mode': 'BOOTING',
    'conn': [False]*8,
    'trig': [False]*8,
    'thermal': [False]*8  
}


FLASH_INTERVAL = 0.33        # seconds for on/off toggle
POLL_INTERVAL  = 0.05        # how often to update the display
BLINK_HOLD     = 25         # default blink duration (seconds)
flash_on         = True
last_flash_tick  = time.monotonic()

# local state
last_status_mtime = 0
prev_trig         = [False]*8
prev_thermal      = [False]*8
blink_start       = [0.0]*8   # when this channel’s blink began
blink_type        = [None]*8  # "camera" or "thermal"


BOOT_DURATION = 8.0  # seconds
start_time = time.time()
heartbeat_received = False

last_stage = None
booting_idx = 0
connect_left = 0
connect_right = 19
connect_previous_left = None
connect_previous_right = None

watchdogActive = False
last_toggle_time = 0
showing_trouble = False
status_changed = Event()

flash_tick_on = True
last_flash_toggle = time.time()
previous_trig_states = [False] * 8




# LCD Setup
I2C_ADDRESS = 0x27
lcd = None


# Tick icon - (Cable Connected)
tick_char = [
    0b00000,
    0b00001,
    0b00010,
    0b10100,
    0b01000,
    0b00000,
    0b00000,
    0b00000
]

# Cross icon (Not Triggered or Connected)
cross_char = [
    0b00000,
    0b10001,
    0b01010,
    0b00100,
    0b01010,
    0b10001,
    0b00000,
    0b00000
]

# Thermometer icon (for thermal-only trigger)
thermo_char = [
  0B11111,
  0B11011,
  0B11011,
  0B11011,
  0B11011,
  0B11111,
  0B11011,
  0B11111,
]

# Flame icon (outline version)
flame_outline = [
  0B00100,
  0B00110,
  0B00110,
  0B01011,
  0B01001,
  0B10101,
  0B10101,
  0B01110,
]

# Flame icon (filled version)
flame_filled = [
  0B00100,
  0B00110,
  0B00110,
  0B01111,
  0B01111,
  0B11111,
  0B11111,
  0B01110,
]




def init_lcd():
    global lcd, bus
    try:
        start = time.time()
        while not os.path.exists(I2C_DEV):
            if time.time() - start > 15:
                break
            time.sleep(0.1)

        # now bus will succeed (or at worst throw)
        bus = smbus.SMBus(1)
        lcd = CharLCD('PCF8574', I2C_ADDRESS, port=1, cols=20, rows=4)

        lcd.create_char(0, tick_char)           # ID 0 - Tick
        lcd.create_char(1, cross_char)          # ID 1 - Cross
        lcd.create_char(2, thermo_char)         # ID 2 - Thermometer
        lcd.create_char(3, flame_outline)       # ID 3 - Flame Outline
        lcd.create_char(4, flame_filled)        # ID 4 - Flame Filled

        return True
    except Exception as e:
        print(f"[ERROR] LCD init failed: {e}", file=sys.stderr)
        lcd = None
        return False


def lcd_write_safe(pos, text):
    try:
        if lcd:
            lcd.cursor_pos = pos
            lcd.write_string(text)
    except Exception as e:
        print(f"[WARN] LCD write failed: {e}", file=sys.stderr)
        init_lcd()


def update_screen(new_screen_data):
    for row in range(4):
        line = new_screen_data[row]
        for col in range(min(len(line), 20)):
            new_char = line[col]
            if lcd_cache[row][col] != new_char:
                lcd_write_safe((row, col), new_char)
                lcd_cache[row][col] = new_char


def build_screen_from_system(system_data):
    # Debug inspection
    print("[DEBUG] system_data =", system_data)

    # Temperature display
    try:
        avg_temp = float(system_data.get("avgTemp", None))
        temp_str = f"{avg_temp:.1f}C"
    except (TypeError, ValueError):
        temp_str = "--.-C"

    # Mode formatting
    mode_text = system_data.get("mode", "UNKNOWN")
    if mode_text == "ARMED":
        mode_str = "Mode: ARMED"
    elif mode_text == "TEST":
        mode_str = "Mode: TEST"
    elif mode_text == "NO_OUTPUT":
        mode_str = "Mode: No Suppr"
    else:
        mode_str = f"Mode: {mode_text}"

    # First line: Mode left, Temp right
    line1 = (mode_str + temp_str.rjust(20 - len(mode_str)))[:20]

    # Channel indicators
    line2 = "CHAN 1 2 3 4 5 6 7 8".center(20)
    conn_row = ['\x00' if c else '\x01' for c in system_data.get("conn", [False]*8)]
    line3 = ("CONN " + ' '.join(conn_row)).center(20)
    
    trig_row = []
    trig_states = system_data.get("trig", [False]*8)
    thermal_states = system_data.get("thermal", [False]*8)

    if len(trig_states) != 8:
        print("[WARN] Invalid trig state length:", trig_states)
        trig_states = [False]*8  # fallback

    if len(trig_states) != 8:
        print("[WARN] Invalid trig state length:", trig_states)
        trig_states = [False]*8  # fallback
    

    # Build Line 4 - Triggered Events
    line4 = build_trig_line(trig_states, thermal_states)
    
    return [line1, line2, line3, line4]



def build_trouble_screen():
    return ["*** SYSTEM TROUBLE ***".center(20), " ".center(20), " ".center(20), " ".center(20)]




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


def clear_screen_full():
    global lcd_cache
    if lcd:
        try:
            lcd.clear()
        except Exception as e:
            print(f"[WARN] Full LCD clear failed: {e}", file=sys.stderr)
            init_lcd()
    lcd_cache = [[' ' for _ in range(20)] for _ in range(4)]


def connect_animation_frame():
    global connect_left, connect_right, connect_previous_left, connect_previous_right

    # Write static text on line 1 (row index 0-based = 1)
    line1 = "Initialising".center(20)
    lcd_write_safe((1, 0), line1)
    for i in range(20):
        lcd_cache[1][i] = line1[i]

    # Clear previous animation arrows (line 3 / row 2)
    if connect_previous_left is not None and connect_previous_left not in [9, 10]:
        lcd_write_safe((2, connect_previous_left), " ")
        lcd_cache[2][connect_previous_left] = " "
    if connect_previous_right is not None and connect_previous_right not in [9, 10]:
        lcd_write_safe((2, connect_previous_right), " ")
        lcd_cache[2][connect_previous_right] = " "

    # Draw new animation arrows
    if connect_left not in [9, 10]:
        lcd_write_safe((2, connect_left), ">")
        lcd_cache[2][connect_left] = ">"
    if connect_right not in [9, 10]:
        lcd_write_safe((2, connect_right), "<")
        lcd_cache[2][connect_right] = "<"

    lcd_write_safe((2, 9), "|")
    lcd_write_safe((2, 10), "|")
    lcd_cache[2][9] = "|"
    lcd_cache[2][10] = "|"

    # Save current positions
    connect_previous_left = connect_left
    connect_previous_right = connect_right

    connect_left += 1
    connect_right -= 1

    if connect_left >= 9:
        sleep(0.5)

        # Clear >||< before restarting
        for col in range(20):
            lcd_write_safe((2, col), " ")
            lcd_cache[2][col] = " "

        connect_left = 0
        connect_right = 19
        connect_previous_left = None
        connect_previous_right = None


# def update_flashing_triggers():
#     trig = current_data.get("trig", [False] * 8)
#     thermal = current_data.get("thermal", [False] * 8)

#     for i in range(8):
#         col = 5 + i * 2  # position on LCD line
#         if trig[i]:
#             char = '\x03' if flash_tick_on else ' '
#         elif thermal[i]:
#             char = '\x02' if flash_tick_on else ' '
#         else:
#             continue  # no update needed

#         lcd_write_safe((3, col), char)
#         lcd_cache[3][col] = char  # keep cache in sync


def build_trig_line(trig_states, thermal_states):
    
    # ID 0 - Tick
    # ID 1 - Cross
    # ID 2 - Thermometer
    # ID 3 - Flame Outline
    # ID 4 - Flame Filled

    line = "TRIG "
    for i in range(8):
        if trig_states[i]:
            icon = '\x03'      # flame
        elif thermal_states[i]:
            icon = '\x02'      # thermometer
        else:
            icon = '\x01'      # cross
        line += icon + " "
    return line.strip().center(20)



class StatusChangeHandler(FileSystemEventHandler):
    def on_modified(self, event):
        if event.src_path == STATUS_FILE:
            status_changed.set()


def load_status_file():
    try:
        with open(STATUS_FILE, 'r') as f:
            data = json.load(f)
            return {
                'stage': data.get('stage', 'booting'),
                'mode': data.get('mode', 'UNKNOWN').upper(),
                'conn': data.get('conn', [False]*8),
                'trig': data.get('trig', [False]*8),
                'thermal':data.get('thermal', [False]*8), 
                'avgTemp': data.get('avgTemp', None)
            }
    except Exception as e:
        print(f"[WARN] Failed to load status file: {e}", file=sys.stderr)
        return None


def handle_status_file_update():
    global current_data, watchdogActive, last_stage
    data = load_status_file()

    if data:
        current_data = data
        global previous_trig_states
        previous_trig_states = current_data.get("trig", [False]*8)

    if os.path.exists(WATCHDOG_FILE):
        try:
            with open(WATCHDOG_FILE, 'r') as f:
                status = json.load(f)
            # new schema: either an explicit "active" key, or "state"
            if 'active' in status:
                watchdogActive = bool(status.get('active'))
            else:
                # treat any non-"ok" state as active
                watchdogActive = status.get('state', 'ok') not in ('ok', 'OK')
        except Exception:
            watchdogActive = False


    # Decide which page we should be on now
    page = determine_page(current_data)

    # # If the page changed, clear the screen
    # if page != last_stage:
    #     clear_screen_full()
    #     last_stage = page

    # For booting / initializing we have their own frame-renderers
    if page == "connected":
        screen = build_screen_from_system(current_data)
        update_screen(screen)
    # else: we do nothing here. The main loop runs the animations


def determine_page(status):
    """
    Three states:
      - 'booting'      : for first BOOT_DURATION seconds after script start
      - 'initializing' : after BOOT_DURATION but before first heartbeat
      - 'connected'    : once we’ve seen a heartbeat from the Arduino
    """
    global heartbeat_received

    now = time.time()
    elapsed = now - start_time

    # Always show “BOOTING” for the first BOOT_DURATION seconds
    if elapsed < BOOT_DURATION:
        return "booting"

    # If the file’s stage is 'connected', or we’ve previously seen one, we’re connected
    if status.get("stage") == "connected" or heartbeat_received:
        heartbeat_received = True
        return "connected"

    # Otherwise we’re still waiting on that first heartbeat
    return "initializing"

def check_and_handle_stage_change(new_stage):
    global last_stage
    global blink_type, blink_start, prev_trig, prev_thermal
    global booting_idx, connect_left, connect_right
    global connect_previous_left, connect_previous_right

    if new_stage == last_stage:
        return

    # 1) Wipe the LCD and our cache
    clear_screen_full()

    # 2) Reset *all* per-page state
    #    – Blink-row
    blink_type  = [None] * 8
    blink_start = [0.0] * 8
    prev_trig   = [False] * 8
    prev_thermal= [False] * 8

    #    – Boot animation
    booting_idx             = 0

    #    – Connection animation
    connect_left          = 0
    connect_right         = 20 - 1  # 19
    connect_previous_left = None
    connect_previous_right= None

    # 3) Immediately *draw* the first frame of whatever page we’re now in
    if new_stage == "booting":
        # draw first booting frame (border + text)
        booting_animation_frame()

    elif new_stage == "initializing":
        # show the static "initializing" text + first connection frame
        connect_animation_frame()

    elif new_stage == "connected":
        # build & draw the *entire* connected screen in one go
        screen = build_screen_from_system(current_data)
        update_screen(screen)

    # 4) Remember for next time
    last_stage = new_stage



def main():
    global current_data, last_stage, watchdogActive, flash_on, last_flash_tick

    # Ensure status file exists
    if not os.path.exists(STATUS_FILE):
        with open(STATUS_FILE, "w") as f:
            json.dump(current_data, f)

    # Initialize LCD
    if not init_lcd():
        print("LCD init failed after waiting for I2C")
        return
    clear_screen_full()

    # Initial load
    time.sleep(0.05)
    initial = load_status_file()
    if initial:
        current_data.update(initial)

    last_stage = None
    last_mtime  = 0.0

    # State for blinking & trouble
    prev_trig       = [False]*8
    prev_thermal    = [False]*8
    blink_start     = [0.0]*8
    blink_type      = [None]*8
    last_toggle     = time.monotonic()
    showing_trouble = False

    try:
        while True:
            now = time.monotonic()
            if now - last_flash_tick >= FLASH_INTERVAL:
                last_flash_tick = now
                flash_on        = not flash_on

            # — 1) Reload JSON & detect changes —
            try:
                mtime = os.path.getmtime(STATUS_FILE)
                if mtime != last_mtime:
                    last_mtime = mtime
                    with open(STATUS_FILE) as f:
                        status = json.load(f)
                    current_data.clear()
                    current_data.update(status)
                    # start any new blink sessions
                    trig_list  = status.get("trig",    [False]*8)
                    therm_list = status.get("thermal", [False]*8)
                    for i in range(8):
                        if trig_list[i] and not prev_trig[i]:
                            blink_start[i] = now
                            blink_type[i]  = "camera"
                        if therm_list[i] and not prev_thermal[i]:
                            blink_start[i] = now
                            blink_type[i]  = "thermal"
                    prev_trig    = trig_list
                    prev_thermal = therm_list
            except FileNotFoundError:
                pass

            # — 2) Figure out which page we should be on —
            page = determine_page(current_data)

            # — 3) Handle a stage‐change *once* on entry to a new page —
            if page != last_stage:
                clear_screen_full()
                # reset page‐specific state
                blink_type[:]   = [None]*8
                blink_start[:]  = [0.0]*8
                prev_trig[:]    = [False]*8
                prev_thermal[:] = [False]*8
                showing_trouble = False
                # draw the *entire* new page
                if page == "booting":
                    booting_animation_frame()
                elif page == "initializing":
                    connect_animation_frame()
                elif page == "connected":
                    # static layout: lines 1–4
                    screen = build_screen_from_system(current_data)
                    update_screen(screen)
                last_stage = page

            # — 4) Run per-frame logic for the current page —
            if page == "booting":
                booting_animation_frame()

            elif page == "initializing":
                connect_animation_frame()

            else:  # connected
                # a) blinking TRIG row
                for i in range(8):
                    elapsed = now - blink_start[i]
                    if blink_type[i] is None or elapsed > BLINK_HOLD:
                        char = "\x01"   # ❌
                    else:
                        # synchronized flash across all channels
                        if not flash_on:
                            char = " "
                        else:
                            char = "\x03" if blink_type[i] == "camera" else "\x02"
                    lcd.cursor_pos = (3, 5 + i*2)
                    lcd.write_string(char)

                # b) trouble‐flash if active
                if watchdogActive:
                    if now - last_toggle > 2.5:
                        last_toggle     = now
                        showing_trouble = not showing_trouble
                    if showing_trouble:
                        update_screen(build_trouble_screen())
                    else:
                        # restore the normal static lines 1–3
                        stat = build_screen_from_system(current_data)
                        update_screen(stat)

            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
