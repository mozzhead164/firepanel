import os
import sys
import json
import time
from time import sleep
from threading import Event
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from RPLCD.i2c import CharLCD



STATUS_FILE = "/home/Dale/firepanel/RasPi/system_status.json"

lcd_cache = [[' ' for _ in range(20)] for _ in range(4)]
current_data = {
    'stage': 'booting',
    'mode': 'BOOTING',
    'conn': [False]*8,
    'trig': [False]*8,
    'thermal': [False]*8  
}


BOOT_DURATION = 10.0  # seconds
start_time = time.time()
heartbeat_received = False

last_stage = None
booting_idx = 0
handshake_left = 0
handshake_right = 19
handshake_previous_left = None
handshake_previous_right = None

watchdogActive = False
last_toggle_time = 0
showing_trouble = False
status_changed = Event()

flash_tick_on = True
last_flash_toggle = time.time()
flash_interval = 0.33
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
    global lcd
    try:
        lcd = CharLCD('PCF8574', I2C_ADDRESS, cols=20, rows=4)
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


def check_and_handle_stage_change(current_stage):
    global last_stage
    if current_stage != last_stage:
        clear_screen_full()
        last_stage = current_stage


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


def update_flashing_triggers():
    trig = current_data.get("trig", [False] * 8)
    thermal = current_data.get("thermal", [False] * 8)

    for i in range(8):
        col = 5 + i * 2  # position on LCD line
        if trig[i]:
            char = '\x03' if flash_tick_on else ' '
        elif thermal[i]:
            char = '\x02' if flash_tick_on else ' '
        else:
            continue  # no update needed

        lcd_write_safe((3, col), char)
        lcd_cache[3][col] = char  # keep cache in sync


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

    if os.path.exists("/home/Dale/firepanel/RasPi/watchdog_status.json"):
        try:
            with open("/home/Dale/firepanel/RasPi/watchdog_status.json", 'r') as f:
                status = json.load(f)
                watchdogActive = status.get('active', False)
        except Exception:
            watchdogActive = False

    stage = current_data.get('stage', 'unknown')

    # Always handle screen clear if stage changed
    if stage != last_stage:
        check_and_handle_stage_change(stage)

    # Always update screen if connected
    if stage == "connected":
        screen = build_screen_from_system(current_data)
        print("[DEBUG] Connected screen content:", screen)
        update_screen(screen)



def main():
    global current_data, last_toggle_time, showing_trouble
    global last_flash_toggle, flash_tick_on

    if not os.path.exists(STATUS_FILE):
        with open(STATUS_FILE, 'w') as f:
            json.dump(current_data, f)
    init_lcd()
    sleep(0.05)
    current_data = load_status_file() or current_data
    handle_status_file_update()

    event_handler = StatusChangeHandler()
    observer = Observer()
    observer.schedule(event_handler, path="/home/Dale/firepanel/RasPi/", recursive=False)
    observer.start()

    try:
        while True:
            if status_changed.is_set():
                handle_status_file_update()
                status_changed.clear()

            now = time.time()
            if current_data.get("stage") == "connected":
                if now - last_flash_toggle >= flash_interval:
                    last_flash_toggle = now
                    flash_tick_on = not flash_tick_on
                    update_flashing_triggers()

            stage = current_data.get('stage', 'booting')
            
            if stage == 'booting':
                booting_animation_frame()
            
            elif stage == 'handshake':
                handshake_animation_frame()
            
            elif stage == 'connected' and watchdogActive:
                now = time.time()
                if now - last_toggle_time > 2.5:
                    last_toggle_time = now
                    showing_trouble = not showing_trouble
            
            sleep(0.01)

    except KeyboardInterrupt:
        observer.stop()
    observer.join()


if __name__ == "__main__":
    main()
