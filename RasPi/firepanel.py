#!/usr/bin/env python3


import serial
import json
import os
import sys
import time
import socket
import datetime
import threading
import logging
from logging.handlers import TimedRotatingFileHandler


# ——— Logging Setup ———
logger = logging.getLogger("firepanel")
logger.propagate = False           # don’t pass to root logger

logger.setLevel(logging.DEBUG)     # capture everything; filter on handlers
# logger.setLevel(logging.INFO)
# logger.setLevel(logging.WARNNG)
# logger.setLevel(logging.ERROR)
# logger.setLevel(logging.EXCEPTION)

DEBUG_FILE = False
DEBUG_TRIG = False
DEBUG_STATUS = False
DEBUG_HEARTBEAT = True
SHOW_ALERTS = False

SERIAL_PORT = "/dev/ttyS0" 
BAUD_RATE = 115200
# How long (in seconds) before we consider a partial frame “stale”
FRAME_TIMEOUT    = 1.0
# Maximum bytes we’ll accumulate between markers before we give up
MAX_FRAME_LENGTH = 1024


STATUS_FILE = "/home/Dale/firepanel/RasPi/system_status.json"
WATCHDOG_FILE = "/home/Dale/firepanel/RasPi/watchdog_status.json"
SOCKET_PATH = "/home/Dale/firepanel/RasPi/firepanel.sock"

last_heartbeat = time.time()
watchdog_timeout = 10  # seconds
watchdog_active = False
missed_heartbeats = 0     # how many seconds-loops in a row we’ve been past timeout

trig = {}
camera_trigger_times = [0] * 8  # Track last set time for each channel trig
TRIG_HOLD_TIME = 20  # 3 minutes

thermal_trigger_times = [0] * 8
THERMAL_HOLD_TIME = 20  # seconds

stop_event = threading.Event()



# ——— Logging Setup ———
LOG_DIR  = os.path.expanduser("/home/Dale/firepanel/RasPi/logs")
LOG_FILE = os.path.join(LOG_DIR, "firepanel.log")

# Ensure log directory exists
try:
    os.makedirs(LOG_DIR, exist_ok=True)
except Exception as e:
    # If we can’t create it, fallback to /tmp
    LOG_DIR  = "/tmp/firepanel-logs"
    LOG_FILE = os.path.join(LOG_DIR, "firepanel.log")
    os.makedirs(LOG_DIR, exist_ok=True)



# — File Handler: rotate every Monday at 00:01, keep ~52 weeks of logs ——
file_handler = TimedRotatingFileHandler(
    LOG_FILE,
    when="W0",           # weekly on Monday
    interval=4,          # every 4 weeks
    backupCount=12,      # keep the last 12 → ~48 weeks (~11 months)
    encoding="utf-8",
    utc=False,
    atTime=datetime.time(hour=0, minute=1) )



# compress rotated files to .gz
file_handler.namer = lambda name: name + ".gz"
def rotator(source, dest):
    import gzip, shutil
    with open(source, "rb") as sf, gzip.open(dest, "wb") as df:
        shutil.copyfileobj(sf, df)
    os.remove(source)
file_handler.rotator = rotator

file_handler.setLevel(logging.DEBUG)  # log everything to file
file_handler.setFormatter(
    logging.Formatter("[%(asctime)s] %(levelname)-5s %(message)s",
                      datefmt="%Y-%m-%d %H:%M:%S") )
logger.addHandler(file_handler)

# — Console Handler ——
console_handler = logging.StreamHandler()
console_handler.setLevel(logging.INFO)   # only INFO+ to console
console_handler.stream = open(sys.stdout.fileno(), 'w', encoding='utf-8', closefd=False)
console_handler.setFormatter(
    logging.Formatter("%(asctime)s %(levelname)-5s %(message)s",
                      datefmt="%H:%M:%S") )

logger.addHandler(console_handler)



# ——— Send JSON Frame To Arduino ———
def send_json(ser, obj):
    try:
        frame = "<" + json.dumps(obj) + ">"
        ser.write(frame.encode('utf-8'))
        ser.flush()
        if DEBUG_STATUS:
            if SHOW_ALERTS:
                logger.debug("Sent JSON frame: %s", frame)
    except Exception:
        logger.exception("Failed to send JSON frame")


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


# ——— Handle Incoming Frame ———
def handle_frame(frame):
    global last_heartbeat, trig

    try:
        parsed = json.loads(frame)
        pretty = json.dumps(parsed, indent=2, sort_keys=True)
        #logger.debug("Raw frame received:\n%s", pretty)

        data = json.loads(frame)
        msg_type = data.get("type")
        if msg_type == "channel_trigger":
            # Incoming camera trigger (before we know if the output fired)
            last_heartbeat = time.time()
            ch = data.get("channel")
            if ch and 1 <= ch <= 8:
                idx = ch - 1

                # mark the raw trigger
                current = load_status_file()
                trig = current.get("trig", [False]*8)
                trig[idx] = True
                camera_trigger_times[idx] = time.time()
                update_status_fields(trig=trig)

                # load the current mode straight from disk
                status = load_status_file()
                mode   = status.get("mode", "UNKNOWN")

                logger.info("🔥 Camera Trigger Detected 🔥 - Channel %d | Mode: %s", ch, mode)


        elif msg_type == "output_confirm":
            # Confirmation from Arduino that the output actually fired
            ch = data.get("channel")
            dummy = data.get("dummy", False)

            if ch and 1 <= ch <= 8:
                idx = ch - 1

                # mark the confirmed trigger
                current = load_status_file()

                # load the current mode straight from disk
                trig    = current.get("trig",    [False]*8)
                confirm = current.get("confirm", [False]*8)

                trig[idx]    = True
                confirm[idx] = True

                update_status_fields(trig=trig, confirm=confirm)

                if dummy:
                    logger.info("✅ Confirmed Dummy Output  ✅ - Channel %d", ch)
                else:
                    logger.info("✅ Confirmed Live Output ✅ - Channel %d", ch)


        elif msg_type == "trigger_thermal":
            ch = data.get("channel")
            if ch is not None and 1 <= ch <= 8:
                ch_index = ch - 1
                current = load_status_file()

                thermal = current.get("thermal", [False] * 8)
                thermal[ch_index] = True
                thermal_trigger_times[ch_index] = time.time()

                update_status_fields(thermal=thermal)
                logger.info("♨️ Thermal Trigger Received ♨️ - Channel %d", ch)


        elif msg_type == "ack":
            cmd = data.get("command", "<none>")
            if cmd == "trigger_thermal":
                ch = data.get("channel", 0)
                if 1 <= ch <= 8:
                    logger.info("✅ Thermal Trigger Confirmed ✅ - Channel %d", ch)
            else:
                # generic acks—just log them, no status‐file writes
                logger.debug("Received ACK for command %r", cmd)
            # and then return, so we don't fall into the final else
            return


        elif msg_type == "data":
            system = data
            last_heartbeat = time.time()

            # Scalars
            mode        = system.get("systemModeStr", "UNKNOWN")
            avg_temp    = system.get("avgTemp", None)
            break_glass = system.get("breakGlass", False)
            temp_alert  = system.get("tempAlert", False)
            psu1_uv     = system.get("psu1UnderVolt", False)
            psu2_uv     = system.get("psu2UnderVolt", False)

            # Bit-mask unpacking
            cam_mask    = system.get("cameraMask", 0)
            th_mask     = system.get("thermalMask", 0)
            cab_mask    = system.get("cableMask", 0)
            conf_mask   = system.get("confirmMask", 0)

            # Unpack the bitmasks into lists of bools
            incoming_cam  = [(cam_mask  >> i) & 1 == 1 for i in range(8)]
            incoming_th   = [(th_mask   >> i) & 1 == 1 for i in range(8)]
            incoming_conn = [(cab_mask  >> i) & 1 == 1 for i in range(8)]
            incoming_conf = [(conf_mask >> i) & 1 == 1 for i in range(8)]
            
            # Log the data
            # logger.debug(
            #     "  DATA Received:\n"
            #     " System Status\n"
            #     "  Mode=%s, Temp=%.1f, BG=%s, TempA=%s, PSU1_UV=%s, PSU2_UV=%s \n"
            #     " Masks:\n"
            #     "  Camera:  %s\n"
            #     "  Thermal: %s\n"
            #     "  Confirm: %s\n"
            #     "  Connect: %s\n",
            #     mode, 
            #     avg_temp, 
            #     break_glass, 
            #     temp_alert, 
            #     psu1_uv, 
            #     psu2_uv, 
            #     f"{cam_mask:08b}",
            #     f"{th_mask:08b}",
            #     f"{conf_mask:08b}",
            #     f"{cab_mask:08b}"
            # )


            # Merge triggers with existing state as before:
            existing = load_status_file()
            current_trig = existing.get("trig", [False]*8)
            merged_trig = [incoming_cam[i] or current_trig[i] for i in range(8)]
            # (Thermal stays separate now, if you’re displaying separately)
            existing_therm = existing.get("thermal", [False]*8)
            merged_therm = [incoming_th[i] or existing_therm[i] for i in range(8)]

            update_status_fields(
                stage="connected",
                mode=mode,
                avgTemp=avg_temp,
                breakGlass=break_glass,
                tempAlert=temp_alert,
                psu1UnderVolt=psu1_uv,
                psu2UnderVolt=psu2_uv,
                conn=incoming_conn,
                trig=merged_trig,
                thermal=merged_therm,
                confirm=incoming_conf
            )


        elif msg_type == "alert":
            last_heartbeat = time.time()
            alert_type = data.get("alertType") or data.get("subtype") or data.get("type")
            logger.debug("Alert %s received: %s", alert_type.upper(), data)

            if alert_type == "mode_change":
                mode = data.get("mode", "UNKNOWN")
                update_status_fields(mode=mode)
                logger.info("🗝️  System mode changed to %s 🗝️", mode)

            elif alert_type == "psu_undervoltage":
                psu = data.get("psu")
                voltage = data.get("voltage")
                logger.warning("🪫  PSU %d undervoltage: %.2f V 🪫", psu, voltage)
            
            elif alert_type == "psu_restored":
                psu = data.get("psu")
                voltage = data.get("voltage")
                logger.info("🔋  PSU %d voltage restored: %.2f V 🔋", psu, voltage)

            elif alert_type == "break_glass":
                ch = data.get("channel")
                logger.warning("🥊 Break Glass Triggered! 🥊")

            elif alert_type == "temperature_alert":
                temp = data.get("avgTemp", "--")
                logger.warning("♨️ High temperature alert: %s°C ♨️", temp)

            elif alert_type == "thermal_trigger":
                ch = data.get("channel")
                temp = data.get("temperature", "--")
                logger.warning("♨️ Thermal camera alarm on channel %d at %s°C ♨️", ch, temp)

            elif alert_type == "camera_trigger":
                ch = data.get("channel")
                logger.warning("🔥 Camera trigger on channel %d 🔥", ch)

            # Optionally: reflect alert visually in status file
            update_status_fields(lastAlert=data)


        elif msg_type == "heartbeat":
            last_heartbeat = time.time()

            if DEBUG_HEARTBEAT:
                logger.debug("Received heartbeat")

            # Ensure screen is set to connected mode
            update_status_fields(stage="connected")

            # Send get_data request in response to heartbeat
            send_json(ser, {"type": "get_data"})
        

        elif msg_type == "self_test":
            rpt = data
            logger.info(
                "\n\n ➤  Startup Arduino SelfTest ➤\n Firmware Version: %s\n"
                " Self-Test: %s\n"
                " Mode: %s\n"
                " PSU #1: %s\n"
                " PSU #2: %s\n"
                " Channels: %s\n"
                " I2C OK: %s%s\n",
                rpt["fwVersion"],
                rpt["passed"],
                rpt["systemMode"],
                rpt["psu1Voltage"],
                rpt["psu2Voltage"],
                rpt["channelCount"],
                rpt["i2cOk"],
                f", freeMem={rpt['freeMemory']}B" if "freeMemory" in rpt else ""
            )


        else:
            logger.debug("Ignoring unhandled frame for status: %s", frame)


    except json.JSONDecodeError:
        logger.debug("Raw frame received (invalid JSON): %s", frame)


# ——— Load Status File ———
def load_status_file():
    try:
        with open(STATUS_FILE, "r") as f:
            return json.load(f)
    except Exception:
        return {
            "stage": "booting",
            "mode": "UNKNOWN",
            "avgTemp": None,
            "conn": [False]*8,
            "trig": [False]*8,
            "thermal": [False]*8
        }


# —– Update Status Fields ———
def update_status_fields(**updates):
    try:
        if DEBUG_STATUS:
            logger.debug("update_status_fields() called")

        # Load existing file or fallback
        if os.path.exists(STATUS_FILE):
            with open(STATUS_FILE, 'r') as f:
                current = json.load(f)
            if DEBUG_STATUS:
                logger.debug("Loaded current status file")
        else:
            current = {
                "stage":     "booting",
                "mode":      "UNKNOWN",
                "avgTemp":   None,
                "conn":      [False]*8,
                "trig":      [False]*8,
                "thermal":   [False]*8,
                "confirm":   [False]*8,

            }
            if DEBUG_STATUS:
                logger.debug("Created new default status")

        # Apply updates
        current.update(updates)

        # Write back prettified
        with open(STATUS_FILE, 'w') as f:
            json.dump(current, f, indent=2)

        if DEBUG_STATUS:
            logger.debug("Status file updated with fields: %s", list(updates.keys()))

    except Exception:
        logger.exception("Failed to update system_status.json")


# ——— Camera Trigger Cleanup Loop ———
def trig_cleanup_loop():
    global trig
    while not stop_event.is_set():
        now = time.time()
        updated = False
        status = load_status_file()
        trig = status.get("trig", [False] * 8)

        for i in range(8):
            if trig[i] and (now - camera_trigger_times[i] > TRIG_HOLD_TIME):
                trig[i] = False
                updated = True

        if updated:
            update_status_fields(trig=trig)
            if DEBUG_TRIG:
                logger.debug("Cleared expired camera trig flags")

        time.sleep(10)


# ——— Thermal Cleanup Loop ———
def thermal_cleanup_loop():
    global thermal_trigger_times
    while not stop_event.is_set():
        now = time.time()
        status = load_status_file()
        thermal = status.get("thermal", [False]*8)
        updated = False

        for i in range(8):
            if thermal[i] and (now - thermal_trigger_times[i] > THERMAL_HOLD_TIME):
                thermal[i] = False
                updated = True

        if updated:
            update_status_fields(thermal=thermal)
            logger.debug("Cleared expired thermal flags")

        time.sleep(10)


# ——— Write Watchdog Status to File ———
def write_watchdog_status(payload):

    #payload can be either:
    #  - a dict: written as-is
    #  - a string: wrapped as {"state": payload, "missed": missed_heartbeats}
    
    global missed_heartbeats

    if isinstance(payload, str):
        obj = {"state": payload, "missed": missed_heartbeats}
    else:
        obj = payload

    try:
        with open(WATCHDOG_FILE, "w") as f:
            json.dump(obj, f)
    except Exception:
        logger.exception("Failed to write watchdog_status.json")


# — Watchdog Loop - Check Arduino Heartbeat —
def watchdog_loop():
    """
    Every second we compare time since last_heartbeat to watchdog_timeout.
    - 1st time over: warning
    - 3rd consecutive time: critical
    - on recovery: info + reset
    """
    global watchdog_active, missed_heartbeats

    while not stop_event.is_set():
        now = time.time()
        elapsed = now - last_heartbeat

        if elapsed > watchdog_timeout:
            missed_heartbeats += 1

            # first miss
            if missed_heartbeats == 1:
                logger.warning(
                    "Watchdog: heartbeat overdue by %.1fs (1/%d)",
                    elapsed, watchdog_timeout
                )
                watchdog_active = True
                write_watchdog_status({"state": "warning", "missed": missed_heartbeats})

            # escalation on the 3rd miss
            elif missed_heartbeats == 3:
                logger.critical(
                    "Watchdog: heartbeat missed %d times (>%ds each)",
                    missed_heartbeats, watchdog_timeout
                )
                write_watchdog_status({"state": "critical", "missed": missed_heartbeats})

        else:
            # recovered: reset everything
            if watchdog_active or missed_heartbeats > 0:
                logger.info(
                    "Watchdog recovered after %d missed heartbeats (elapsed=%.1fs)",
                    missed_heartbeats, elapsed
                )
                watchdog_active = False
                write_watchdog_status({"state": "ok", "missed": 0})

            missed_heartbeats = 0

        time.sleep(1)


#  Socket Listener From Nodered Thermal Trigger
def socket_command_listener(ser):
    # Clean up old socket if needed
    try:
        os.unlink(SOCKET_PATH)
    except FileNotFoundError:
        pass

    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(SOCKET_PATH)
    server.listen(1)
    logger.info("Socket listening on %s", SOCKET_PATH)


    while not stop_event.is_set():
        try:
            conn, _ = server.accept()
            with conn:
                data = conn.recv(1024).decode().strip()
                logger.debug("Socket received command %r", data)

                if data == "ping":
                    conn.sendall(b"OK\n")
                    continue

                elif data == "self-test":
                    logger.info("🔧 Selftest requested over socket")
                    # forward to Arduino
                    send_json(ser, {"type": "selftest"})
                    # acknowledge immediately
                    conn.sendall(b"OK: selftest started\n")

                elif data.startswith("thermal_trigger:"):
                    parts = data.split(":", 1)

                    try:
                        ch = int(parts[1])
                    except (ValueError, IndexError):
                        conn.sendall(b"ERR: Bad format\n")
                    else:
                        if 1 <= ch <= 8:
                            idx = ch - 1

                            # 1) update status
                            update_status_fields( thermal=[i == idx for i in range(8)] )
                            # 2) log with ch defined
                            logger.info("🌡️ Thermal Alarm Triggered – Channel %d", ch)
                            # 3) acknowledge back to Node-RED
                            conn.sendall(b"OK\n")
                            # 4) forward to Arduino
                            cmd = {"type": "trigger_thermal", "channel": ch}
                            send_json(ser, cmd)
                            logger.debug("Forwarded to Arduino: %s", cmd)
                        else:
                            conn.sendall(b"ERR: Invalid channel\n")
                else:
                    conn.sendall(b"ERR: Unknown command\n")
        except Exception:
            logger.exception("Socket listener error")


# ——— ask Arduino for a full data frame ASAP so avgTemp gets set quickly ———
def prime_data_request():
    time.sleep(0.5)  # give threads a moment to settle
    send_json(ser, {"type": "get_data"})


# —– Main Program Loop ———
if __name__ == "__main__":
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5, exclusive=True)
        ser.reset_input_buffer()
    except Exception:
        logger.exception("FATAL: could not open serial port, exiting")
        sys.exit(1)

    logger.info("Firepanel controller started")

    #update_status_fields(stage="booting", mode="BOOTING", conn=[False]*8, trig=[False]*8, thermal=[False]*8)
    
    # initialise the status file so LCD has something to read
    update_status_fields(
        stage="booting",
        mode="BOOTING",
        conn=[False]*8,
        trig=[False]*8,
        thermal=[False]*8,
        avgTemp=29.0,
        breakGlass=False,
        tempAlert=False,
        psu1UnderVolt=False,
        psu2UnderVolt=False,
        confirm=[False]*8
    )

    serial_thread = threading.Thread(target=read_from_serial, args=(ser,))
    serial_thread.start()

    trig_thread = threading.Thread(target=trig_cleanup_loop)
    trig_thread.start()

    thermal_thread = threading.Thread(target=thermal_cleanup_loop)
    thermal_thread.start()

    watchdog_thread = threading.Thread(target=watchdog_loop)
    watchdog_thread.start()

    socket_thread = threading.Thread(target=socket_command_listener, args=(ser,), daemon=True)
    socket_thread.start()
    logger.debug("Socket listener thread started")
    print("[MAIN] socket_thread started")

    threading.Thread(target=prime_data_request, daemon=True).start()

    try:
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        stop_event.set()
        serial_thread.join()
        watchdog_thread.join()
        trig_thread.join()
        thermal_thread.join()
        socket_thread.join()
        ser.close()
        try:
            os.unlink(SOCKET_PATH)
        except Exception:
            pass
        logger.info("Firepanel controller stopped")
