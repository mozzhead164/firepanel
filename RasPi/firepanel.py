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

# TEST 3

DEBUG_FILE = False
DEBUG_TRIG = False
DEBUG_STATUS = False
DEBUG_HEARTBEAT = False
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

HANDSHAKE_PAYLOAD = {"type": "handshake", "payload": "HELLO_PI"}
ACK_HANDSHAKE = {"type": "ack", "command": "handshake"}

handshake_complete = False
last_heartbeat = time.time()
watchdog_timeout = 10  # seconds
watchdog_active = False

trig = {}
camera_trigger_times = [0] * 8  # Track last set time for each channel trig
TRIG_HOLD_TIME = 20  # 3 minutes

thermal_trigger_times = [0] * 8
THERMAL_HOLD_TIME = 20  # seconds

stop_event = threading.Event()



# ——— Logging Setup ———
LOG_DIR  = os.path.expanduser("~/firepanel/RasPi/logs")
LOG_FILE = os.path.join(LOG_DIR, "firepanel.log")

# Ensure log directory exists
try:
    os.makedirs(LOG_DIR, exist_ok=True)
except Exception as e:
    # If we can’t create it, fallback to /tmp
    LOG_DIR  = "/tmp/firepanel-logs"
    LOG_FILE = os.path.join(LOG_DIR, "firepanel.log")
    os.makedirs(LOG_DIR, exist_ok=True)


# ——— Logger Setup ———
logger = logging.getLogger("firepanel")

logger.setLevel(logging.DEBUG)     # capture everything; filter on handlers
# logger.setLevel(logging.INFO)
# logger.setLevel(logging.WARNNG)
# logger.setLevel(logging.ERROR)
# logger.setLevel(logging.EXCEPTION)

logger.propagate = False           # don’t pass to root logger


# — File Handler: rotate every Monday at 00:01, keep ~52 weeks of logs ——
file_handler = TimedRotatingFileHandler(
    LOG_FILE,
    when="W0",           # weekly on Monday
    interval=4,          # every 4 weeks
    backupCount=12,      # keep the last 12 → ~48 weeks (~11 months)
    encoding="utf-8",
    utc=False,
    atTime=datetime.time(hour=0, minute=1)
)



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
                      datefmt="%Y-%m-%d %H:%M:%S")
)
logger.addHandler(file_handler)

# — Console Handler ——
# console_handler = logging.StreamHandler()
# console_handler.setLevel(logging.INFO)   # only INFO+ to console
# console_handler.setFormatter(
#     logging.Formatter("%(asctime)s %(levelname)-5s %(message)s",
#                       datefmt="%H:%M:%S")
# )
# logger.addHandler(console_handler)




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




def handle_frame(frame):
    global handshake_complete, last_heartbeat, trig

    try:
        parsed = json.loads(frame)
        pretty = json.dumps(parsed, indent=2, sort_keys=True)
        logger.debug("Raw frame received:\n%s", pretty)

        data = json.loads(frame)
        msg_type = data.get("type")


        if msg_type == "handshake" and data.get("payload") == "HELLO_ATMEGA":
            logger.debug("Received handshake from Arduino")
            send_json(ser, HANDSHAKE_PAYLOAD)


        elif msg_type == "channel_trigger":
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
                logger.info("🔥 Camera Trigger Detected - Channel %d 🔥", ch)


        elif msg_type == "output_confirm":
            # Confirmation from Arduino that the output actually fired
            ch = data.get("channel")
            dummy = data.get("dummy", False)
            if ch and 1 <= ch <= 8:
                idx = ch - 1

                # mark the confirmed trigger
                current = load_status_file()
                confirm = current.get("confirm", [False]*8)
                confirm[idx] = True
                update_status_fields(confirm=confirm)

                if dummy:
                    logger.info("✅ Confirmed Dummy Output - Channel %d ✅", ch)
                else:
                    logger.info("✅ Confirmed Live Output - Channel %d ✅", ch)


        elif msg_type == "trigger_thermal":
            ch = data.get("channel")
            if ch is not None and 1 <= ch <= 8:
                ch_index = ch - 1
                current = load_status_file()

                thermal = current.get("thermal", [False] * 8)
                thermal[ch_index] = True
                thermal_trigger_times[ch_index] = time.time()

                update_status_fields(thermal=thermal)
                logger.info("Thermal trigger received on channel %d", ch)


        elif msg_type == "ack":
            cmd = data.get("command", "<none>")
            if cmd == "handshake":
                # your current handshake logic
                update_status_fields(stage="connected", mode="ARMED")
                write_watchdog_status(False)
                handshake_complete = True
                last_heartbeat = time.time()
                logger.info("Received handshake ACK from Arduino")
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
            logger.debug(
                "  DATA Received:\n"
                " System Status\n"
                "  Mode=%s, Temp=%.1f, BG=%s, TempA=%s, PSU1_UV=%s, PSU2_UV=%s \n"
                " Masks:\n"
                "  Camera:  %s\n"
                "  Thermal: %s\n"
                "  Confirm: %s\n"
                "  Connect: %s\n",
                mode, 
                avg_temp, 
                break_glass, 
                temp_alert, 
                psu1_uv, 
                psu2_uv, 
                f"{cam_mask:08b}",
                f"{th_mask:08b}",
                f"{conf_mask:08b}",
                f"{cab_mask:08b}"
            )


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
                logger.info("System mode changed to %s", mode)

            elif alert_type == "psu_undervoltage":
                psu = data.get("psu")
                voltage = data.get("voltage")
                logger.warning("PSU%d undervoltage: %.2f V", psu, voltage)
            
            elif alert_type == "psu_restored":
                psu = data.get("psu")
                voltage = data.get("voltage")
                logger.info("PSU%d voltage restored: %.2f V", psu, voltage)

            elif alert_type == "break_glass":
                ch = data.get("channel")
                logger.info("Break Glass Triggered!")

            elif alert_type == "temperature_alert":
                temp = data.get("avgTemp", "--")
                logger.warning("High temperature alert: %s°C", temp)

            elif alert_type == "thermal_trigger":
                ch = data.get("channel")
                temp = data.get("temperature", "--")
                logger.warning("Thermal camera alarm on channel %d at %s°C", ch, temp)

            elif alert_type == "camera_trigger":
                ch = data.get("channel")
                logger.warning("Camera trigger on channel %d", ch)

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
        

        else:
            logger.debug("Ignoring unhandled frame for status: %s", frame)


    except json.JSONDecodeError:
        logger.debug("Raw frame received (invalid JSON): %s", frame)




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


def write_watchdog_status(active):
    try:
        with open(WATCHDOG_FILE, "w") as f:
            json.dump({"active": active}, f)
    except Exception:
        logger.exception("Failed to write watchdog_status.json")


def watchdog_loop():
    global watchdog_active
    while not stop_event.is_set():
        now = time.time()
        if handshake_complete and (now - last_heartbeat > watchdog_timeout):
            if not watchdog_active:
                logger.warning("Watchdog timeout – activating trouble screen")
                watchdog_active = True
                write_watchdog_status(True)
        elif watchdog_active and (now - last_heartbeat <= watchdog_timeout):
            logger.info("Watchdog recovered – clearing trouble state")
            watchdog_active = False
            write_watchdog_status(False)
        time.sleep(1)


def socket_command_listener():
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
                t0 = time.time()
                logger.debug("Socket received command %r at %.3fs", data, t0)

                if data.startswith("thermal_trigger:"):
                    ch1 = int(data.split(":",1)[1])
                    if 1 <= ch1 <= 8:
                        idx = ch1 - 1

                        # Update status file immediately
                        update_status_fields(thermal=[i == idx for i in range(8)])
                        logger.debug("Status file updated for thermal trigger at %.3fs", time.time())

                        # Prepare and send to Arduino
                        msg = {"type":"trigger_thermal","channel":ch1}
                        frame = json.dumps(msg)
                        t1 = time.time()
                        ser.write(f'<{frame}>'.encode())
                        ser.flush()
                        t2 = time.time()
                        logger.debug(
                            "Wrote to serial at %.3fs (took %.3f s): %s",
                            t2, t2 - t1, frame
                        )

                        conn.sendall(b"OK\n")
                    else:
                        conn.sendall(b"ERR: Invalid channel\n")
                else:
                    conn.sendall(b"ERR: Unknown command\n")
        except Exception as e:
            logger.error("Socket listener error", exc_info=e)




if __name__ == "__main__":
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5, exclusive=True)
        ser.reset_input_buffer()
    except Exception:
        logger.exception("FATAL: could not open serial port, exiting")
        sys.exit(1)

    logger.info("Firepanel controller started")
    update_status_fields(stage="booting", mode="BOOTING", conn=[False]*8, trig=[False]*8, thermal=[False]*8)

    serial_thread = threading.Thread(target=read_from_serial, args=(ser,))
    serial_thread.start()

    trig_thread = threading.Thread(target=trig_cleanup_loop)
    trig_thread.start()

    thermal_thread = threading.Thread(target=thermal_cleanup_loop)
    thermal_thread.start()

    watchdog_thread = threading.Thread(target=watchdog_loop)
    watchdog_thread.start()

    socket_thread = threading.Thread(
        target=socket_command_listener,
        daemon=True
    )
    socket_thread.start()
    logger.debug("Socket listener thread started")
    print("[MAIN] socket_thread started")

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
