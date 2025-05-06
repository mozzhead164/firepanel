#!/usr/bin/env python3


import serial
import json
import os
import time
import socket
import threading

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
LOG_FILE = "/home/Dale/firepanel/RasPi/firepanel.log"
SOCKET_PATH = "/home/Dale/firepanel/RasPi/firepanel.sock"

HANDSHAKE_PAYLOAD = {"type": "handshake", "payload": "HELLO_PI"}
ACK_HANDSHAKE = {"type": "ack", "command": "handshake"}

handshake_complete = False
last_heartbeat = time.time()
watchdog_timeout = 10  # seconds
watchdog_active = False

trig = {}
camera_trigger_times = [0] * 8  # Track last set time for each channel trig
TRIG_HOLD_TIME = 30  # 3 minutes

thermal_trigger_times = [0] * 8
THERMAL_HOLD_TIME = 30  # seconds

stop_event = threading.Event()


def write_log(entry):
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(LOG_FILE, "a") as f:
        f.write(f"[{timestamp}] {entry}\n")


def send_json(ser, obj):
    try:
        frame = "<" + json.dumps(obj) + ">"
        ser.write(frame.encode('utf-8'))
        ser.flush()
        if DEBUG_STATUS:
            if SHOW_ALERTS:
                write_log(f"SENT: {frame}")
    except Exception as e:
        write_log(f"[ERROR] Failed to send: {e}")


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
                    write_log("[SERIAL] Frame timeout, discarding partial frame")
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
                    # accumulate, but guard against runaway
                    buffer.append(ch)
                    # if len(buffer) > MAX_FRAME_LENGTH:
                    #     write_log(f"[SERIAL] Frame too long ({len(buffer)} bytes), resyncing")
                    #     buffer.clear()
                    #     in_frame = False

        except serial.SerialException as e:
            write_log(f"[ERROR] Serial exception: {e}. Reopening port in 1s.")
            try:
                ser.close()
            except Exception:
                pass
            time.sleep(1)
            try:
                ser.open()
                ser.reset_input_buffer()
                write_log("[SERIAL] Port reopened and buffer flushed")
            except Exception as reopen_err:
                write_log(f"[ERROR] Failed to reopen port: {reopen_err}")
        except Exception as e:
            write_log(f"[ERROR] Unexpected error in serial read loop: {e}")
            # small sleep to prevent busy‐spin on unexpected errors
            time.sleep(0.1)




def handle_frame(frame):
    global handshake_complete, last_heartbeat, trig

    try:
        # write_log(f"[DEBUG] Raw frame received: {frame}")

        data = json.loads(frame)
        msg_type = data.get("type")

        if msg_type == "handshake" and data.get("payload") == "HELLO_ATMEGA":
            write_log("Received handshake from Arduino")
            send_json(ser, HANDSHAKE_PAYLOAD)


        elif msg_type == "channel_trigger" or msg_type == "output_confirm":
            last_heartbeat = time.time()
            ch = data.get("channel")
            if ch is not None and 0 <= ch < 8:
                current_status = load_status_file()
                trig = current_status.get("trig", [False]*8)
                if ch is not None and 1 <= ch <= 8:
                    trig[ch-1] = True  # mark as triggered
                update_status_fields(trig=trig)
                write_log(f"[INFO] Channel {ch} triggered")


        elif msg_type == "trigger_thermal":
            ch = data.get("channel")
            if ch is not None and 1 <= ch <= 8:
                ch_index = ch - 1
                current = load_status_file()

                thermal = current.get("thermal", [False] * 8)
                thermal[ch_index] = True
                thermal_trigger_times[ch_index] = time.time()

                update_status_fields(thermal=thermal)
                write_log(f"[INFO] Thermal trigger received on channel {ch}")


        elif msg_type == "ack" and data.get("command") == "handshake":
            update_status_fields(stage="connected", mode="ARMED")
            write_watchdog_status(False)
            handshake_complete = True
            last_heartbeat = time.time()
            write_log("Received ACK for handshake")
            write_watchdog_status(False)


        elif msg_type == "data":
            system_data = data.get("data", {})
            last_heartbeat = time.time()

            mode = system_data.get("systemModeStr", "UNKNOWN")
            avg_temp = system_data.get("avgTemp", None)
            conn = [ch.get("cableConnected", False) for ch in system_data.get("channels", [])]

            incoming_trig = [ch.get("cameraTriggered", False) or ch.get("thermalTriggered", False) for ch in system_data.get("channels", [])]

            # Load existing trig values to merge with new ones
            existing_status = load_status_file()
            current_trig = existing_status.get("trig", [False] * 8)
            merged_trig = []

            for i in range(8):
                if incoming_trig[i]:
                    camera_trigger_times[i] = time.time()
                merged_trig.append(incoming_trig[i] or current_trig[i])

            update_status_fields(
                stage="connected",
                mode=mode,
                avgTemp=avg_temp,
                conn=conn,
                trig=merged_trig
            )

            # write_log("[DEBUG] Wrote merged status from data packet")


        elif msg_type == "alert":
            last_heartbeat = time.time()
            alert_type = data.get("alertType") or data.get("subtype") or data.get("type")
            write_log(f"[ALERT] {alert_type.upper()} received: {json.dumps(data)}")

            if alert_type == "mode_change":
                mode = data.get("mode", "UNKNOWN")
                update_status_fields(mode=mode)
                write_log(f"[INFO] Mode changed to: {mode}")

            elif alert_type == "psu_undervoltage":
                psu = data.get("psu")
                voltage = data.get("voltage")
                write_log(f"[ALERT] PSU{psu} Undervoltage: {voltage:.2f}V")
            
            elif alert_type == "psu_restored":
                psu = data.get("psu")
                voltage = data.get("voltage")
                write_log(f"[ALERT] PSU{psu} Voltage Restored: {voltage:.2f}V")

            elif alert_type == "break_glass":
                ch = data.get("channel")
                write_log(f"[ALERT] Break glass trigger on channel {ch}")

            elif alert_type == "temperature_alert":
                temp = data.get("avgTemp", "--")
                write_log(f"[ALERT] High Temperature Alert: {temp}C")

            elif alert_type == "thermal_trigger":
                ch = data.get("channel")
                temp = data.get("temperature", "--")
                write_log(f"[ALERT] Thermal camera trigger on channel {ch}, temp={temp}C")

            elif alert_type == "camera_trigger":
                ch = data.get("channel")
                write_log(f"[ALERT] Camera trigger on channel {ch}")

            # Optionally: reflect alert visually in status file
            update_status_fields(lastAlert=data)


        elif msg_type == "heartbeat":
            last_heartbeat = time.time()
            if DEBUG_HEARTBEAT:
                write_log("Received heartbeat")

            # Ensure screen is set to connected mode
            update_status_fields(stage="connected")

            # Send get_data request in response to heartbeat
            send_json(ser, {"type": "get_data"})
        

        else:
            update_status_fields(lastUnhandled=data)
            write_log(f"Unhandled message: {frame}")


    except json.JSONDecodeError:
        write_log(f"[ERROR] Failed to parse frame: {frame}")




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
            write_log("[DEBUG] update_status_fields() called")

        # Load existing file or fallback
        if os.path.exists(STATUS_FILE):
            with open(STATUS_FILE, 'r') as f:
                current = json.load(f)
            if DEBUG_STATUS:
                write_log("[DEBUG] Loaded current status file")
        else:
            current = {
                "stage": "booting",
                "mode": "UNKNOWN",
                "avgTemp": None,
                "conn": [False]*8,
                "trig": [False]*8
            }
            if DEBUG_STATUS:
                write_log("[DEBUG] Created new default status")

        # write_log(f"[DEBUG] Updates received: {json.dumps(updates, indent=2)}")

        # Apply updates
        current.update(updates)

        # Write back prettified
        with open(STATUS_FILE, 'w') as f:
            json.dump(current, f, indent=2)

        if DEBUG_STATUS:
            write_log(f"[DEBUG] Status file updated with fields: {list(updates.keys())}")

    except Exception as e:
        write_log(f"[ERROR] Failed to update status file: {e}")


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
                write_log("[DEBUG] Cleared expired trig flags")

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
            write_log("[DEBUG] Cleared expired thermal flags")

        time.sleep(10)


def write_watchdog_status(active):
    try:
        with open(WATCHDOG_FILE, "w") as f:
            json.dump({"active": active}, f)
    except Exception as e:
        write_log(f"[ERROR] Failed to write watchdog file: {e}")


def watchdog_loop():
    global watchdog_active
    while not stop_event.is_set():
        now = time.time()
        if handshake_complete and (now - last_heartbeat > watchdog_timeout):
            if not watchdog_active:
                write_log("[WARN] Watchdog timeout! Activating trouble screen.")
                watchdog_active = True
                write_watchdog_status(True)
        elif watchdog_active and (now - last_heartbeat <= watchdog_timeout):
            write_log("[INFO] Watchdog recovered, clearing trouble screen.")
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
    write_log("[SOCKET] Listening on /home/Dale/firepanel/RasPi/firepanel.sock")

    while not stop_event.is_set():
        try:
            conn, _ = server.accept()
            with conn:
                data = conn.recv(1024).decode().strip()
                write_log(f"[SOCKET] Received command: {data}")
                if data.startswith("thermal_trigger:"):
                    ch = int(data.split(":")[1])
                    if 0 <= ch < 8:
                        update_status_fields(thermal=[i == ch for i in range(8)])
                        write_log(f"[SOCKET] Thermal trigger on channel {ch}")
                        conn.sendall(b"OK\n")
                    else:
                        conn.sendall(b"ERR: Invalid channel\n")
                else:
                    conn.sendall(b"ERR: Unknown command\n")
        except Exception as e:
            write_log(f"[SOCKET] Error: {e}")


if __name__ == "__main__":
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.5, exclusive=True)
        ser.reset_input_buffer()
    except Exception as e:
        write_log(f"[FATAL] Could not open serial port: {e}")
        exit(1)

    write_log("Firepanel controller started")
    update_status_fields(stage="booting", mode="BOOTING", conn=[False]*8, trig=[False]*8)

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
    write_log("[MAIN] socket_thread started")
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
        write_log("Firepanel controller stopped")
