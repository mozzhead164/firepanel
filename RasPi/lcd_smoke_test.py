#!/usr/bin/env python3
from RPLCD.i2c import CharLCD
import time

def main():
    # 1) Create the CharLCD exactly as in your old lcd_controller.py:
    lcd = CharLCD(
        i2c_expander='PCF8574',
        address=0x27,        # <– your known good address
        port=1,
        cols=20,
        rows=4,
        charmap='A00',       # <– match the old working charmap
        auto_linebreaks=False
    )

    # 2) Give the HD44780 a solid 200 ms to finish any power‐on‐reset.
    time.sleep(0.15)

    # 3) Clear & home, then wait a bit so the command really completes.
    lcd.clear()            # clear display (also returns cursor to home in HD44780 spec)
    lcd.home()             # explicitly force cursor to (0,0)
    time.sleep(0.15)        # let the controller finish processing clear+home

    # 4) Write two lines of plain ASCII exactly at (0,0) and (1,0).
    lcd.write_string("MERGED TEST: HELLO")
    lcd.cursor_pos = (1, 0)
    lcd.write_string("LINE 2 OKAY")

    # 5) Hold the text on‐screen for 2 seconds so you can visually confirm.
    time.sleep(3)

    # 6) Before exiting, clear & home again to leave the LCD in a blank state.
    lcd.clear()
    lcd.home()
    time.sleep(0.15)        # allow clear+home to fully clock out before process exits

if __name__ == "__main__":
    main()
