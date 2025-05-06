


namespace Buzzer {
    void init();                       // call once in setup()
    void beep(uint16_t ms);            // start / retrigger a beep
    void update();                     // call every loop()
    bool busy();                       // optional: true while beeping
}