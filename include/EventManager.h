#ifndef EVENT_MANAGER_H
#define EVENT_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

// Define event types. For now we only need a cable state change event.
enum EventType {
    EVENT_NONE = 0,
    EVENT_CABLE_CHANGE,
    EVENT_OUTPUT_CONFIRM,
    EVENT_CHANNEL_TRIGGER,
    EVENT_BREAK_GLASS,
    EVENT_PSU_UNDERVOLTAGE,
    EVENT_PSU_RESTORED    
    // Other event types (e.g., button press) can be added here.
};


// Define the event structure; channel is the index (0 to 7) of the cable.
typedef struct {
    EventType type;     // type of event
    uint8_t channel;    // which cable (or LED) this event relates to
    bool connected;     // true if cable is connected, false if disconnected
} Event;

// Define the type for the event callback.
typedef void (*EventCallback)(const Event*);

// Register a callback to be notified of events.
void registerEventCallback(EventCallback cb);

// Register an event handler for the event manager.
void eventHandler(const Event *event);

// Dispatch an event to the registered callback.
void dispatchEvent(const Event* event);

#endif // EVENT_MANAGER_H
