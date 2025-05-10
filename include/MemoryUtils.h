// MemoryUtils.h
#ifndef MEMORY_UTILS_H
#define MEMORY_UTILS_H

extern int __heap_start;   // provided by linker
extern int *__brkval;      // provided by malloc()

/// Returns the number of free bytes left in SRAM
inline int freeMemory() {
  int v;
  // if __brkval is zero, heap hasn't moved, so use &__heap_start
  return (int)&v - (__brkval ? (int)__brkval : (int)&__heap_start);
}

#endif // MEMORY_UTILS_H