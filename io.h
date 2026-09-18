#ifndef IO_H
#define IO_H

// Initialize the single USB CDC port, waiting briefly for enumeration.
void io_begin(void);

// Service USB and LEDs. Used by the main loop and serial-hook wait paths.
// Must not touch a wrapped bus or call zenoh, to avoid recursion.
void io_poll(void);

#endif // IO_H
