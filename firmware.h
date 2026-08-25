#ifndef FIRMWARE_H  // Prevent this firmware lifecycle header from being included more than once.
#define FIRMWARE_H  // Define the include guard used by this firmware lifecycle header.

void firmwareSetup();  // Initialize every firmware subsystem using the preserved tested startup order.
void firmwareLoop();   // Run the preserved cooperative flight-control and network loop.

#endif  // Close the FIRMWARE_H include guard.
