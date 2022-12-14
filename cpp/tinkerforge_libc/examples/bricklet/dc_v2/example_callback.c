#define IPCON_EXPOSE_MILLISLEEP

#include <stdio.h>

#include "ip_connection.h"
#include "bricklet_dc_v2.h"

#define HOST "localhost"
#define PORT 4223
#define UID "XYZ" // Change XYZ to the UID of your DC Bricklet 2.0

// Use velocity reached callback to swing back and forth
// between full speed forward and full speed backward
void cb_velocity_reached(int16_t velocity, void *user_data) {
	DCV2 *dc = (DCV2 *)user_data;

	if(velocity == 32767) {
		printf("Velocity: Full speed forward, now turning backward\n");
		dc_v2_set_velocity(dc, -32767);
	} else if(velocity == -32767) {
		printf("Velocity: Full speed backward, now turning forward\n");
		dc_v2_set_velocity(dc, 32767);
	} else {
		printf("Error\n"); // Can only happen if another program sets velocity
	}
}

int main(void) {
	// Create IP connection
	IPConnection ipcon;
	ipcon_create(&ipcon);

	// Create device object
	DCV2 dc;
	dc_v2_create(&dc, UID, &ipcon);

	// Connect to brickd
	if(ipcon_connect(&ipcon, HOST, PORT) < 0) {
		fprintf(stderr, "Could not connect\n");
		return 1;
	}
	// Don't use device before ipcon is connected

	// The acceleration has to be smaller or equal to the maximum
	// acceleration of the DC motor, otherwise the velocity reached
	// callback will be called too early
	dc_v2_set_motion(&dc, 4096,
	                 16384); // Slow acceleration (12.5 %/s), fast decceleration (50 %/s) for stopping
	dc_v2_set_velocity(&dc, 32767); // Full speed forward (100 %)

	// Register velocity reached callback to function cb_velocity_reached
	dc_v2_register_callback(&dc,
	                        DC_V2_CALLBACK_VELOCITY_REACHED,
	                        (void (*)(void))cb_velocity_reached,
	                        &dc);

	// Enable motor power
	dc_v2_set_enabled(&dc, true);

	printf("Press key to exit\n");
	getchar();

	dc_v2_set_velocity(&dc, 0); // Stop motor before disabling motor power
	millisleep(2000); // Wait for motor to actually stop: velocity (100 %) / decceleration (50 %/s) = 2 s
	dc_v2_set_enabled(&dc, false); // Disable motor power

	dc_v2_destroy(&dc);
	ipcon_destroy(&ipcon); // Calls ipcon_disconnect internally
	return 0;
}
