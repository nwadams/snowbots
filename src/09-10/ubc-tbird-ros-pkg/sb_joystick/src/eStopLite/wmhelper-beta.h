#include <cwiid.h>
#include <stdlib.h>
#include <stdio.h>

cwiid_err_t err;
cwiid_mesg_callback_t cwiid_callback;

void set_led_state(cwiid_wiimote_t *wiimote, unsigned char led_state);
void set_rpt_mode(cwiid_wiimote_t *wiimote, unsigned char rpt_mode);
