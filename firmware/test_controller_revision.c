#include "../replacement-control/w600_endpoint.h"
#include <assert.h>
#include <stdio.h>
int main(void)
{
    struct w600_endpoint e;w600_endpoint_init(&e);
    struct w600_frame query={.sequence=7,.kind=1,.length=2,.payload={0x20,2}},reply;
    assert(w600_endpoint_process(&e,&query,&reply));
    assert(reply.kind==4 && reply.sequence==7 && reply.length==6);
    assert(reply.payload[0]==0x20 && reply.payload[1]==2 && !reply.payload[2] &&
        reply.payload[3]==3 && reply.payload[4]==(W600_CONTROLLER_REVISION>>8) &&
        reply.payload[5]==(W600_CONTROLLER_REVISION&255));
    assert(!e.control_enabled && !e.calibration_pending && !e.moving);
    puts("Controller revision: BE16 getter, no motor/control side effects PASS");
}
