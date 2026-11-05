"""Exercise the actual bounded MQTT PUBACK correlation state machine."""
from pathlib import Path
import subprocess,tempfile
root=Path('firmware/BUSA_1M_WIFI_3v_GOOGLE').resolve()
code=r'''
#include "BUSA_RmPublishTracker.h"
#include <cassert>
int main(){
 BusaRmPublishTracker t;t.connected();assert(t.generation==1);
 t.submitted(12,100,110,1,10);assert(t.enqueued==1&&t.outstanding()==1&&t.acked==0);
 t.published(99,120);assert(t.acked==0);t.published(12,130);assert(t.acked==1&&t.lastAckSeq==1&&t.lastAckUptime==10);
 // PUBACK may arrive on the event task before the publish function returns.
 t.published(13,150);t.submitted(13,140,160,2,20);assert(t.acked==2&&t.outstanding()==0);
 // A stale ACK with a reused ID must not acknowledge a later frame.
 t.published(14,170);t.submitted(14,180,190,3,30);assert(t.acked==2&&t.outstanding()==1);
 t.messageDeleted(14);assert(t.unconfirmed==1&&t.deleted==1&&t.outstanding()==0);
 t.submitted(15,200,210,4,40);assert(!t.expire(60209));assert(t.expire(60210));assert(t.unconfirmed==2);
 t.submitted(16,70000,70001,5,50);t.disconnected();assert(t.unconfirmed==3&&t.outstanding()==0);
 t.connected();assert(t.generation==2);
 t.submitted(18,71000,71001,7,70);t.submitted(17,71000,71002,6,60);
 t.published(18,72000);t.published(17,72001);assert(t.lastAckSeq==7&&t.lastAckUptime==70);
 uint32_t n=UINT32_MAX-100;t.submitted(19,n,n+10,8,80);assert(t.expire(n+60010));
 t.published(20,n+15);t.submitted(20,n+10,n+20,9,90);assert(t.lastAckSeq==9);
 for(int i=0;i<17;i++)t.submitted(100+i,1,2,10+i,100+i);
 assert(t.outstanding()==16);assert(t.unconfirmed>=5);
}
'''
with tempfile.TemporaryDirectory() as d:
 p=Path(d)/'tracker.cpp';p.write_text(code);exe=Path(d)/'tracker'
 subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-I'+str(root),str(p),'-o',str(exe)],check=True);subprocess.run([str(exe)],check=True)
print('PASS actual PUBACK tracker: message-ID correlation, early/out-of-order ACK, delete/timeout/disconnect, bounded storage and rollover')
