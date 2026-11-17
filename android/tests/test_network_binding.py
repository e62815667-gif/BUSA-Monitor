"""Execute the production Wi-Fi selection helper with controlled Android networks."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[2]
source=(root/'android/app/src/main/java/ru/busa/monitor/MainActivity.java').read_text()
start=source.index('private boolean bindExistingWifi()');end=source.index('{',start)+1;depth=1
while depth:
    if source[end]=='{':depth+=1
    elif source[end]=='}':depth-=1
    end+=1
helper=source[start:end]
harness='''
public class NetworkBindingTest {
 static class Network {final boolean wifi;Network(boolean w){wifi=w;}}
 static class NetworkCapabilities {static final int TRANSPORT_WIFI=1;final Network n;NetworkCapabilities(Network x){n=x;}boolean hasTransport(int t){return t==TRANSPORT_WIFI&&n.wifi;}}
 static class Connectivity {
  Network[] all={};Network bound;int calls;boolean denied;
  Network[] getAllNetworks(){if(denied)throw new SecurityException();return all;}
  NetworkCapabilities getNetworkCapabilities(Network n){return new NetworkCapabilities(n);}
  Network getBoundNetworkForProcess(){return bound;}
  boolean bindProcessToNetwork(Network n){bound=n;calls++;return true;}
 }
 Connectivity connectivity=new Connectivity();
'''+helper+'''
 static void require(boolean value,String text){if(!value)throw new AssertionError(text);}
 public static void main(String[] ignored){
  NetworkBindingTest app=new NetworkBindingTest();Network cell=new Network(false),wifiWithoutInternet=new Network(true);
  app.connectivity.all=new Network[]{cell,wifiWithoutInternet};app.connectivity.bound=cell;
  require(app.bindExistingWifi()&&app.connectivity.bound==wifiWithoutInternet,"Existing Wi-Fi without internet must win over cellular");
  int calls=app.connectivity.calls;require(app.bindExistingWifi()&&app.connectivity.calls==calls,"Already selected Wi-Fi must not reset connections");
  app.connectivity.all=new Network[]{cell};app.connectivity.bound=null;
  require(!app.bindExistingWifi()&&app.connectivity.bound==null,"Absent Wi-Fi must leave system route intact");
  app.connectivity.denied=true;require(!app.bindExistingWifi(),"Permission failure must be reported as unavailable");
  System.out.println("PASS production Wi-Fi selector: existing internetless Wi-Fi wins over cellular; no needless rebind; absent/denied cases");
 }
}
'''
with tempfile.TemporaryDirectory() as tmp:
    p=Path(tmp);(p/'NetworkBindingTest.java').write_text(harness)
    log=p/'android/util/Log.java';log.parent.mkdir(parents=True);log.write_text('package android.util; public class Log {public static int w(String t,String m,Throwable e){return 0;}}')
    subprocess.run(['java','com.sun.tools.javac.Main','-d',tmp,str(p/'NetworkBindingTest.java'),str(log)],check=True)
    subprocess.run(['java','-cp',tmp,'NetworkBindingTest'],check=True)
