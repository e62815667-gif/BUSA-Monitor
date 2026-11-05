"""Generate the existing four-anchor Google trust bundle; fingerprint checks mandatory."""
from pathlib import Path
import argparse,hashlib,ssl,urllib.request

FINGERPRINTS={
 'gtsr1':'d947432abde7b7fa90fc2e6b59101b1280e0e1c7e4e40fa3c6887fff57a7f4cf',
 'gtsr2':'8d25cd97229dbf70356bda4eb3cc734031e24cf00fafcfd32dc76eb5841c7ea8',
 'gtsr3':'34d8a73ee208d9bcdb0d956520934b4e40e69482596e8b6f73c8426b010a6f48',
 'gtsr4':'349dfa4058c5e263123b398ae795573c4e1313c83fe68f93556cd5e8031b3c7d'}

def generate(output):
    bundle=[]
    for name,expected in FINGERPRINTS.items():
        cert=Path('/etc/ssl/certs')/('GTS_Root_R'+name[-1]+'.pem')
        if cert.is_file():pem=cert.read_text()
        else:
            with urllib.request.urlopen('https://pki.goog/repo/certs/'+name+'.pem',timeout=25) as response:
                pem=response.read(12000).decode('ascii')
        assert pem.count('-----BEGIN CERTIFICATE-----')==1
        assert hashlib.sha256(ssl.PEM_cert_to_DER_cert(pem)).hexdigest()==expected,name
        bundle.append(pem.strip())
    Path(output).write_text('const char GOOGLE_TRUST_ANCHOR[] PROGMEM=R"GOOGLECERT(\n'+'\n'.join(bundle)+'\n)GOOGLECERT";\n')
    print('PASS verified GTS Root R1–R4 bundle')

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True)
    generate(parser.parse_args().output)
