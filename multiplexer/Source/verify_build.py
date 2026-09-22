from pathlib import Path
import hashlib,json,subprocess,sys,tempfile
S=Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix='multiplexer-rebuild-') as temporary:
    subprocess.run([sys.executable,str(S/'build.py'),'--output',temporary],check=True)
    for entry in json.loads((S/'accepted-output-hashes.json').read_text()):
        actual=hashlib.sha256((Path(temporary)/entry['file']).read_bytes()).hexdigest()
        assert actual==entry['sha256'],entry['file']
print('PASS: all 10 part STLs and complete print layout match the accepted revision byte-for-byte.')
