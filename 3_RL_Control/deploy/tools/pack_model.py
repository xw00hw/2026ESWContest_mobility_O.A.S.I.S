"""Model package for the Pi (models are not in git: CONTRIBUTING "Data policy").

  PC : python deploy/tools/pack_model.py deploy/model_v8                 -> dist/oasis_model_v8.tar.gz (+ .sha256)
       scp dist/oasis_model_v8.tar.gz* oasis@<PI>:~/
  Pi : python deploy/tools/pack_model.py --install ~/oasis_model_v8.tar.gz --into deploy/
       (checks the archive sha256 and every file against manifest.json, then extracts deploy/model_v8/)

Runtime files only: manifest.json, obs_layout.py, actor.npz, predictor.npz (BeamNG preview), check_set.npy (backend
equivalence check), actor_weights.h (C backend: `make MODEL=../model_v8` on the Pi). ONNX / Hailo calibration sets
(~40 MB) stay on the PC. Record the printed sha256 in the model card / run notes (storage location + checksum).
"""
import argparse, hashlib, io, json, os, sys, tarfile

KEEP = ('manifest.json', 'obs_layout.py', 'actor.npz', 'predictor.npz', 'check_set.npy', 'actor_weights.h')


def sha(b): return hashlib.sha256(b).hexdigest()


def pack(model_dir, out_dir):
    man = json.load(open(os.path.join(model_dir, 'manifest.json'))); name = os.path.basename(os.path.normpath(model_dir))
    os.makedirs(out_dir, exist_ok=True); path = os.path.join(out_dir, f'oasis_{name}.tar.gz')
    with tarfile.open(path, 'w:gz') as t:
        for f in KEEP:
            p = os.path.join(model_dir, f)
            if not os.path.exists(p):
                if f == 'predictor.npz' and not (man.get('predictor') or {}).get('npz'): continue
                sys.exit(f'missing {p}')
            data = open(p, 'rb').read()
            if f != 'manifest.json' and man['files'].get(f) != sha(data): sys.exit(f'{f} does not match manifest.json')
            ti = tarfile.TarInfo(f'{name}/{f}'); ti.size = len(data); ti.mtime = 0; t.addfile(ti, io.BytesIO(data))
    digest = sha(open(path, 'rb').read()); open(path + '.sha256', 'w').write(f'{digest}  {os.path.basename(path)}\n')
    print(json.dumps(dict(package=path, sha256=digest, model=man['source'], model_sha256=man['source_sha256'],
                          actuator=man.get('actuator')), indent=1))


def install(archive, into):
    data = open(archive, 'rb').read(); want = open(archive + '.sha256').read().split()[0] if os.path.exists(archive + '.sha256') else None
    if want and want != sha(data): sys.exit('archive sha256 mismatch')
    with tarfile.open(fileobj=io.BytesIO(data), mode='r:gz') as t:
        members = t.getmembers()
        names = {m.name for m in members}
        if any(m.issym() or m.islnk() or m.name.startswith(('/', '..')) or '..' in m.name.split('/') for m in members):
            sys.exit('unsafe archive member')
        root = members[0].name.split('/')[0]
        man = json.load(t.extractfile(f'{root}/manifest.json'))
        for m in members:
            f = m.name.split('/', 1)[1]
            if f != 'manifest.json' and man['files'].get(f) != sha(t.extractfile(m).read()): sys.exit(f'{f}: hash mismatch')
        dst = os.path.join(into, root); os.makedirs(dst, exist_ok=True)
        for m in members:
            open(os.path.join(dst, m.name.split('/', 1)[1]), 'wb').write(t.extractfile(m).read())
    print(f'installed {sorted(n.split("/", 1)[1] for n in names)} -> {dst} (sha256 {want or "not checked: no .sha256"})')


if __name__ == '__main__':
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('model_dir', nargs='?'); ap.add_argument('--out', default='dist')
    ap.add_argument('--install'); ap.add_argument('--into', default='deploy')
    a = ap.parse_args()
    install(a.install, a.into) if a.install else pack(a.model_dir, a.out)
