import hashlib
import glob
from pathlib import Path

for path in sorted(Path(".").glob("*.csv")):
    data = path.read_bytes()
    h = hashlib.sha256(data).hexdigest()
    print(f"{path.name:45s} | size: {len(data):8d} B | sha256: {h}")

