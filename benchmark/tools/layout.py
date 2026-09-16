"""Integrity guard for current relocated tools; historical manifests stay immutable."""
from pathlib import Path
import hashlib,json

def verify_tool_sources(root):
    root=Path(root)
    manifest=json.loads((root/'benchmark/manifests/layout.json').read_text())
    for name,expected in manifest['active_source_sha256'].items():
        actual=hashlib.sha256((root/name).read_bytes()).hexdigest()
        if actual!=expected:raise RuntimeError('Source checksum mismatch: '+name)
