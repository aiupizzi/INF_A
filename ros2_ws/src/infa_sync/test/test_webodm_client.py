from pathlib import Path
from tempfile import TemporaryDirectory

from infa_sync.webodm_client import WebODMClient


def test_encode_multipart_contains_images_and_name():
    client = WebODMClient.__new__(WebODMClient)

    with TemporaryDirectory() as tmp_dir:
        img = Path(tmp_dir) / 'a.jpg'
        img.write_bytes(b'abc')
        payload = client._encode_multipart('BOUNDARY', 'mission-task', [img], {'fast-orthophoto': True})

    assert b'name="name"' in payload
    assert b'mission-task' in payload
    assert b'name="images"; filename="a.jpg"' in payload
