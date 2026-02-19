from __future__ import annotations

import json
import mimetypes
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib import error, parse, request


@dataclass
class WebODMTaskResult:
    task_id: int
    status: int
    output: dict[str, Any]


class WebODMClient:
    """Authenticated WebODM API wrapper for imagery upload and task polling."""

    def __init__(
        self,
        base_url: str,
        username: str,
        password: str,
        verify_tls: bool = True,
        timeout_s: float = 15.0,
    ) -> None:
        self._base_url = base_url.rstrip('/')
        self._timeout_s = timeout_s
        self._verify_tls = verify_tls
        self._token = self._authenticate(username, password)

    def _json_request(self, method: str, url: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        body = None if payload is None else json.dumps(payload).encode('utf-8')
        headers = {'Content-Type': 'application/json'}
        if self._token:
            headers['Authorization'] = f'JWT {self._token}'

        req = request.Request(url, method=method, data=body, headers=headers)
        try:
            with request.urlopen(req, timeout=self._timeout_s) as resp:
                return json.loads(resp.read().decode('utf-8'))
        except error.HTTPError as exc:
            raise RuntimeError(f'WebODM API error {exc.code} for {url}') from exc

    def _authenticate(self, username: str, password: str) -> str:
        payload = self._json_request(
            'POST',
            f'{self._base_url}/api/token-auth/',
            {'username': username, 'password': password},
        )
        token = payload.get('token')
        if not token:
            raise RuntimeError('WebODM authentication succeeded but no token was returned.')
        return str(token)

    def create_task(
        self,
        project_id: int,
        image_paths: list[Path],
        task_name: str,
        options: dict[str, Any] | None = None,
    ) -> int:
        boundary = f'----INFABoundary{uuid.uuid4().hex}'
        headers = {'Authorization': f'JWT {self._token}', 'Content-Type': f'multipart/form-data; boundary={boundary}'}
        data = self._encode_multipart(boundary, task_name, image_paths, options)
        req = request.Request(
            f'{self._base_url}/api/projects/{project_id}/tasks/',
            data=data,
            headers=headers,
            method='POST',
        )
        try:
            with request.urlopen(req, timeout=max(60.0, self._timeout_s)) as resp:
                payload = json.loads(resp.read().decode('utf-8'))
        except error.HTTPError as exc:
            raise RuntimeError(f'WebODM task create failed with status {exc.code}') from exc

        task_id = payload.get('id')
        if task_id is None:
            raise RuntimeError('WebODM create task response missing task id.')
        return int(task_id)

    def _encode_multipart(
        self,
        boundary: str,
        task_name: str,
        image_paths: list[Path],
        options: dict[str, Any] | None,
    ) -> bytes:
        lines: list[bytes] = []

        def add_field(name: str, value: str) -> None:
            lines.extend(
                [
                    f'--{boundary}'.encode(),
                    f'Content-Disposition: form-data; name="{name}"'.encode(),
                    b'',
                    value.encode(),
                ]
            )

        add_field('name', task_name)
        if options:
            add_field('options', json.dumps(options))

        for path in image_paths:
            mime = mimetypes.guess_type(path.name)[0] or 'application/octet-stream'
            lines.extend(
                [
                    f'--{boundary}'.encode(),
                    f'Content-Disposition: form-data; name="images"; filename="{parse.quote(path.name)}"'.encode(),
                    f'Content-Type: {mime}'.encode(),
                    b'',
                    path.read_bytes(),
                ]
            )

        lines.append(f'--{boundary}--'.encode())
        lines.append(b'')
        return b'\r\n'.join(lines)

    def get_task(self, project_id: int, task_id: int) -> WebODMTaskResult:
        payload = self._json_request('GET', f'{self._base_url}/api/projects/{project_id}/tasks/{task_id}/')
        return WebODMTaskResult(
            task_id=int(payload['id']),
            status=int(payload['status']),
            output=payload,
        )
