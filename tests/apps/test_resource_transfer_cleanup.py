from __future__ import annotations

import base64
import io
import json
import zipfile
from types import SimpleNamespace

import pytest

from rynnrcp_app_common.resource_transfer import download_collection_resource


class ResourceClient:
    def __init__(self, archive_bytes: bytes, *, fail_read: bool = False, fail_delete: bool = False) -> None:
        self.archive_bytes = archive_bytes
        self.fail_read = fail_read
        self.fail_delete = fail_delete
        self.deleted: list[str] = []

    def prepare_resource_archive(self, **_kwargs):
        return SimpleNamespace(ok=True, payload={"resource": {"resource_id": "archive-1"}}, message="")

    def read_resource(self, resource_id: str, **_kwargs):
        assert resource_id == "archive-1"
        if self.fail_read:
            return SimpleNamespace(ok=False, payload={}, message="read failed")
        return SimpleNamespace(
            ok=True,
            payload={
                "data": base64.b64encode(self.archive_bytes).decode("ascii"),
                "next_offset": len(self.archive_bytes),
                "eof": True,
            },
            message="",
        )

    def delete_resource(self, resource_id: str, **_kwargs):
        self.deleted.append(resource_id)
        if self.fail_delete:
            raise RuntimeError("delete failed")
        return SimpleNamespace(ok=True, payload={"deleted": True}, message="")


def _collection_archive() -> bytes:
    content = io.BytesIO()
    with zipfile.ZipFile(content, "w") as archive:
        archive.writestr(
            "collection/collection_meta.json",
            json.dumps({"collection_id": "dataset-1", "episode_id": "episode-1"}),
        )
        archive.writestr("collection/sample.bin", b"sample")
    return content.getvalue()


def _collection() -> dict:
    return {
        "collection_id": "dataset-1",
        "episode_id": "episode-1",
        "collection_resource": {"resource_id": "collection-1"},
    }


def test_download_releases_server_archive_after_success(tmp_path) -> None:
    client = ResourceClient(_collection_archive())

    result = download_collection_resource(client, _collection(), str(tmp_path))

    assert client.deleted == ["archive-1"]
    assert (tmp_path / "dataset-1" / "episode-1" / "sample.bin").read_bytes() == b"sample"


def test_download_releases_server_archive_after_failure(tmp_path) -> None:
    client = ResourceClient(_collection_archive(), fail_read=True)

    with pytest.raises(RuntimeError, match="read failed"):
        download_collection_resource(client, _collection(), str(tmp_path))

    assert client.deleted == ["archive-1"]


def test_cleanup_failure_does_not_discard_successful_download(tmp_path, caplog) -> None:
    client = ResourceClient(_collection_archive(), fail_delete=True)

    result = download_collection_resource(client, _collection(), str(tmp_path))

    assert result == str(tmp_path / "dataset-1" / "episode-1")
    assert "Failed to release temporary Resource archive-1" in caplog.text
