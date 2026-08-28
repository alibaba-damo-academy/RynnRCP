"""Tests for ResourceRegistry and ResourceService protocol tools."""

from __future__ import annotations

import base64
import json
import zipfile
from pathlib import Path

import pytest

from rynnrcp.runtime.tool_bus import ToolBus
from rynnrcp.services.resource_service import (
    ResourceRegistry,
    ResourceService,
    _unique_path,
)


@pytest.fixture
def registry(tmp_path: Path) -> ResourceRegistry:
    return ResourceRegistry(str(tmp_path / "tmp"))


@pytest.fixture
def service(registry: ResourceRegistry) -> ResourceService:
    return ResourceService(ToolBus(), registry)


def _make_file(tmp_path: Path, name: str = "sample.txt", data: bytes = b"hello") -> Path:
    path = tmp_path / name
    path.write_bytes(data)
    return path


def _make_collection(tmp_path: Path, collection: str = "col", episode: str = "ep") -> Path:
    root = tmp_path / "datasets" / collection / episode
    (root / "streams" / "obs").mkdir(parents=True)
    (root / "streams" / "obs" / "samples.msgpack").write_bytes(b"\x81")
    (root / "collection_meta.json").write_text(
        json.dumps(
            {
                "collection_id": collection,
                "episode_id": episode,
                "started_at_unix": 5.0,
                "metadata": {"task_prompt": "pick", "task_description": "desc"},
            }
        )
    )
    return root


# ---------------------------------------------------------------------------
# ResourceRegistry
# ---------------------------------------------------------------------------

def test_register_path_describes_file_with_hash(registry: ResourceRegistry, tmp_path: Path) -> None:
    path = _make_file(tmp_path)
    resource = registry.register_path(str(path), domain="data", name="sample")
    assert resource["type"] == "file"
    assert resource["format"] == "txt"
    assert resource["size_bytes"] == 5
    assert resource["hash"].startswith("sha256:")
    assert resource["mime_type"] == "text/plain"

    # Registering again reuses the same resource id.
    again = registry.register_path(str(path), domain="data")
    assert again["resource_id"] == resource["resource_id"]


def test_register_path_rejects_missing_path(registry: ResourceRegistry, tmp_path: Path) -> None:
    with pytest.raises(FileNotFoundError):
        registry.register_path(str(tmp_path / "nope.bin"))


def test_register_directory_reports_total_size(registry: ResourceRegistry, tmp_path: Path) -> None:
    directory = tmp_path / "dir"
    directory.mkdir()
    (directory / "a.bin").write_bytes(b"12345")
    (directory / "b.bin").write_bytes(b"123")
    resource = registry.register_path(str(directory))
    assert resource["type"] == "directory"
    assert resource["format"] == "directory"
    assert resource["size_bytes"] == 8


def test_resolve_unknown_resource_raises(registry: ResourceRegistry) -> None:
    with pytest.raises(KeyError, match="unknown resource_id"):
        registry.resolve("res_missing")


def test_list_resources_filters_and_drops_missing_paths(
    registry: ResourceRegistry, tmp_path: Path
) -> None:
    keep = _make_file(tmp_path, "keep.txt")
    gone = _make_file(tmp_path, "gone.txt")
    registry.register_path(str(keep), domain="data", metadata={"kind": "log"})
    removed = registry.register_path(str(gone), domain="model")
    gone.unlink()

    resources = registry.list_resources()
    ids = {item["resource_id"] for item in resources}
    assert removed["resource_id"] not in ids

    assert registry.list_resources(domain="model") == []
    assert [r["metadata"]["kind"] for r in registry.list_resources(kind="log")] == ["log"]


def test_catalog_roots_discover_collections_and_logs(
    registry: ResourceRegistry, tmp_path: Path
) -> None:
    collection_dir = _make_collection(tmp_path)
    logs_root = tmp_path / "logs"
    logs_root.mkdir()
    (logs_root / "server.log").write_text("line")

    registry.add_catalog_root(
        "datasets", str(tmp_path / "datasets"), domain="data",
        metadata={"kind": "collections_root"},
    )
    registry.add_catalog_root(
        "logs", str(logs_root), domain="log", metadata={"kind": "logs_root"}
    )
    registry.add_catalog_root("missing", str(tmp_path / "missing"), domain="data")

    resources = registry.list_resources()
    by_name = {item["name"]: item for item in resources}
    assert "col/ep" in by_name
    collection = by_name["col/ep"]
    assert collection["metadata"]["task_prompt"] == "pick"
    assert collection["metadata"]["local_path"] == str(collection_dir)
    assert by_name["server.log"]["metadata"]["kind"] == "log"


def test_collection_meta_fallbacks(registry: ResourceRegistry, tmp_path: Path) -> None:
    broken = tmp_path / "datasets" / "broken" / "ep"
    broken.mkdir(parents=True)
    (broken / "collection_meta.json").write_text("not json")
    registry.add_catalog_root(
        "datasets", str(tmp_path / "datasets"), domain="data",
        metadata={"kind": "collections_root"},
    )
    names = {item["name"] for item in registry.list_resources()}
    # Unreadable metadata falls back to the directory basename.
    assert "ep" in names


def test_delete_removes_files_and_directories(
    registry: ResourceRegistry, tmp_path: Path
) -> None:
    path = _make_file(tmp_path)
    record = registry.register_path(str(path))
    assert registry.delete(record["resource_id"]) is True
    assert not path.exists()

    directory = tmp_path / "dir"
    directory.mkdir()
    (directory / "f").write_bytes(b"1")
    record = registry.register_path(str(directory))
    assert registry.delete(record["resource_id"]) is True
    assert not directory.exists()


def test_is_managed_temporary_requires_flag_and_tmp_prefix(
    registry: ResourceRegistry, tmp_path: Path
) -> None:
    outside = _make_file(tmp_path, "outside.bin")
    record = registry.register_path(str(outside), metadata={"temporary": True})
    assert registry.is_managed_temporary(registry.resolve(record["resource_id"])) is False

    inside = Path(registry.tmp_dir) / "inside.bin"
    inside.write_bytes(b"1")
    flagged = registry.register_path(str(inside), metadata={"temporary": True})
    assert registry.is_managed_temporary(registry.resolve(flagged["resource_id"])) is True

    unflagged = registry.register_path(str(inside))
    assert registry.is_managed_temporary(registry.resolve(unflagged["resource_id"])) is False


# ---------------------------------------------------------------------------
# ResourceService
# ---------------------------------------------------------------------------

def test_bind_registers_resource_tools(service: ResourceService) -> None:
    service.bind()
    tools = service.bus.list_tools()
    assert len(service.tool_names) == 7
    for name in service.tool_names:
        assert name in tools
    service.unbind()
    assert service.bus.list_tools() == {}


def test_get_resource_info_success_and_failure(
    service: ResourceService, registry: ResourceRegistry, tmp_path: Path
) -> None:
    record = registry.register_path(str(_make_file(tmp_path)))
    ok = service.get_resource_info(record["resource_id"])
    assert ok["success"] is True
    assert ok["result"]["resource"]["resource_id"] == record["resource_id"]

    bad = service.get_resource_info("res_unknown")
    assert bad["success"] is False


def test_list_resources_paginates(service: ResourceService, registry: ResourceRegistry, tmp_path: Path) -> None:
    for index in range(3):
        registry.register_path(str(_make_file(tmp_path, f"f{index}.txt")))
    first = service.list_resources(limit=2)
    assert first["success"] is True
    assert len(first["result"]["resources"]) == 2
    assert first["result"]["next_cursor"] == "2"

    second = service.list_resources(cursor=first["result"]["next_cursor"], limit=2)
    assert len(second["result"]["resources"]) == 1
    assert "next_cursor" not in second["result"]


def test_list_resource_entries(service: ResourceService, registry: ResourceRegistry, tmp_path: Path) -> None:
    directory = tmp_path / "dir"
    (directory / "sub").mkdir(parents=True)
    (directory / "a.txt").write_text("a")
    (directory / "sub" / "b.txt").write_text("b")
    record = registry.register_path(str(directory))

    flat = service.list_resource_entries(record["resource_id"])
    names = [entry["name"] for entry in flat["result"]["entries"]]
    assert names == ["a.txt", "sub"]

    deep = service.list_resource_entries(record["resource_id"], recursive=True)
    deep_names = {entry["name"] for entry in deep["result"]["entries"]}
    assert "sub/b.txt" in deep_names or "sub\\b.txt" in deep_names

    file_record = registry.register_path(str(_make_file(tmp_path)))
    not_dir = service.list_resource_entries(file_record["resource_id"])
    assert not_dir["success"] is False
    assert "not a directory" in not_dir["message"]

    missing = service.list_resource_entries("res_unknown")
    assert missing["success"] is False


def test_read_resource_supports_offsets(service: ResourceService, registry: ResourceRegistry, tmp_path: Path) -> None:
    record = registry.register_path(str(_make_file(tmp_path, data=b"0123456789")))
    rid = record["resource_id"]

    full = service.read_resource(rid)
    assert full["success"] is True
    assert base64.b64decode(full["result"]["data"]) == b"0123456789"
    assert full["result"]["eof"] is True

    partial = service.read_resource(rid, offset=4, limit=3)
    assert base64.b64decode(partial["result"]["data"]) == b"456"
    assert partial["result"]["next_offset"] == 7
    assert partial["result"]["eof"] is False

    directory = tmp_path / "dir"
    directory.mkdir()
    dir_record = registry.register_path(str(directory))
    not_file = service.read_resource(dir_record["resource_id"])
    assert not_file["success"] is False

    assert service.read_resource("res_unknown")["success"] is False


def test_delete_resource_rules(service: ResourceService, registry: ResourceRegistry, tmp_path: Path) -> None:
    assert service.delete_resource("")["success"] is False
    assert service.delete_resource("res_unknown")["success"] is False

    protected = registry.register_path(str(_make_file(tmp_path)))
    denied = service.delete_resource(protected["resource_id"])
    assert denied["success"] is False
    assert "only log and managed temporary" in denied["message"]

    log_file = _make_file(tmp_path, "run.log")
    log_record = registry.register_path(str(log_file), domain="log")
    deleted = service.delete_resource(log_record["resource_id"])
    assert deleted["success"] is True
    assert not log_file.exists()


def test_snapshot_resource_modes(service: ResourceService, registry: ResourceRegistry, tmp_path: Path) -> None:
    snapshot_record = registry.register_path(str(_make_file(tmp_path)), mode="snapshot")
    result = service.snapshot_resource(snapshot_record["resource_id"])
    assert result["success"] is True
    assert result["result"]["resource"]["resource_id"] == snapshot_record["resource_id"]

    live_file = _make_file(tmp_path, "live.txt", b"live-data")
    live_record = registry.register_path(str(live_file), mode="live")
    copied = service.snapshot_resource(live_record["resource_id"])
    assert copied["success"] is True
    copy_resource = copied["result"]["resource"]
    assert copy_resource["resource_id"] != live_record["resource_id"]
    assert copy_resource["metadata"]["temporary"] is True

    directory = tmp_path / "dir"
    directory.mkdir()
    (directory / "f.txt").write_text("x")
    dir_record = registry.register_path(str(directory), mode="live")
    archived = service.snapshot_resource(dir_record["resource_id"])
    assert archived["success"] is True
    assert archived["result"]["resource"]["format"] == "zip"

    assert service.snapshot_resource("res_unknown")["success"] is False


def test_prepare_resource_archive(service: ResourceService, registry: ResourceRegistry, tmp_path: Path) -> None:
    file_record = registry.register_path(str(_make_file(tmp_path, "a.txt", b"a")))
    directory = tmp_path / "dir"
    directory.mkdir()
    (directory / "inner.txt").write_text("inner")
    dir_record = registry.register_path(str(directory))

    unsupported = service.prepare_resource_archive(
        resource_id=file_record["resource_id"], format="tar"
    )
    assert unsupported["success"] is False

    empty = service.prepare_resource_archive()
    assert empty["success"] is False

    combined = service.prepare_resource_archive(
        resource_ids=[file_record["resource_id"], dir_record["resource_id"]]
    )
    assert combined["success"] is True
    archive = combined["result"]["resource"]
    assert archive["format"] == "zip"
    archive_path = registry.resolve(archive["resource_id"]).path
    with zipfile.ZipFile(archive_path) as zf:
        names = set(zf.namelist())
    assert any(name.endswith("inner.txt") for name in names)
    assert any(name.endswith("a.txt") for name in names)

    # Archiving again with the same name must not clobber the first archive.
    single = service.prepare_resource_archive(resource_id=dir_record["resource_id"])
    assert single["success"] is True


def test_unique_path_appends_suffix(tmp_path: Path) -> None:
    target = tmp_path / "archive.zip"
    assert _unique_path(str(target)) == str(target)
    target.write_bytes(b"")
    assert _unique_path(str(target)) == str(tmp_path / "archive_2.zip")
