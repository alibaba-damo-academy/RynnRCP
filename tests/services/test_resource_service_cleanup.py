from __future__ import annotations

from pathlib import Path

from rynnrcp.runtime.tool_bus import ToolBus
from rynnrcp.services.resource_service import ResourceRegistry, ResourceService


def _service(tmp_path: Path) -> tuple[ResourceRegistry, ResourceService]:
    registry = ResourceRegistry(str(tmp_path / "resources"))
    return registry, ResourceService(ToolBus(), registry)


def test_prepared_archive_can_be_deleted(tmp_path: Path) -> None:
    registry, service = _service(tmp_path)
    collection_dir = tmp_path / "collection"
    collection_dir.mkdir()
    (collection_dir / "sample.txt").write_text("sample", encoding="utf-8")
    source = registry.register_path(str(collection_dir), resource_type="directory", mode="snapshot")

    prepared = service.prepare_resource_archive(resource_id=source["resource_id"])
    archive = prepared["result"]["resource"]
    archive_path = registry.resolve(archive["resource_id"]).path

    assert prepared["success"] is True
    assert Path(archive_path).is_file()
    deleted = service.delete_resource(archive["resource_id"])
    assert deleted == {"success": True, "message": "OK", "result": {"deleted": True}}
    assert not Path(archive_path).exists()


def test_data_resource_outside_tmp_cannot_be_deleted(tmp_path: Path) -> None:
    registry, service = _service(tmp_path)
    data_path = tmp_path / "dataset.bin"
    data_path.write_bytes(b"data")
    resource = registry.register_path(
        str(data_path),
        domain="data",
        mode="snapshot",
        metadata={"temporary": True},
    )

    result = service.delete_resource(resource["resource_id"])

    assert result["success"] is False
    assert data_path.is_file()
