"""ResourceService: expose files and directories without leaking local paths."""

from __future__ import annotations

import base64
import hashlib
import json
import mimetypes
import os
import shutil
import time
import zipfile
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Optional

from .base_service import BaseService
from rynnrcp.protocol.methods import (
    DELETE_RESOURCE,
    GET_RESOURCE_INFO,
    LIST_RESOURCES,
    LIST_RESOURCE_ENTRIES,
    PREPARE_RESOURCE_ARCHIVE,
    READ_RESOURCE,
    SNAPSHOT_RESOURCE,
)
from rynnrcp.runtime.tool_bus import ToolBus
from rynnrcp.utils import safe_name


@dataclass
class ResourceRecord:
    resource_id: str
    path: str
    type: str
    domain: str
    name: str
    format: str
    mode: str
    metadata: Dict[str, Any]
    created_at: float


@dataclass
class ResourceCatalogRoot:
    key: str
    path: str
    domain: str
    name: str
    metadata: Dict[str, Any]


class ResourceRegistry:
    """In-memory map from opaque protocol resource ids to local paths."""

    def __init__(self, tmp_dir: str) -> None:
        self._tmp_dir = os.path.abspath(os.path.expanduser(str(tmp_dir)))
        os.makedirs(self._tmp_dir, exist_ok=True)
        self._records: Dict[str, ResourceRecord] = {}
        self._path_to_id: Dict[str, str] = {}
        self._catalog_roots: Dict[str, ResourceCatalogRoot] = {}

    @property
    def tmp_dir(self) -> str:
        return self._tmp_dir

    def register_path(
        self,
        path: str,
        *,
        resource_type: Optional[str] = None,
        domain: str = "data",
        name: Optional[str] = None,
        format: Optional[str] = None,
        mode: str = "snapshot",
        metadata: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        abs_path = os.path.abspath(os.path.expanduser(str(path)))
        if not os.path.exists(abs_path):
            raise FileNotFoundError(f"resource path not found: {abs_path}")
        rtype = resource_type or ("directory" if os.path.isdir(abs_path) else "file")
        rid = self._path_to_id.get(abs_path) or _resource_id(abs_path)
        record = ResourceRecord(
            resource_id=rid,
            path=abs_path,
            type=rtype,
            domain=str(domain or ""),
            name=str(name or os.path.basename(abs_path) or rid),
            format=str(format or _resource_format(abs_path, rtype)),
            mode=str(mode or "snapshot"),
            metadata=dict(metadata or {}),
            created_at=time.time(),
        )
        self._records[rid] = record
        self._path_to_id[abs_path] = rid
        return self.describe(rid)

    def add_catalog_root(
        self,
        key: str,
        path: str,
        *,
        domain: str,
        name: Optional[str] = None,
        metadata: Optional[Dict[str, Any]] = None,
    ) -> None:
        root_path = os.path.abspath(os.path.expanduser(str(path)))
        self._catalog_roots[str(key)] = ResourceCatalogRoot(
            key=str(key),
            path=root_path,
            domain=str(domain or "data"),
            name=str(name or os.path.basename(root_path) or key),
            metadata=dict(metadata or {}),
        )

    def list_resources(
        self,
        *,
        domain: Optional[str] = None,
        kind: Optional[str] = None,
    ) -> list[Dict[str, Any]]:
        self.refresh_catalog()
        resources: list[Dict[str, Any]] = []
        for rid, record in list(self._records.items()):
            if not os.path.exists(record.path):
                self._records.pop(rid, None)
                self._path_to_id.pop(record.path, None)
                continue
            resources.append(self.describe(rid))
        if domain:
            resources = [item for item in resources if item.get("domain") == domain]
        if kind:
            resources = [item for item in resources if (item.get("metadata") or {}).get("kind") == kind]
        resources.sort(key=lambda item: (str(item.get("domain") or ""), str(item.get("name") or "")))
        return resources

    def refresh_catalog(self) -> None:
        for root in self._catalog_roots.values():
            if not os.path.exists(root.path):
                continue
            self.register_path(
                root.path,
                resource_type="directory",
                domain=root.domain,
                name=root.name,
                format="directory",
                mode="live",
                metadata={**root.metadata, "catalog_root": root.key},
            )
            root_kind = str(root.metadata.get("kind") or "")
            if root_kind == "collections_root":
                for collection_dir in _iter_collection_dirs(root.path):
                    meta = _read_collection_meta(collection_dir)
                    self.register_path(
                        collection_dir,
                        resource_type="directory",
                        domain=root.domain,
                        name=_collection_resource_name(collection_dir, meta),
                        format="directory",
                        mode="snapshot",
                        metadata={
                            "kind": "collection",
                            "catalog_root": root.key,
                            "collection_id": str(meta.get("collection_id") or ""),
                            "episode_id": str(meta.get("episode_id") or ""),
                            "task_prompt": str(meta.get("task_prompt") or ""),
                            "task_description": str(meta.get("task_description") or ""),
                            "started_at_unix": float(meta.get("started_at_unix") or 0.0),
                            "local_path": os.path.abspath(collection_dir),
                        },
                    )
            elif root_kind == "logs_root":
                for file_path in _iter_files(root.path):
                    rel = os.path.relpath(file_path, root.path)
                    self.register_path(
                        file_path,
                        resource_type="file",
                        domain=root.domain,
                        name=rel,
                        mode="live",
                        metadata={"kind": "log", "catalog_root": root.key, "relative_path": rel},
                    )

    def resolve(self, resource_id: str) -> ResourceRecord:
        rid = str(resource_id or "")
        record = self._records.get(rid)
        if record is None:
            raise KeyError(f"unknown resource_id: {rid}")
        return record

    def describe(self, resource_id: str) -> Dict[str, Any]:
        record = self.resolve(resource_id)
        stat = os.stat(record.path)
        resource = {
            "resource_id": record.resource_id,
            "type": record.type,
            "domain": record.domain,
            "name": record.name,
            "format": record.format,
            "mime_type": _mime_type(record.path, record.format),
            "mode": record.mode,
            "created_at": record.created_at,
            "updated_at": float(stat.st_mtime),
            "metadata": dict(record.metadata),
        }
        if os.path.isfile(record.path):
            resource["size_bytes"] = int(stat.st_size)
            if record.mode == "snapshot":
                resource["hash"] = _sha256_file(record.path)
        elif os.path.isdir(record.path):
            resource["size_bytes"] = _directory_size(record.path)
        return resource

    def delete(self, resource_id: str) -> bool:
        record = self.resolve(resource_id)
        existed = os.path.exists(record.path)
        if os.path.isdir(record.path):
            shutil.rmtree(record.path)
        elif os.path.isfile(record.path):
            os.remove(record.path)
        self._records.pop(record.resource_id, None)
        self._path_to_id.pop(record.path, None)
        return existed


class ResourceService(BaseService):
    """Protocol Resource tools backed by a local ResourceRegistry."""

    def __init__(self, bus: ToolBus, registry: ResourceRegistry) -> None:
        super().__init__(bus, "resource_service")
        self._registry = registry

    def bind(self) -> None:
        for spec, handler in (
            (GET_RESOURCE_INFO, self.get_resource_info),
            (LIST_RESOURCES, self.list_resources),
            (LIST_RESOURCE_ENTRIES, self.list_resource_entries),
            (READ_RESOURCE, self.read_resource),
            (DELETE_RESOURCE, self.delete_resource),
            (SNAPSHOT_RESOURCE, self.snapshot_resource),
            (PREPARE_RESOURCE_ARCHIVE, self.prepare_resource_archive),
        ):
            self._register_tool(
                spec.name,
                handler,
                input_schema=spec.input_schema,
                output_schema=spec.output_schema,
                description=spec.description,
            )

    def get_resource_info(self, resource_id: str) -> Dict[str, Any]:
        try:
            return ToolBus.make_result(True, result={"resource": self._registry.describe(resource_id)}, message="OK")
        except Exception as exc:
            return ToolBus.make_result(False, result={}, message=str(exc))

    def list_resources(
        self,
        domain: Optional[str] = None,
        kind: Optional[str] = None,
        cursor: Optional[str] = None,
        limit: Optional[int] = None,
    ) -> Dict[str, Any]:
        try:
            resources = self._registry.list_resources(domain=domain, kind=kind)
            start = int(cursor or 0)
            count = max(1, int(limit or 500))
            page = resources[start:start + count]
            next_cursor = str(start + count) if start + count < len(resources) else None
            result: Dict[str, Any] = {"resources": page}
            if next_cursor is not None:
                result["next_cursor"] = next_cursor
            return ToolBus.make_result(True, result=result, message="OK")
        except Exception as exc:
            return ToolBus.make_result(False, result={"resources": []}, message=str(exc))

    def list_resource_entries(
        self,
        resource_id: str,
        recursive: bool = False,
        cursor: Optional[str] = None,
        limit: Optional[int] = None,
    ) -> Dict[str, Any]:
        try:
            record = self._registry.resolve(resource_id)
            if record.type != "directory" or not os.path.isdir(record.path):
                return ToolBus.make_result(False, result={"entries": []}, message="resource is not a directory")
            entries = [
                self._entry_resource(path, root=record.path)
                for path in _iter_entries(record.path, recursive=bool(recursive))
            ]
            start = int(cursor or 0)
            count = max(1, int(limit or 100))
            page = entries[start:start + count]
            next_cursor = str(start + count) if start + count < len(entries) else None
            result: Dict[str, Any] = {"entries": page}
            if next_cursor is not None:
                result["next_cursor"] = next_cursor
            return ToolBus.make_result(True, result=result, message="OK")
        except Exception as exc:
            return ToolBus.make_result(False, result={"entries": []}, message=str(exc))

    def read_resource(
        self,
        resource_id: str,
        offset: int = 0,
        limit: Optional[int] = None,
    ) -> Dict[str, Any]:
        try:
            record = self._registry.resolve(resource_id)
            if record.type not in ("file", "archive") or not os.path.isfile(record.path):
                return ToolBus.make_result(False, result={}, message="resource is not readable bytes")
            size = os.path.getsize(record.path)
            start = max(0, int(offset or 0))
            max_bytes = max(1, min(int(limit or 1024 * 1024), 8 * 1024 * 1024))
            with open(record.path, "rb") as f:
                f.seek(start)
                data = f.read(max_bytes)
            next_offset = start + len(data)
            return ToolBus.make_result(
                True,
                result={
                    "resource_id": record.resource_id,
                    "offset": start,
                    "next_offset": next_offset,
                    "data": base64.b64encode(data).decode("ascii"),
                    "encoding": "base64",
                    "eof": next_offset >= size,
                },
                message="OK",
            )
        except Exception as exc:
            return ToolBus.make_result(False, result={}, message=str(exc))

    def delete_resource(self, resource_id: str) -> Dict[str, Any]:
        rid = str(resource_id or "").strip()
        if not rid:
            return ToolBus.make_result(False, result={"deleted": False}, message="resource_id is required")
        try:
            record = self._registry.resolve(rid)
            if record.domain != "log":
                return ToolBus.make_result(False, result={"deleted": False}, message="only log resources can be deleted")
            deleted = self._registry.delete(rid)
        except Exception as exc:
            return ToolBus.make_result(False, result={"deleted": False}, message=str(exc))
        return ToolBus.make_result(True, result={"deleted": deleted}, message="OK")

    def snapshot_resource(self, resource_id: str) -> Dict[str, Any]:
        try:
            record = self._registry.resolve(resource_id)
            if record.type == "directory":
                return self.prepare_resource_archive(resource_id=record.resource_id)
            if record.mode == "snapshot":
                return ToolBus.make_result(True, result={"resource": self._registry.describe(record.resource_id)}, message="OK")
            target = os.path.join(self._registry.tmp_dir, f"{safe_name(record.name)}_{int(time.time())}")
            shutil.copy2(record.path, target)
            resource = self._registry.register_path(
                target,
                resource_type=record.type,
                domain=record.domain,
                name=record.name,
                format=record.format,
                mode="snapshot",
                metadata=record.metadata,
            )
            return ToolBus.make_result(True, result={"resource": resource}, message="OK")
        except Exception as exc:
            return ToolBus.make_result(False, result={}, message=str(exc))

    def prepare_resource_archive(
        self,
        resource_id: Optional[str] = None,
        resource_ids: Optional[Iterable[str]] = None,
        format: str = "zip",
    ) -> Dict[str, Any]:
        try:
            if str(format or "zip") != "zip":
                return ToolBus.make_result(False, result={}, message="only zip archive format is supported")
            ids = [str(resource_id)] if resource_id else [str(item) for item in (resource_ids or [])]
            if not ids:
                return ToolBus.make_result(False, result={}, message="resource_id or resource_ids is required")
            records = [self._registry.resolve(rid) for rid in ids]
            archive_name = safe_name(records[0].name if len(records) == 1 else f"resources_{int(time.time())}")
            archive_path = os.path.join(self._registry.tmp_dir, f"{archive_name}.zip")
            archive_path = _unique_path(archive_path)
            with zipfile.ZipFile(archive_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
                for record in records:
                    _write_resource_to_zip(zf, record.path, arc_root=safe_name(record.name))
            resource = self._registry.register_path(
                archive_path,
                resource_type="archive",
                domain=records[0].domain if len(records) == 1 else "data",
                name=os.path.basename(archive_path),
                format="zip",
                mode="snapshot",
                metadata={"source_resource_ids": [record.resource_id for record in records]},
            )
            return ToolBus.make_result(True, result={"resource": resource}, message="OK")
        except Exception as exc:
            return ToolBus.make_result(False, result={}, message=str(exc))

    def _entry_resource(self, path: str, *, root: str) -> Dict[str, Any]:
        rel = os.path.relpath(path, root)
        resource = self._registry.register_path(
            path,
            domain="data",
            name=rel,
            mode="snapshot",
            metadata={"relative_path": rel},
        )
        return resource


def _resource_id(path: str) -> str:
    token = f"{os.path.abspath(path)}:{time.time_ns()}".encode("utf-8")
    return "res_" + hashlib.sha256(token).hexdigest()[:16]


def _resource_format(path: str, rtype: str) -> str:
    if rtype == "directory":
        return "directory"
    if rtype == "archive":
        return "zip"
    ext = os.path.splitext(path)[1].lstrip(".").lower()
    return ext or "bin"


def _mime_type(path: str, fmt: str) -> str:
    if fmt == "zip":
        return "application/zip"
    return mimetypes.guess_type(path)[0] or "application/octet-stream"


def _sha256_file(path: str) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
    return "sha256:" + digest.hexdigest()


def _directory_size(path: str) -> int:
    total = 0
    for dirpath, _dirnames, filenames in os.walk(path):
        for name in filenames:
            file_path = os.path.join(dirpath, name)
            if os.path.isfile(file_path):
                total += os.path.getsize(file_path)
    return total


def _iter_entries(path: str, *, recursive: bool) -> Iterable[str]:
    if recursive:
        for dirpath, dirnames, filenames in os.walk(path):
            for name in sorted(dirnames):
                yield os.path.join(dirpath, name)
            for name in sorted(filenames):
                yield os.path.join(dirpath, name)
        return
    for name in sorted(os.listdir(path)):
        yield os.path.join(path, name)


def _iter_files(path: str) -> Iterable[str]:
    for dirpath, _dirnames, filenames in os.walk(path):
        for name in sorted(filenames):
            yield os.path.join(dirpath, name)


def _iter_collection_dirs(path: str) -> Iterable[str]:
    for dirpath, dirnames, filenames in os.walk(path):
        if "collection_meta.json" in filenames:
            yield dirpath
            dirnames[:] = []


def _read_collection_meta(path: str) -> Dict[str, Any]:
    meta_path = os.path.join(path, "collection_meta.json")
    try:
        with open(meta_path, "r", encoding="utf-8") as f:
            meta = json.load(f)
    except Exception:
        return {}
    if not isinstance(meta, dict):
        return {}
    user_meta = meta.get("metadata") if isinstance(meta.get("metadata"), dict) else {}
    return {
        "collection_id": str(meta.get("collection_id") or user_meta.get("collection_id") or ""),
        "episode_id": str(meta.get("episode_id") or user_meta.get("episode_id") or ""),
        "task_prompt": str(user_meta.get("task_prompt") or ""),
        "task_description": str(user_meta.get("task_description") or ""),
        "started_at_unix": float(meta.get("started_at_unix") or 0.0),
    }


def _collection_resource_name(path: str, meta: Dict[str, Any]) -> str:
    collection_id = str(meta.get("collection_id") or "")
    episode_id = str(meta.get("episode_id") or "")
    if collection_id and episode_id:
        return f"{collection_id}/{episode_id}"
    return os.path.basename(os.path.abspath(path))


def _write_resource_to_zip(zf: zipfile.ZipFile, path: str, *, arc_root: str) -> None:
    if os.path.isdir(path):
        for dirpath, _dirnames, filenames in os.walk(path):
            for name in sorted(filenames):
                file_path = os.path.join(dirpath, name)
                rel = os.path.relpath(file_path, path)
                zf.write(file_path, os.path.join(arc_root, rel))
        return
    zf.write(path, arc_root)


def _unique_path(path: str) -> str:
    root, ext = os.path.splitext(path)
    candidate = path
    suffix = 2
    while os.path.exists(candidate):
        candidate = f"{root}_{suffix}{ext}"
        suffix += 1
    return candidate
