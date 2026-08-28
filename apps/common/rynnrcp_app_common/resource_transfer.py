"""Resource download helpers used by app-side workflows."""

from __future__ import annotations

import base64
import json
import logging
import os
import shutil
import time
import zipfile
from typing import Any, Mapping, Optional

from rynnrcp.utils import safe_name


logger = logging.getLogger(__name__)


def download_collection_resource(
    client: Any,
    collection: Mapping[str, Any],
    destination_root: str,
    *,
    timeout_ms: int = 30000,
    chunk_size: int = 4 * 1024 * 1024,
) -> str:
    """Download a collection Resource from a server and return the local episode dir."""

    resource = collection.get("collection_resource")
    if not isinstance(resource, Mapping):
        raise RuntimeError("collection_resource is required")
    resource_id = str(resource.get("resource_id") or "")
    if not resource_id:
        raise RuntimeError("collection_resource.resource_id is required")

    collection_id = str(collection.get("collection_id") or _metadata_value(resource, "collection_id") or "")
    episode_id = str(collection.get("episode_id") or _metadata_value(resource, "episode_id") or "")
    if not collection_id or not episode_id:
        raise RuntimeError("collection_id and episode_id are required")

    archive = client.prepare_resource_archive(resource_id=resource_id, timeout_ms=timeout_ms)
    if not archive.ok:
        raise RuntimeError(archive.message or "prepare_resource_archive failed")
    archive_resource = archive.payload.get("resource") if isinstance(archive.payload, Mapping) else None
    if not isinstance(archive_resource, Mapping) or not archive_resource.get("resource_id"):
        raise RuntimeError("prepare_resource_archive did not return a Resource")

    root = os.path.abspath(os.path.expanduser(str(destination_root)))
    final_dir = os.path.join(root, safe_name(collection_id), safe_name(episode_id))
    staging_root = os.path.join(root, "_downloads")
    staging_dir = os.path.join(staging_root, f"{safe_name(str(archive_resource['resource_id']))}_{int(time.time())}")
    zip_path = os.path.join(staging_dir, "resource.zip")

    try:
        if os.path.isdir(final_dir):
            shutil.rmtree(final_dir)
        os.makedirs(staging_dir, exist_ok=True)
        try:
            _read_resource_to_file(
                client,
                str(archive_resource["resource_id"]),
                zip_path,
                timeout_ms=timeout_ms,
                chunk_size=chunk_size,
            )
            _extract_zip(zip_path, staging_dir)
            episode_dir = _find_collection_dir(staging_dir, collection_id=collection_id, episode_id=episode_id)
            if episode_dir is None:
                raise RuntimeError("downloaded Resource does not contain collection_meta.json")
            os.makedirs(os.path.dirname(final_dir), exist_ok=True)
            shutil.move(episode_dir, final_dir)
            return final_dir
        finally:
            shutil.rmtree(staging_dir, ignore_errors=True)
    finally:
        _release_temporary_resource(
            client,
            str(archive_resource["resource_id"]),
            timeout_ms=timeout_ms,
        )


def download_collection_entries(
    client: Any,
    collection: Mapping[str, Any],
    destination_root: str,
    relative_paths: list[str],
    *,
    timeout_ms: int = 30000,
    chunk_size: int = 4 * 1024 * 1024,
) -> str:
    """Download selected files from a collection Resource and return local episode dir."""

    resource = collection.get("collection_resource")
    if not isinstance(resource, Mapping):
        raise RuntimeError("collection_resource is required")
    resource_id = str(resource.get("resource_id") or "")
    if not resource_id:
        raise RuntimeError("collection_resource.resource_id is required")

    collection_id = str(collection.get("collection_id") or _metadata_value(resource, "collection_id") or "")
    episode_id = str(collection.get("episode_id") or _metadata_value(resource, "episode_id") or "")
    if not collection_id or not episode_id:
        raise RuntimeError("collection_id and episode_id are required")

    wanted = {str(path).strip("/") for path in relative_paths if str(path).strip("/")}
    if not wanted:
        raise RuntimeError("relative_paths is required")

    root = os.path.abspath(os.path.expanduser(str(destination_root)))
    final_dir = os.path.join(root, safe_name(collection_id), safe_name(episode_id))
    if os.path.isdir(final_dir):
        shutil.rmtree(final_dir)
    os.makedirs(final_dir, exist_ok=True)

    entries = _list_resource_entries(client, resource_id, timeout_ms=timeout_ms)
    by_rel = {
        str((entry.get("metadata") or {}).get("relative_path") or entry.get("name") or "").strip("/"): entry
        for entry in entries
        if isinstance(entry, Mapping)
    }
    missing = sorted(path for path in wanted if path not in by_rel)
    if missing:
        shutil.rmtree(final_dir, ignore_errors=True)
        raise RuntimeError("collection Resource missing entries: " + ", ".join(missing))

    try:
        for rel in sorted(wanted):
            entry = by_rel[rel]
            target = os.path.abspath(os.path.join(final_dir, rel))
            if os.path.commonpath([final_dir, target]) != final_dir:
                raise RuntimeError(f"unsafe collection entry: {rel}")
            os.makedirs(os.path.dirname(target), exist_ok=True)
            _read_resource_to_file(
                client,
                str(entry["resource_id"]),
                target,
                timeout_ms=timeout_ms,
                chunk_size=chunk_size,
            )
        return final_dir
    except Exception:
        shutil.rmtree(final_dir, ignore_errors=True)
        raise


def _list_resource_entries(client: Any, resource_id: str, *, timeout_ms: int) -> list[Mapping[str, Any]]:
    entries: list[Mapping[str, Any]] = []
    cursor = None
    while True:
        response = client.list_resource_entries(
            resource_id,
            recursive=True,
            cursor=cursor,
            limit=500,
            timeout_ms=timeout_ms,
        )
        if not response.ok:
            raise RuntimeError(response.message or "list_resource_entries failed")
        payload = response.payload if isinstance(response.payload, Mapping) else {}
        entries.extend(item for item in payload.get("entries", []) if isinstance(item, Mapping))
        cursor = payload.get("next_cursor")
        if not cursor:
            return entries


def _read_resource_to_file(
    client: Any,
    resource_id: str,
    path: str,
    *,
    timeout_ms: int,
    chunk_size: int,
) -> None:
    offset = 0
    with open(path, "wb") as f:
        while True:
            response = client.read_resource(
                resource_id,
                offset=offset,
                limit=chunk_size,
                timeout_ms=timeout_ms,
            )
            if not response.ok:
                raise RuntimeError(response.message or "read_resource failed")
            payload = response.payload if isinstance(response.payload, Mapping) else {}
            data = base64.b64decode(str(payload.get("data") or ""))
            f.write(data)
            offset = int(payload.get("next_offset") or offset + len(data))
            if payload.get("eof"):
                break
            if not data:
                raise RuntimeError("read_resource returned an empty non-eof chunk")


def _release_temporary_resource(client: Any, resource_id: str, *, timeout_ms: int) -> None:
    try:
        response = client.delete_resource(resource_id, timeout_ms=timeout_ms)
        if not response.ok:
            logger.warning(
                "Failed to release temporary Resource %s: %s",
                resource_id,
                response.message or "delete_resource failed",
            )
    except Exception as exc:
        logger.warning("Failed to release temporary Resource %s: %s", resource_id, exc)


def _extract_zip(path: str, destination: str) -> None:
    root = os.path.abspath(destination)
    with zipfile.ZipFile(path, "r") as zf:
        for member in zf.infolist():
            target = os.path.abspath(os.path.join(root, member.filename))
            if os.path.commonpath([root, target]) != root:
                raise RuntimeError(f"unsafe zip member: {member.filename}")
        zf.extractall(root)


def _find_collection_dir(root: str, *, collection_id: str, episode_id: str) -> Optional[str]:
    for dirpath, dirnames, filenames in os.walk(root):
        if "collection_meta.json" not in filenames:
            continue
        meta_path = os.path.join(dirpath, "collection_meta.json")
        try:
            with open(meta_path, "r", encoding="utf-8") as f:
                meta = json.load(f)
        except Exception:
            dirnames[:] = []
            continue
        metadata = meta.get("metadata") if isinstance(meta.get("metadata"), Mapping) else {}
        if str(meta.get("collection_id") or metadata.get("collection_id") or "") == collection_id and str(
            meta.get("episode_id") or metadata.get("episode_id") or ""
        ) == episode_id:
            return os.path.abspath(dirpath)
        dirnames[:] = []
    return None


def _metadata_value(resource: Mapping[str, Any], key: str) -> Any:
    metadata = resource.get("metadata")
    if isinstance(metadata, Mapping):
        return metadata.get(key)
    return None
