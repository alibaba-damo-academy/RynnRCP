"""OSS upload helpers for the RynnBot app."""

from __future__ import annotations

import logging
import os
import zipfile
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional

logger = logging.getLogger(__name__)


@dataclass
class OSSCredential:
    """OSS credential and destination bucket."""

    access_key_id: str
    access_key_secret: str
    security_token: str = ""
    bucket: str = ""
    prefix: str = ""
    endpoint: str = ""
    region: str = ""
    expires_at: str = ""


class OSSManager:
    """Small wrapper around ``alibabacloud_oss_v2`` multipart upload."""

    def __init__(self, cred: OSSCredential):
        """Create an OSSManager with the given credentials."""
        self.cred = cred
        self.client = self._create_client(cred)

    @staticmethod
    def _create_client(cred: OSSCredential) -> Any:
        """Create an OSS client using the provided credentials."""
        oss = _oss_module()

        if cred.security_token:
            credentials_provider = oss.credentials.StaticCredentialsProvider(
                access_key_id=cred.access_key_id,
                access_key_secret=cred.access_key_secret,
                security_token=cred.security_token,
            )
        else:
            credentials_provider = oss.credentials.StaticCredentialsProvider(
                access_key_id=cred.access_key_id,
                access_key_secret=cred.access_key_secret,
            )

        cfg = oss.config.load_default()
        cfg.credentials_provider = credentials_provider

        cfg.endpoint = cred.endpoint
        cfg.region = cred.region

        return oss.Client(cfg)

    def _join_key(self, key: str) -> str:
        """Join the configured prefix with the given object key."""
        p = (self.cred.prefix or "").strip("/")
        k = key.lstrip("/")
        return f"{p}/{k}" if p else k

    def multipart_upload_file(
        self,
        file_path: str,
        key: str,
        part_size: int = 5 * 1024 * 1024,
        progress_fn: Optional[Callable[[int, int, int], None]] = None,
        abort_on_fail: bool = True,
    ) -> Dict[str, Any]:
        """Upload one local file to OSS."""
        oss = _oss_module()
        if not os.path.isfile(file_path):
            raise FileNotFoundError(file_path)
        if not self.cred.bucket:
            raise ValueError("bucket cannot be empty")
        if part_size <= 0:
            raise ValueError("part_size must be > 0")

        bucket = self.cred.bucket
        object_key = self._join_key(key)
        data_size = os.path.getsize(file_path)

        # 1) initiate multipart upload
        init_res = self.client.initiate_multipart_upload(
            oss.InitiateMultipartUploadRequest(
                bucket=bucket,
                key=object_key,
            )
        )
        upload_id = init_res.upload_id

        upload_parts: List[oss.UploadPart] = []
        part_number = 1

        try:
            # 2) upload parts sequentially
            with open(file_path, "rb") as f:
                for start in range(0, data_size, part_size):
                    n = min(part_size, data_size - start)
                    reader = oss.io_utils.SectionReader(
                        oss.io_utils.ReadAtReader(f),
                        start,
                        n,
                    )

                    up_res = self.client.upload_part(
                        oss.UploadPartRequest(
                            bucket=bucket,
                            key=object_key,
                            upload_id=upload_id,
                            part_number=part_number,
                            body=reader,
                            progress_fn=progress_fn,
                        )
                    )

                    upload_parts.append(
                        oss.UploadPart(part_number=part_number, etag=up_res.etag)
                    )
                    part_number += 1

            # 3) complete multipart upload
            parts = sorted(upload_parts, key=lambda p: p.part_number)
            comp_res = self.client.complete_multipart_upload(
                oss.CompleteMultipartUploadRequest(
                    bucket=bucket,
                    key=object_key,
                    upload_id=upload_id,
                    complete_multipart_upload=oss.CompleteMultipartUpload(parts=parts),
                )
            )

            return {
                "bucket": comp_res.bucket,
                "key": comp_res.key,
                "etag": comp_res.etag,
                "location": comp_res.location,
                "version_id": comp_res.version_id,
                "request_id": comp_res.request_id,
                "status_code": comp_res.status_code,
            }

        except Exception:
            if abort_on_fail and upload_id:
                try:
                    self.client.abort_multipart_upload(
                        oss.AbortMultipartUploadRequest(
                            bucket=bucket,
                            key=object_key,
                            upload_id=upload_id,
                        )
                    )
                except Exception:
                    pass
            raise

    def multipart_upload_from_bytes(
        self,
        data: bytes,
        key: str,
        part_size: int = 5 * 1024 * 1024,
        progress_fn: Optional[Callable[[int, int, int], None]] = None,
        abort_on_fail: bool = True,
    ) -> Dict[str, Any]:
        """Upload bytes to OSS."""
        oss = _oss_module()
        if not self.cred.bucket:
            raise ValueError("bucket cannot be empty")
        if part_size <= 0:
            raise ValueError("part_size must be > 0")

        bucket = self.cred.bucket
        object_key = self._join_key(key)
        data_size = len(data)

        # 1) initiate multipart upload
        init_res = self.client.initiate_multipart_upload(
            oss.InitiateMultipartUploadRequest(
                bucket=bucket,
                key=object_key,
            )
        )
        upload_id = init_res.upload_id

        upload_parts: List[oss.UploadPart] = []
        part_number = 1

        try:
            for start in range(0, data_size, part_size):
                n = min(part_size, data_size - start)
                part_data = data[start:start + n]

                up_res = self.client.upload_part(
                    oss.UploadPartRequest(
                        bucket=bucket,
                        key=object_key,
                        upload_id=upload_id,
                        part_number=part_number,
                            body=part_data,
                        progress_fn=progress_fn,
                    )
                )

                upload_parts.append(
                    oss.UploadPart(part_number=part_number, etag=up_res.etag)
                )
                part_number += 1

            parts = sorted(upload_parts, key=lambda p: p.part_number)
            comp_res = self.client.complete_multipart_upload(
                oss.CompleteMultipartUploadRequest(
                    bucket=bucket,
                    key=object_key,
                    upload_id=upload_id,
                    complete_multipart_upload=oss.CompleteMultipartUpload(parts=parts),
                )
            )

            return {
                "bucket": comp_res.bucket,
                "key": comp_res.key,
                "etag": comp_res.etag,
                "location": comp_res.location,
                "version_id": comp_res.version_id,
                "request_id": comp_res.request_id,
                "status_code": comp_res.status_code,
            }

        except Exception:
            if abort_on_fail and upload_id:
                try:
                    self.client.abort_multipart_upload(
                        oss.AbortMultipartUploadRequest(
                            bucket=bucket,
                            key=object_key,
                            upload_id=upload_id,
                        )
                    )
                except Exception:
                    pass
            raise

def upload_package_if_requested(
    package_path: str,
    *,
    params: Dict[str, Any],
    kind: str,
) -> Optional[Dict[str, Any]]:
    oss_credential = params.get("oss_credential")
    if not oss_credential:
        return None
    if not isinstance(oss_credential, dict):
        raise TypeError("params.oss_credential must be a dict")

    manager = OSSManager(build_oss_credential(oss_credential))
    part_size = int(params.get("part_size") or 5 * 1024 * 1024)
    if kind == "skill_record":
        return upload_zip_members_to_oss(manager, package_path, part_size)

    key = str(params.get("oss_key") or "import_dataset.zip")
    logger.info("[RynnBot][OSS] upload package zip: %s -> %s", package_path, key)
    result = manager.multipart_upload_file(
        file_path=package_path,
        key=key,
        part_size=part_size,
        progress_fn=oss_progress_logger("tele_data_coll"),
        abort_on_fail=True,
    )
    return {"mode": "zip", "key": key, "result": result}


def build_oss_credential(oss_credential: Dict[str, Any]) -> OSSCredential:
    cred = OSSCredential(
        access_key_id=str(oss_credential.get("access_key_id") or oss_credential.get("accessKeyId") or ""),
        access_key_secret=str(oss_credential.get("access_key_secret") or oss_credential.get("accessKeySecret") or ""),
        security_token=str(oss_credential.get("security_token") or oss_credential.get("securityToken") or ""),
        bucket=str(oss_credential.get("bucket") or ""),
        prefix=str(oss_credential.get("prefix") or ""),
        endpoint=str(oss_credential.get("endpoint") or ""),
        region=str(oss_credential.get("region") or ""),
        expires_at=str(oss_credential.get("expires_at") or oss_credential.get("expiresAt") or ""),
    )
    missing = [
        name
        for name, value in {
            "access_key_id": cred.access_key_id,
            "access_key_secret": cred.access_key_secret,
            "bucket": cred.bucket,
            "endpoint": cred.endpoint,
        }.items()
        if not value
    ]
    if missing:
        raise ValueError(f"oss_credential missing required fields: {missing}")
    return cred


def upload_zip_members_to_oss(manager: OSSManager, package_path: str, part_size: int) -> Dict[str, Any]:
    uploaded = []
    total_bytes = 0
    with zipfile.ZipFile(package_path, "r") as zf:
        for info in zf.infolist():
            if info.is_dir():
                continue
            data = zf.read(info.filename)
            total_bytes += len(data)
            key = skill_record_oss_key(info.filename)
            logger.info("[RynnBot][OSS] upload package member: %s bytes=%d", info.filename, len(data))
            result = manager.multipart_upload_from_bytes(
                data=data,
                key=key,
                part_size=part_size,
                progress_fn=oss_progress_logger("skill_record"),
                abort_on_fail=True,
            )
            uploaded.append({"key": key, "size_bytes": len(data), "result": result})
    return {
        "mode": "files",
        "files": uploaded,
        "file_count": len(uploaded),
        "total_bytes": total_bytes,
    }


def skill_record_oss_key(zip_member_name: str) -> str:
    normalized = str(zip_member_name).replace("\\", "/").lstrip("/")
    if "/" in normalized:
        return normalized.split("/", 1)[1]
    return normalized


def _oss_module():
    try:
        import alibabacloud_oss_v2 as oss
    except ImportError as exc:
        raise RuntimeError("alibabacloud-oss-v2 is required when oss_credential is provided") from exc
    return oss


def oss_progress_logger(tag: str):
    seen: set[tuple[int, int]] = set()

    def progress(part_number: int, written: int, total: int) -> None:
        if total <= 0:
            return
        pct = int(100 * written / total)
        bucket = min(100, (pct // 25) * 25)
        key = (part_number, bucket)
        if bucket in (0, 25, 50, 75, 100) and key not in seen:
            seen.add(key)
            logger.info(
                "[RynnBot][OSS][%s] part=%d progress=%d%% (%d/%d)",
                tag,
                part_number,
                bucket,
                written,
                total,
            )

    return progress
