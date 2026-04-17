# comm_plugin/rynnbot_plugin/oss_manager.py

"""
OSS upload manager for the Rynnbot plugin.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module provides :class:`~comm_plugin.rynnbot_plugin.oss_manager.OSSManager`,
a thin wrapper around ``alibabacloud_oss_v2`` for uploading files to Alibaba Cloud
OSS using either long-term access keys or temporary STS credentials.

Key features
------------
- Uses a single endpoint URL (e.g. ``https://oss-cn-hangzhou.aliyuncs.com``),
  region is optional.
- Supports both AK/SK and STS token authentication via
  :class:`~comm_plugin.rynnbot_plugin.oss_manager.OSSCredential`.
- Provides high-level ``multipart_upload_file`` helper for multipart uploads
  of large local files, with optional progress callback.

Typical usage
-------------
1. Construct :class:`OSSCredential` from an STS response (or static AK/SK).
2. Create an :class:`OSSManager` with the credential.
3. Call :meth:`OSSManager.multipart_upload_file` to upload a local file to OSS.

Progress callback
-----------------
The ``progress_fn`` parameter of :meth:`OSSManager.multipart_upload_file` is passed
through to the SDK's ``upload_part`` call. It should be a callable with signature::

    def progress_fn(part_number: int, written: int, total: int) -> None:
        ...

where:
- ``part_number`` is the current part index.
- ``written`` is the number of bytes written so far for this part.
- ``total`` is the total number of bytes for this part.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Optional, Callable, List, Dict, Any

import alibabacloud_oss_v2 as oss


@dataclass
class OSSCredential:
    """Simple container for OSS credential and bucket information.

    Attributes
    ----------
    access_key_id : str
        AccessKey ID or STS AccessKey ID.
    access_key_secret : str
        AccessKey Secret or STS AccessKey Secret.
    security_token : str
        STS Security Token (leave empty when using long-term AK).
    bucket : str
        Default bucket name for uploads.
    prefix : str
        Optional object key prefix, e.g. ``"tmp/session123/"``.
    endpoint : str
        OSS endpoint, e.g. ``"https://oss-cn-hangzhou.aliyuncs.com"``.
    region : str
        Region ID (e.g. ``"cn-hangzhou"``); optional if endpoint is sufficient.
    expires_at : str
        Optional ISO timestamp for credential expiration (for logging/validation).
    """

    access_key_id: str
    access_key_secret: str
    security_token: str = ""
    bucket: str = ""
    prefix: str = ""
    endpoint: str = ""
    region: str = ""
    expires_at: str = ""


class OSSManager:
    """High-level OSS manager based on ``alibabacloud_oss_v2``.

    The manager encapsulates client initialization and provides a convenient
    multipart-upload helper that automatically:
      - initiates a multipart upload,
      - uploads parts sequentially,
      - completes the multipart upload (or aborts on failure).

    Authentication is configured via :class:`OSSCredential`:
    - If ``security_token`` is provided, STS credentials are used.
    - Otherwise, long-term AK/SK is used.
    """

    def __init__(self, cred: OSSCredential):
        """Create an OSSManager with the given credentials."""
        self.cred = cred
        self.client = self._create_client(cred)

    @staticmethod
    def _create_client(cred: OSSCredential) -> oss.Client:
        """Create an OSS client using the provided credentials."""

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
        """Upload a local file to OSS using multipart upload.

        Parameters
        ----------
        file_path : str
            Path to the local file to be uploaded.
        key : str
            OSS object key (the final key will be ``prefix/key`` if a prefix
            is configured in :class:`OSSCredential`).
        part_size : int, optional
            Size of each part in bytes, by default 5 * 1024 * 1024 (5MB).
        progress_fn : callable, optional
            Progress callback passed through to the SDK; signature::

                progress_fn(part_number: int, written: int, total: int) -> None
        abort_on_fail : bool, optional
            Whether to abort the multipart upload if any exception occurs.

        Returns
        -------
        Dict[str, Any]
            A dict containing fields from ``CompleteMultipartUploadResponse``, e.g.:

            - ``bucket``
            - ``key``
            - ``etag``
            - ``location``
            - ``version_id``
            - ``request_id``
            - ``status_code``

        Raises
        ------
        FileNotFoundError
            If ``file_path`` does not exist.
        ValueError
            If bucket is empty or ``part_size`` is not positive.
        Exception
            Any exception from the underlying SDK; if ``abort_on_fail`` is True
            the multipart upload will be aborted before re-raising.
        """
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
            # On failure, try to abort the multipart upload if requested
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
                    # Ignore abort failures; re-raise original exception
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
        """Upload data from memory (bytes) to OSS using multipart upload.

        This method is useful for uploading files extracted from zip archives
        without writing them to disk first.

        Parameters
        ----------
        data : bytes
            The data to upload as a bytes object.
        key : str
            OSS object key (the final key will be ``prefix/key`` if a prefix
            is configured in :class:`OSSCredential`).
        part_size : int, optional
            Size of each part in bytes, by default 5 * 1024 * 1024 (5MB).
        progress_fn : callable, optional
            Progress callback passed through to the SDK; signature::

                progress_fn(part_number: int, written: int, total: int) -> None
        abort_on_fail : bool, optional
            Whether to abort the multipart upload if any exception occurs.

        Returns
        -------
        Dict[str, Any]
            A dict containing fields from ``CompleteMultipartUploadResponse``, e.g.:

            - ``bucket``
            - ``key``
            - ``etag``
            - ``location``
            - ``version_id``
            - ``request_id``
            - ``status_code``
        """
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
            # 2) upload parts sequentially from memory
            for start in range(0, data_size, part_size):
                n = min(part_size, data_size - start)
                # Create a reader for this part from memory
                part_data = data[start:start + n]

                up_res = self.client.upload_part(
                    oss.UploadPartRequest(
                        bucket=bucket,
                        key=object_key,
                        upload_id=upload_id,
                        part_number=part_number,
                        body=part_data,  # Directly use bytes as body
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
            # On failure, try to abort the multipart upload if requested
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
                    # Ignore abort failures; re-raise original exception
                    pass
            raise
