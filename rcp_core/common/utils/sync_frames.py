# rcp_core/common/utils/sync_frames.py

"""
Timestamp-based multi-key frame synchronization.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This module provides :func:`~rcp_core.common.utils.sync_frames.sync_by_trigger_time`,
a utility to align samples from multiple time-ordered per-key deques into a single
“synchronized” set of values.

Algorithm
---------
Given ``buffers: {key: deque([(ts, value), ...])}``:

1. Determine ``out_keys``:
   - if not provided, use all keys in ``buffers``.

2. Determine ``trigger_keys``:
   - if not provided, defaults to ``out_keys``.
   - if provided, must be a subset of ``out_keys`` (otherwise returns ``None``).

3. Compute ``trigger_time``:
   - for each trigger key, take the **latest** timestamp (deque[-1][0])
   - ``trigger_time = min(latest_ts(trigger_key) for trigger_key in trigger_keys)``
   This effectively chooses a time that all trigger streams have *at least reached*.

4. For every key in ``out_keys``, pick the first sample whose ``ts >= trigger_time``.
   If any key has no such sample, synchronization fails and returns ``None``.

Diagnostics
-----------
- Logs a warning if the computed ``trigger_time`` is more than 70ms older than current time,
  which can indicate stale buffers or lag (but does not fail sync by itself).

Return value
------------
On success: ``List[(key, chosen_ts, chosen_value)]`` aligned across keys.
On failure: ``None``.
"""

from typing import Dict, Deque, Tuple, Any, List, Optional
import time

from rcp_core.common.utils.logger import server_logger

logger = server_logger()


def sync_by_trigger_time(
    buffers: Dict[str, Deque[Tuple[float, Any]]],
    out_keys: Optional[List[str]] = None,
    trigger_keys: Optional[List[str]] = None,
) -> Optional[List[Tuple[str, float, Any]]]:
    """
    Synchronize multi-sensor data by timestamps using a computed trigger time.

    This function takes a snapshot of multiple time-ordered queues (one queue
    per sensor/key), computes a "trigger_time" based on trigger_keys, and then
    for each requested key picks the first sample whose timestamp is >=
    trigger_time. If any requirement is not met, it returns None.

    Parameters
    ----------
    buffers : Dict[str, Deque[Tuple[float, Any]]]
        Snapshot of data queues for each key. Example:
        {
            "observation.images.cam_head": deque[(ts: float, value), ...],
            "observation.joint_position.left": deque[(ts: float, value), ...],
            ...
        }
        Each deque must be ordered by increasing timestamp (append in time order).

    out_keys : list[str] | None, optional
        Keys that should be synchronized and returned.
        - If None or empty, all keys in `buffers` are used.
        - If not None, only these keys are considered in the output.

    trigger_keys : list[str] | None, optional
        Keys used to compute the trigger_time; must be a subset of out_keys.
        The trigger_time is:
            min( latest_timestamp(key) for key in trigger_keys )
        where latest_timestamp(key) is the last timestamp in that key's deque.

        - If None or empty, `out_keys` is used as trigger_keys.
        - If non-empty, all keys must exist in out_keys; otherwise returns None.

    Returns
    -------
    Optional[List[Tuple[str, float, Any]]]
        On success, returns a list of (key, ts, value), where:
          - key ∈ out_keys
          - ts is the chosen timestamp for that key (first ts >= trigger_time)
          - value is the corresponding data

        Returns None if synchronization fails, e.g.:
          - buffers is empty
          - any trigger_key has no data
          - any out_key has no data at or after trigger_time
          - trigger_keys is not a subset of out_keys

    Notes
    -----
    - A warning is printed if the computed trigger_time is too old compared
      to current time (diff > 70 ms). This does not cause failure by itself.
    """

    if not buffers:
        return None

    # 1. Determine out_keys
    if not out_keys:
        out_keys = list(buffers.keys())
    if not out_keys:
        return None

    # 2. Determine trigger_keys and ensure trigger_keys ⊆ out_keys
    if not trigger_keys:
        trigger_keys = out_keys
    else:
        out_set = set(out_keys)
        trig_set = set(trigger_keys)
        if not trig_set.issubset(out_set):
            logger.warning(
                f"[sync_by_trigger_time] trigger_keys is not a subset of out_keys: "
                f"trigger_keys={trigger_keys}, out_keys={out_keys}"
            )
            return None

    # 3. Compute trigger_time as min over trigger_keys
    latest_ts_list: List[float] = []
    for key in trigger_keys:
        q = buffers.get(key)
        if not q:
            # No data for a trigger key, cannot sync
            return None
        latest_ts_list.append(q[-1][0])  # q[-1] is latest (ts, value)

    if not latest_ts_list:
        return None

    trigger_time = min(latest_ts_list)

    # Warn if trigger_time is too old compared to current time
    now_ts = time.time()
    diff = now_ts - trigger_time
    if diff > 0.07:  # 70 ms
        logger.warning(
            f"[sync_by_trigger_time] WARNING: trigger_time is too old: "
            f"now={now_ts:.6f}, trigger_time={trigger_time:.6f}, diff={diff*1000:.1f} ms"
        )

    # 4. For each out_key, find the first sample with ts >= trigger_time
    aligned: List[Tuple[str, float, Any]] = []
    for key in out_keys:
        q = buffers.get(key)
        if not q:
            return None

        chosen_ts = None
        chosen_value = None
        for ts, value in q:
            if ts >= trigger_time:
                chosen_ts = ts
                chosen_value = value
                break

        if chosen_ts is None:
            # No data after trigger_time for this key
            return None

        aligned.append((key, chosen_ts, chosen_value))

    return aligned
