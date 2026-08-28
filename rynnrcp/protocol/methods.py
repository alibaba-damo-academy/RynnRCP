"""RCP protocol method definitions."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict


@dataclass(frozen=True)
class MethodSpec:
    name: str
    input_schema: Dict[str, Any]
    output_schema: Any
    description: str


GET_MANIFEST = MethodSpec(
    name="get_manifest",
    input_schema={},
    output_schema={
        "robot_id": "str",
        "robot_name": "str",
        "embodiment_type": "str",
        "components": "list[dict]",
        "observations": "list[dict]",
        "actions": "list[dict]",
        "capabilities": "dict",
        "model_refs": "dict",
        "metadata": "dict",
    },
    description="Get the RCP robot manifest.",
)

LIST_OBSERVATIONS = MethodSpec(
    name="list_observations",
    input_schema={},
    output_schema={
        "observations": [
            {
                "name": "str",
                "type": "str",
                "component_name": "str  # optional",
                "description": "str",
                "frame_rate": "float  # optional",
                "value_schema": "dict",
            }
        ],
    },
    description="List RCP Observation descriptors.",
)

GET_OBSERVATIONS = MethodSpec(
    name="get_observations",
    input_schema={
        "names": "list[str]  # required Observation names",
        "sync": "bool  # optional; current server only supports false",
    },
    output_schema={
        "observations": [
            {
                "name": "str",
                "timestamp": "float",
                "value": "any",
            }
        ],
    },
    description="Get latest RCP Observation values.",
)

SUBSCRIBE_OBSERVATIONS = MethodSpec(
    name="subscribe_observations",
    input_schema={
        "names": "list[str]  # required Observation names",
        "sync": "bool  # optional; current server only supports false",
        "stream_hz": "float  # optional stream sample rate",
    },
    output_schema=GET_OBSERVATIONS.output_schema,
    description="Stream latest RCP Observation values.",
)

LIST_ACTIONS = MethodSpec(
    name="list_actions",
    input_schema={},
    output_schema={
        "actions": [
            {
                "name": "str",
                "type": "str",
                "component_name": "str  # optional",
                "description": "str",
                "frame_rate": "float",
                "input_schema": "dict",
            }
        ],
    },
    description="List RCP Action descriptors.",
)

RUN_ACTION_CHUNK = MethodSpec(
    name="run_action_chunk",
    input_schema={
        "name": "str  # target Action name",
        "frames": "list[dict]  # each item is one Action input frame",
        "frame_rate": "float  # target execution frame rate",
    },
    output_schema={"accepted_frames": "int"},
    description="Submit a finite RCP Action frame chunk.",
)

RUN_ACTION_CHUNK_ASYNC = MethodSpec(
    name="run_action_chunk_async",
    input_schema=RUN_ACTION_CHUNK.input_schema,
    output_schema=RUN_ACTION_CHUNK.output_schema,
    description="Submit an RCP Action chunk through a streaming response interface.",
)

STOP_ACTION = MethodSpec(
    name="stop_action",
    input_schema={"reason": "str  # optional stop reason"},
    output_schema={"stopped": "bool"},
    description="Stop current RCP Action execution.",
)

LIST_POLICIES = MethodSpec(
    name="list_policies",
    input_schema={},
    output_schema={
        "policies": [
            {
                "policy_id": "str",
                "name": "str",
                "description": "str  # optional",
                "inputs": "dict",
                "outputs": "dict",
            }
        ],
        "active_policy_id": "str  # optional",
        "last_error": "str",
    },
    description="List local RCP policies.",
)

START_POLICY = MethodSpec(
    name="start_policy",
    input_schema={
        "policy_id": "str",
        "runtime_inputs": "dict  # optional latest runtime inputs",
    },
    output_schema={"active_policy_id": "str"},
    description="Start or switch the active RCP policy.",
)

UPDATE_POLICY_INPUTS = MethodSpec(
    name="update_policy_inputs",
    input_schema={
        "policy_id": "str  # optional; defaults to active policy",
        "runtime_inputs": "dict",
    },
    output_schema={"active_policy_id": "str", "runtime_inputs": "dict"},
    description="Update latest runtime inputs for the active RCP policy.",
)

STOP_POLICY = MethodSpec(
    name="stop_policy",
    input_schema={
        "policy_id": "str  # optional; defaults to active policy",
        "reason": "str  # optional stop reason",
    },
    output_schema={"stopped_policy_id": "str"},
    description="Stop the active RCP policy.",
)

GET_HEALTH = MethodSpec(
    name="get_health",
    input_schema={},
    output_schema={"msg": "list"},
    description="Get current RCP health messages.",
)

SUBSCRIBE_HEALTH = MethodSpec(
    name="subscribe_health",
    input_schema={
        "stream_hz": "float  # optional stream sample rate",
    },
    output_schema=GET_HEALTH.output_schema,
    description="Stream current RCP health messages.",
)

GET_RESOURCE_INFO = MethodSpec(
    name="get_resource_info",
    input_schema={"resource_id": "str"},
    output_schema={"resource": "dict"},
    description="Get RCP Resource metadata.",
)

LIST_RESOURCES = MethodSpec(
    name="list_resources",
    input_schema={
        "domain": "str  # optional",
        "kind": "str  # optional metadata.kind filter",
        "cursor": "str  # optional",
        "limit": "int  # optional",
    },
    output_schema={"resources": "list[dict]", "next_cursor": "str  # optional"},
    description="List RCP resources available from this instance.",
)

LIST_RESOURCE_ENTRIES = MethodSpec(
    name="list_resource_entries",
    input_schema={
        "resource_id": "str",
        "recursive": "bool  # optional",
        "cursor": "str  # optional",
        "limit": "int  # optional",
    },
    output_schema={"entries": "list[dict]", "next_cursor": "str  # optional"},
    description="List child resources of a directory Resource.",
)

READ_RESOURCE = MethodSpec(
    name="read_resource",
    input_schema={
        "resource_id": "str",
        "offset": "int  # optional",
        "limit": "int  # optional",
    },
    output_schema={
        "resource_id": "str",
        "offset": "int",
        "next_offset": "int",
        "data": "str  # base64",
        "encoding": "base64",
        "eof": "bool",
    },
    description="Read a byte Resource chunk.",
)

DELETE_RESOURCE = MethodSpec(
    name="delete_resource",
    input_schema={"resource_id": "str"},
    output_schema={"deleted": "bool"},
    description="Delete a log or managed temporary Resource.",
)

SNAPSHOT_RESOURCE = MethodSpec(
    name="snapshot_resource",
    input_schema={"resource_id": "str"},
    output_schema={"resource": "dict"},
    description="Create a stable snapshot Resource.",
)

PREPARE_RESOURCE_ARCHIVE = MethodSpec(
    name="prepare_resource_archive",
    input_schema={
        "resource_id": "str  # optional",
        "resource_ids": "list[str]  # optional",
        "format": "str  # optional; zip only",
    },
    output_schema={"resource": "dict"},
    description="Create an archive Resource from a directory or resource list.",
)

START_COLLECTION = MethodSpec(
    name="start_collection",
    input_schema={
        "names": "list[str]  # Observation and Action object names to record",
        "collection_id": "str",
        "episode_id": "str",
        "task_prompt": "str",
        "task_description": "str",
        "frame_rate": "float  # optional",
        "max_duration": "float  # optional seconds",
        "metadata": "dict  # optional user/application metadata",
    },
    output_schema={
        "collection_id": "str",
        "episode_id": "str",
        "collection_resource": "dict",
        "names": "list[str]",
        "task_prompt": "str",
        "task_description": "str",
        "started_at": "float",
    },
    description="Start RCP data collection.",
)

STOP_COLLECTION = MethodSpec(
    name="stop_collection",
    input_schema={},
    output_schema={
        "collection_id": "str",
        "episode_id": "str",
        "duration": "float",
        "per_name_counts": "dict",
        "collection_resource": "dict",
    },
    description="Stop the active RCP data collection.",
)

GET_COLLECTION_STATUS = MethodSpec(
    name="get_collection_status",
    input_schema={},
    output_schema="dict",
    description="Get active RCP data collection status.",
)

DELETE_COLLECTION = MethodSpec(
    name="delete_collection",
    input_schema={"resource_id": "str"},
    output_schema={"deleted": "bool"},
    description="Delete collected data by collection Resource id.",
)

PROTOCOL_METHODS = {
    spec.name: spec
    for spec in (
        GET_MANIFEST,
        LIST_OBSERVATIONS,
        GET_OBSERVATIONS,
        LIST_ACTIONS,
        RUN_ACTION_CHUNK,
        STOP_ACTION,
        LIST_POLICIES,
        START_POLICY,
        UPDATE_POLICY_INPUTS,
        STOP_POLICY,
        GET_HEALTH,
        GET_RESOURCE_INFO,
        LIST_RESOURCES,
        LIST_RESOURCE_ENTRIES,
        READ_RESOURCE,
        DELETE_RESOURCE,
        SNAPSHOT_RESOURCE,
        PREPARE_RESOURCE_ARCHIVE,
        START_COLLECTION,
        STOP_COLLECTION,
        GET_COLLECTION_STATUS,
        DELETE_COLLECTION,
    )
}
