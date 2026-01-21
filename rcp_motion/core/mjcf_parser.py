"""
MJCF Parser - Parse MuJoCo MJCF files for robot configuration

Python equivalent of motion/manager/mjcf_parser.cpp
Extracts robot configuration from MJCF files including:
- Keyframes (home, stand positions)
- Joint limits (TODO)
- Actuator modes (TODO)
- End-effector detection (TODO)
- Site names (TODO)
"""

import numpy as np
import mujoco


class MjcfParser:
    """
    MJCF file parser for extracting robot configuration

    Methods:
    - extractKeyframes: Extract home and stand keyframes from MJCF
    - extractJointLimits: Extract joint position/velocity/torque limits (TODO)
    - detectActuatorModes: Detect actuator control modes (TODO)
    - extractSiteNames: Extract site names from MJCF (TODO)
    """

    @staticmethod
    def extractRobotStateDim(mjcf_path: str) -> dict:
        """
        Initialize robot model from MJCF file.
        """
        mj_model = mujoco.MjModel.from_xml_path(mjcf_path)
        actuator_dim = mj_model.nu
        joint_dim = mj_model.njnt
        site_num = mj_model.nsite

        """ ft sensor is define as total num of force sensor """
        ft_sensor_num = 0
        for i in range(mj_model.nsensor):
            sensor_type = mj_model.sensor_type[i]
            if sensor_type == mujoco.mjtSensor.mjSENS_FORCE:
                ft_sensor_num += 1

        """ if gripper name define as "grip", this joint will checkout as gripper joint """
        gripper_dim = 0
        gripper_idx = []
        gripper_name = []
        for i in range(actuator_dim):
            name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name and ("grip" in name.lower()):
                gripper_dim += 1
                gripper_idx.append(i)
                gripper_name.append(name)

        ee_num = 0
        ee_idx = []
        ee_site_name = []
        for i in range(site_num):
            name = mujoco.mj_id2name(mj_model, mujoco.mjtObj.mjOBJ_SITE, i)
            if name and ("ee" in name.lower()):
                ee_num += 1
                ee_idx.append(i)
                ee_site_name.append(name)

        # Build result with both new indexed format and backward compatibility
        result = {
            "joint_dim": joint_dim,
            "actuator_dim": actuator_dim,
            "ee_num": ee_num,
            "gripper_dim": gripper_dim,
            "ft_sensor_num": ft_sensor_num,
            "site_num": site_num,
            "gripper_joint_idx": gripper_idx,
            "gripper_joint_name": gripper_name,
            "ee_site_idx": ee_idx,
            "ee_site_name": ee_site_name,
        }
        return result

    @staticmethod
    def extractKeyframes(mjcf_path: str) -> dict:
        """
        Extract keyframe configurations from MJCF file

        Extracts ALL keyframes and organizes them using universal indexing:
        - Index 0: "home" - Home/initial position
        - Index 1: "standby1" - First standby position
        - Index 2: "standby2" - Second standby position
        - Index 3+: Extra keyframes (e.g., "standby3", "rest", etc.)

        Args:
            mjcf_path: Path to MJCF file (e.g., fr3_pinocchio.xml)

        Returns:
            Dictionary with keyframe data:
            {
                'keyframes': list[np.ndarray],  # Indexed keyframes [q0, q1, q2, ...]
                'names': list[str],              # Keyframe names for reference
                'nq': int,                       # Number of position DOF
                'home': np.ndarray,              # Backward compat - same as keyframes[0]
                'stand': np.ndarray,             # Backward compat - same as keyframes[1]
            }

        Example:
            >>> kf = MjcfParser.extractKeyframes("fr3_pinocchio.xml")
            >>> q_home = kf['keyframes'][0]      # Index 0: home
            >>> q_standby1 = kf['keyframes'][1]  # Index 1: standby1
            >>> q_standby2 = kf['keyframes'][2]  # Index 2: standby2
        """
        # Load MuJoCo model from MJCF file
        model = mujoco.MjModel.from_xml_path(mjcf_path)
        data = mujoco.MjData(model)

        nq = model.nq  # Number of position coordinates

        # Universal keyframe mapping (index -> expected name)
        UNIVERSAL_KEYFRAME_NAMES = ["home", "standby1", "standby2"]

        keyframes_list = []
        names_list = []

        # Extract keyframes following universal mapping
        for expected_name in UNIVERSAL_KEYFRAME_NAMES:
            key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, expected_name)
            if key_id != -1:
                mujoco.mj_resetDataKeyframe(model, data, key_id)
                keyframes_list.append(data.qpos.copy())
                names_list.append(expected_name)
            else:
                # If keyframe not found, use zeros as placeholder
                keyframes_list.append(np.zeros(nq))
                names_list.append(expected_name)

        # Extract any additional keyframes beyond the universal set
        # (e.g., SO101's "rest" keyframe)
        for key_idx in range(model.nkey):
            key_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_KEY, key_idx)
            if key_name and key_name not in UNIVERSAL_KEYFRAME_NAMES:
                mujoco.mj_resetDataKeyframe(model, data, key_idx)
                keyframes_list.append(data.qpos.copy())
                names_list.append(key_name)

        # Build result with both new indexed format and backward compatibility
        result = {
            "keyframes": keyframes_list,
            "names": names_list,
            "nq": nq,
            # Backward compatibility
            "home": keyframes_list[0] if len(keyframes_list) > 0 else np.zeros(nq),
            "stand": keyframes_list[1] if len(keyframes_list) > 1 else np.zeros(nq),
        }

        return result

    @staticmethod
    def extractJointLimits(mjcf_path: str) -> dict:
        """
        Extract joint limits from MJCF file

        TODO: Implement extraction of:
        - qPosMin, qPosMax: Position limits
        - qVelMax: Velocity limits
        - qTorqueMax: Torque/force limits

        Args:
            mjcf_path: Path to MJCF file

        Returns:
            Dictionary with limit data
        """
        # TODO: Implement
        pass

    @staticmethod
    def detectActuatorModes(mjcf_path: str) -> dict:
        """
        Detect actuator control modes from MJCF file

        TODO: Implement detection of:
        - Position control actuators
        - Velocity control actuators
        - Torque control actuators
        - End-effector/gripper actuators

        Args:
            mjcf_path: Path to MJCF file

        Returns:
            Dictionary with actuator mode information
        """
        # TODO: Implement
        pass

    @staticmethod
    def extractSiteNames(mjcf_path: str) -> list:
        """
        Extract all site names from MJCF file

        Sites in MJCF are attachment points for:
        - End effectors (EE)
        - Sensors (cameras, force sensors)
        - Visual markers
        - Key reference points (shoulderSite, elbowSite, wristSite)

        Args:
            mjcf_path: Path to MJCF file

        Returns:
            List of site names

        Example:
            >>> sites = MjcfParser.extractSiteNames("fr3_pinocchio.xml")
            >>> print(sites)
            ['EE', 'shoulderSite', 'elbowSite', 'wristSite']
        """
        # TODO: Implement
        # model = mujoco.MjModel.from_xml_path(mjcf_path)
        # site_names = []
        # for i in range(model.nsite):
        #     name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_SITE, i)
        #     if name:
        #         site_names.append(name)
        # return site_names
        pass
