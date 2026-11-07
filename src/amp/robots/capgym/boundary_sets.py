#
# For licensing see accompanying LICENSE file.
# Copyright (C) 2026 Apple Inc. All Rights Reserved.
#
"""The CaP-Gym boundary's allow-lists: per family, the exact capx modules and the files it may read."""

from __future__ import annotations

from dataclasses import dataclass

CORE = ("capx", "capx.envs", "capx.envs.base", "capx.envs.simulators")

SIM_FRANKA_REAL = ("capx.envs.simulators.franka_real",)

SIM_ROBOSUITE = (
    "capx.envs.simulators.robosuite_base",
    "capx.envs.simulators.robosuite_cube_lift",
    "capx.envs.simulators.robosuite_cubes",
    "capx.envs.simulators.robosuite_cubes_restack",
    "capx.envs.simulators.robosuite_handover",
    "capx.envs.simulators.robosuite_nut_assembly",
    "capx.envs.simulators.robosuite_spill_wipe",
    "capx.envs.simulators.robosuite_two_arm_lift",
)

SIM_LIBERO = ("capx.envs.simulators.libero",)

SIM_BEHAVIOR = ("capx.envs.simulators.r1pro_b1k",)

CONFIGS_INSTANTIATE = ("capx.envs.configs", "capx.envs.configs.instantiate")

CONFIGS_LOADER = ("capx.envs.configs.loader",)

UTILS_SIM = ("capx.utils", "capx.utils.camera_utils", "capx.utils.depth_utils",
             "capx.utils.msgpack_server_client_utils", "capx.utils.video_utils")

UTILS_CHAIN = ("capx.utils.serve_utils", "capx.utils.visualization_utils")


INTEGRATIONS = (
    "capx.integrations",
    "capx.integrations.base_api",
    "capx.integrations.franka",
    "capx.integrations.franka.common",
    "capx.integrations.franka.control",
    "capx.integrations.franka.control_privileged",
    "capx.integrations.franka.control_reduced",
    "capx.integrations.franka.control_reduced_exampleless",
    "capx.integrations.franka.control_reduced_skill_library",
    "capx.integrations.franka.handover",
    "capx.integrations.franka.handover_privileged",
    "capx.integrations.franka.handover_reduced",
    "capx.integrations.franka.handover_reduced_exampleless",
    "capx.integrations.franka.libero",
    "capx.integrations.franka.libero_privileged",
    "capx.integrations.franka.libero_reduced",
    "capx.integrations.franka.libero_reduced_skill_library",
    "capx.integrations.franka.nut_assembly_privileged",
    "capx.integrations.franka.nut_assembly_visual",
    "capx.integrations.franka.spill_wipe",
    "capx.integrations.franka.spill_wipe_privileged",
    "capx.integrations.franka.two_arm_lift",
    "capx.integrations.franka.two_arm_lift_privileged",
    "capx.integrations.libero",
    "capx.integrations.motion",
    "capx.integrations.motion.pyroki",
    "capx.integrations.motion.pyroki_context",
    "capx.integrations.motion.pyroki_snippets",
    "capx.integrations.motion.pyroki_snippets._solve_ik",
    "capx.integrations.motion.pyroki_snippets._solve_ik_vel_cost",
    "capx.integrations.motion.pyroki_snippets._solve_ik_with_base",
    "capx.integrations.motion.pyroki_snippets._solve_ik_with_collision",
    "capx.integrations.motion.pyroki_snippets._solve_ik_with_manipulability",
    "capx.integrations.motion.pyroki_snippets._solve_ik_with_multiple_targets",
    "capx.integrations.motion.pyroki_snippets._solve_ik_with_rest_cost",
    "capx.integrations.motion.pyroki_snippets._trajopt",
    "capx.integrations.r1pro",
    "capx.integrations.vision",
    "capx.integrations.vision.graspnet",
    "capx.integrations.vision.molmo",
    "capx.integrations.vision.owlvit",
    "capx.integrations.vision.sam2",
    "capx.integrations.vision.sam3",
)

ATTEMPT_ROBOSUITE = (
    "capx.envs.simulators.libero",
    "capx.envs.simulators.r1pro_b1k",
    "capx.integrations.r1pro.control",
)
ATTEMPT_LIBERO = (
    "capx.envs.simulators.robosuite_cube_lift",
    "capx.envs.simulators.r1pro_b1k",
    "capx.integrations.r1pro.control",
)


ATTEMPT_BEHAVIOR = (
    "capx.envs.simulators.robosuite_cube_lift",
    "capx.envs.simulators.libero",
)


CONTROLLER = "capx/integrations/robosuite/controllers/config/robots/panda_joint_ctrl.json"
CONTROLLER_2 = "capx/integrations/robosuite/controllers/config/robots/panda_joint_ctrl2.json"



TREE_ROBOSUITE = ("capx/third_party/robosuite/robosuite/",
                  "robosuite, the robosuite venv's simulator: its code, models and assets")
TREE_LIBERO_ROBOSUITE = ("capx/third_party/libero_dependencies/robosuite/robosuite/",
                         "robosuite as LIBERO needs it: its code, models and assets")
TREE_LIBERO_PRO = ("capx/third_party/LIBERO-PRO/libero/libero/",
                   "LIBERO-PRO: its envs, assets, BDDL and init files")


TREE_B1K_OMNIGIBSON = ("capx/third_party/b1k/OmniGibson/omnigibson/",
                       "OmniGibson, BEHAVIOR's simulator: its code, configs, kit file and materials")
TREE_B1K_BDDL = ("capx/third_party/b1k/bddl3/bddl/", "bddl3, BEHAVIOR's task-definition package")

B1K_LOGO = "capx/third_party/b1k/docs/assets/OmniGibson_logo.png"






@dataclass(frozen=True)
class FamilySet:
    """What a family's process may load: modules are required, optional ones allowed."""

    modules: frozenset[str]
    attempts: frozenset[str]
    optional: frozenset[str] = frozenset()
    files: frozenset[str] = frozenset()
    env_config_dir: str | None = None
    trees: tuple[tuple[str, str], ...] = ()

    def allowed(self) -> frozenset[str]:
        return self.modules | self.attempts | self.optional


_ROBOSUITE = CORE + SIM_FRANKA_REAL + SIM_ROBOSUITE + UTILS_SIM + UTILS_CHAIN + INTEGRATIONS
_LIBERO = (CORE + SIM_FRANKA_REAL + SIM_LIBERO + CONFIGS_INSTANTIATE + UTILS_SIM + UTILS_CHAIN
           + INTEGRATIONS)

_BEHAVIOR = CORE + SIM_FRANKA_REAL + SIM_BEHAVIOR + UTILS_SIM


def _robosuite(files: tuple[str, ...]) -> FamilySet:
    return FamilySet(frozenset(_ROBOSUITE), frozenset(ATTEMPT_ROBOSUITE), files=frozenset(files),
                     trees=(TREE_ROBOSUITE,))



FAMILY_SETS: dict[str, FamilySet] = {
    "libero": FamilySet(frozenset(_LIBERO), frozenset(ATTEMPT_LIBERO),
                        optional=frozenset(CONFIGS_LOADER), env_config_dir="env_configs/libero/",
                        trees=(TREE_LIBERO_ROBOSUITE, TREE_LIBERO_PRO)),
    "robosuite": _robosuite((CONTROLLER,)),
    "robosuite_two_arm": _robosuite((CONTROLLER, CONTROLLER_2)),
    "robosuite_wipe": _robosuite((CONTROLLER,)),
    "behavior": FamilySet(frozenset(_BEHAVIOR), frozenset(ATTEMPT_BEHAVIOR),
                          files=frozenset((B1K_LOGO,)), trees=(TREE_B1K_OMNIGIBSON, TREE_B1K_BDDL)),
}
