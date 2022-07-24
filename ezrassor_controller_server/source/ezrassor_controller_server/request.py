"""Request verification tools."""
from tokenize import Double
import ezrassor_controller_server as server


WHEEL_ACTION_KEY = "wheel_action"
LINEAR_X_KEY = "linear_x"
ANGULAR_Z_KEY = "angular_z"
BACK_ARM_ACTION_KEY = "back_arm_action"
BACK_DRUM_ACTION_KEY = "back_drum_action"
ROUTINE_ACTION_KEY = "routine_action"

JOINT_1_ACTION_KEY = "joint_1_action"
JOINT_2_ACTION_KEY = "joint_2_action"
JOINT_3_ACTION_KEY = "joint_3_action"
JOINT_4_ACTION_KEY = "joint_4_action"
JOINT_5_ACTION_KEY = "joint_5_action"
CLAW_ACTION_KEY = "claw_action"
PARTIAL_AUTONOMY_KEY = "partial_autonomy"

def verify(request):
    """Validate the contents of the request."""
    for key in request.keys():
        if key == WHEEL_ACTION_KEY:
            if not isinstance(request[key], dict):
                raise VerificationError(
                    f"missing {LINEAR_X_KEY} and {ANGULAR_Z_KEY} for {key}",
                )
            if LINEAR_X_KEY not in request[key]:
                raise VerificationError(f"missing {LINEAR_X_KEY} for {key}")
            if ANGULAR_Z_KEY not in request[key]:
                raise VerificationError(f"missing {ANGULAR_Z_KEY} for {key}")
            if not isinstance(request[key][LINEAR_X_KEY], float):
                raise VerificationError(f"{LINEAR_X_KEY} must be a float")
            if not isinstance(request[key][ANGULAR_Z_KEY], float):
                raise VerificationError(f"{ANGULAR_Z_KEY} must be a float")
        elif key == BACK_ARM_ACTION_KEY:
            if request[key] not in server.ArmAction:
                raise VerificationError(
                    f"{key} value must be one of [{str(server.ArmAction)}]",
                )
        elif key == BACK_DRUM_ACTION_KEY:
            if request[key] not in server.DrumAction:
                raise VerificationError(
                    f"{key} value must be one of [{str(server.DrumAction)})",
                )
        elif key == ROUTINE_ACTION_KEY:
            if request[key] not in server.RoutineAction:
                raise VerificationError(
                    f"{key} value must be one of [{str(server.RoutineAction)}]",
                )
        elif key == JOINT_1_ACTION_KEY:
            if request[key] not in server.AllJointAction:
                raise VerificationError(
                    f"{key} value must be one of the [{str(server.AllJointAction)}]",
                )
        elif key == JOINT_2_ACTION_KEY:
            if request[key] not in server.AllJointAction:
                raise VerificationError(
                    f"{key} value must be one of the [{str(server.AllJointAction)}]",
                )
        elif key == JOINT_3_ACTION_KEY:
            if request[key] not in server.AllJointAction:
                raise VerificationError(
                    f"{key} value must be one of the [{str(server.AllJointAction)}]",
                )
        elif key == JOINT_4_ACTION_KEY:
            if request[key] not in server.AllJointAction:
                raise VerificationError(
                    f"{key} value must be one of the [{str(server.AllJointAction)}]",
                )
        elif key == JOINT_5_ACTION_KEY:
            if request[key] not in server.AllJointAction:
                raise VerificationError(
                    f"{key} value must be one of the [{str(server.AllJointAction)}]",
                )
        elif key == CLAW_ACTION_KEY:
            if request[key] not in server.ClawAction:
                raise VerificationError(
                    f"{key} value must be one of the [{str(server.ClawAction)}]",
                )
        elif key == PARTIAL_AUTONOMY_KEY:
            if request[key] not in server.PartialAutonomy:
                raise VerificationError(
                    f"{request[key]} value must be one of the [{str(server.PartialAutonomy)}]",
                )
        else:
            raise VerificationError(f"unknown key: {key}")


class VerificationError(Exception):
    """Encapsulate verification errors."""

    def __init__(self, message):
        """Initialize this error with a message."""
        self.message = message

    def ___str___(self):
        """Create a human-readable representation of this error."""
        return self.message
