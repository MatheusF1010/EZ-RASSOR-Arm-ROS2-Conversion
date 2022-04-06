"""Request verification tools."""
import ezrassor_controller_server as server


WHEEL_ACTION_KEY = "wheel_action"
LINEAR_X_KEY = "linear_x"
ANGULAR_Z_KEY = "angular_z"
FRONT_ARM_ACTION_KEY = "front_arm_action"
BACK_ARM_ACTION_KEY = "back_arm_action"
FRONT_DRUM_ACTION_KEY = "front_drum_action"
BACK_DRUM_ACTION_KEY = "back_drum_action"
ROUTINE_ACTION_KEY = "routine_action"


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
        elif key == FRONT_ARM_ACTION_KEY or key == BACK_ARM_ACTION_KEY:
            if request[key] not in server.ArmAction:
                raise VerificationError(
                    f"{key} value must be one of [{str(server.ArmAction)}]",
                )
        elif key == FRONT_DRUM_ACTION_KEY or key == BACK_DRUM_ACTION_KEY:
            if request[key] not in server.DrumAction:
                raise VerificationError(
                    f"{key} value must be one of [{str(server.DrumAction)})",
                )
        elif key == ROUTINE_ACTION_KEY:
            if request[key] not in server.RoutineAction:
                raise VerificationError(
                    f"{key} value must be one of [{str(server.RoutineAction)}]",
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
