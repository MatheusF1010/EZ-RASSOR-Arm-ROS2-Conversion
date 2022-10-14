"""Test request verification in this module."""
import ezrassor_controller_server as server
import unittest

class unitTest(unittest.TestCase):
        
    def test_verify_with_empty_request(self):
        """Should do nothing because the an empty request is valid."""
        server.verify({})


    def test_verify_with_valid_request(self):
        """Should do nothing because the request is valid."""
        server.verify(
            {
                server.JOINT_1_ACTION_KEY: server.AllJointAction.ROTATELEFT.name,
                server.JOINT_1_ACTION_KEY: server.AllJointAction.ROTATERIGHT.name,
                server.JOINT_1_ACTION_KEY: server.AllJointAction.ROTATEUP.name,
                server.JOINT_1_ACTION_KEY: server.AllJointAction.ROTATEDOWN.name,
                
                server.JOINT_2_ACTION_KEY: server.AllJointAction.ROTATELEFT.name,
                server.JOINT_2_ACTION_KEY: server.AllJointAction.ROTATERIGHT.name,
                server.JOINT_2_ACTION_KEY: server.AllJointAction.ROTATEUP.name,
                server.JOINT_2_ACTION_KEY: server.AllJointAction.ROTATEDOWN.name,

                server.JOINT_3_ACTION_KEY: server.AllJointAction.ROTATELEFT.name,
                server.JOINT_3_ACTION_KEY: server.AllJointAction.ROTATERIGHT.name,
                server.JOINT_3_ACTION_KEY: server.AllJointAction.ROTATEUP.name,
                server.JOINT_3_ACTION_KEY: server.AllJointAction.ROTATEDOWN.name,

                server.JOINT_4_ACTION_KEY: server.AllJointAction.ROTATELEFT.name,
                server.JOINT_4_ACTION_KEY: server.AllJointAction.ROTATERIGHT.name,
                server.JOINT_4_ACTION_KEY: server.AllJointAction.ROTATEUP.name,
                server.JOINT_4_ACTION_KEY: server.AllJointAction.ROTATEDOWN.name,

                server.JOINT_5_ACTION_KEY: server.AllJointAction.ROTATELEFT.name,
                server.JOINT_5_ACTION_KEY: server.AllJointAction.ROTATERIGHT.name,
                server.JOINT_5_ACTION_KEY: server.AllJointAction.ROTATEUP.name,
                server.JOINT_5_ACTION_KEY: server.AllJointAction.ROTATEDOWN.name,
                
                server.CLAW_ACTION_KEY: server.ClawAction.OPEN.name,
                server.CLAW_ACTION_KEY: server.ClawAction.CLOSE.name,
                
                server.PARTIAL_AUTONOMY_KEY: server.PartialAutonomy.PLACE.name,
                server.PARTIAL_AUTONOMY_KEY: server.PartialAutonomy.PICKUP.name,
                server.PARTIAL_AUTONOMY_KEY: server.PartialAutonomy.HOME.name,
            },
        )

    def test_verify_with_unsupported_joint_one_action_in_request(self):
        """Should fail because the joint one action is not supported."""
        try:
            server.verify({server.JOINT_1_ACTION_KEY: "INVALID"})
            assert False, "verify() should have failed"
        except Exception as error:
            assert isinstance(error, server.VerificationError)
            assert "value must be one of" in error.message

    def test_verify_with_unsupported_joint_two_action_in_request(self):
        """Should fail because the joint two action is not supported."""
        try:
            server.verify({server.JOINT_2_ACTION_KEY: "INVALID"})
            assert False, "verify() should have failed"
        except Exception as error:
            assert isinstance(error, server.VerificationError)
            assert "value must be one of" in error.message

    def test_verify_with_unsupported_joint_three_action_in_request(self):
        """Should fail because the joint three action is not supported."""
        try:
            server.verify({server.JOINT_3_ACTION_KEY: "INVALID"})
            assert False, "verify() should have failed"
        except Exception as error:
            assert isinstance(error, server.VerificationError)
            assert "value must be one of" in error.message

    def test_verify_with_unsupported_joint_four_action_in_request(self):
        """Should fail because the joint four action is not supported."""
        try:
            server.verify({server.JOINT_4_ACTION_KEY: "INVALID"})
            assert False, "verify() should have failed"
        except Exception as error:
            assert isinstance(error, server.VerificationError)
            assert "value must be one of" in error.message

    def test_verify_with_unsupported_joint_five_action_in_request(self):
        """Should fail because the joint five action is not supported."""
        try:
            server.verify({server.JOINT_5_ACTION_KEY: "INVALID"})
            assert False, "verify() should have failed"
        except Exception as error:
            assert isinstance(error, server.VerificationError)
            assert "value must be one of" in error.message

    def test_verify_with_unsupported_claw_action_in_request(self):
        """Should fail because the claw action is not supported."""
        try:
            server.verify({server.CLAW_ACTION_KEY: "INVALID"})
            assert False, "verify() should have failed"
        except Exception as error:
            assert isinstance(error, server.VerificationError)
            assert "value must be one of" in error.message

    def test_verify_with_unsupported_partial_autonomy__action_in_request(self):
        """Should fail because the autonomy action is not supported."""
        try:
            server.verify({server.PARTIAL_AUTONOMY_KEY: "INVALID"})
            assert False, "verify() should have failed"
        except Exception as error:
            assert isinstance(error, server.VerificationError)
            assert "value must be one of" in error.message


    # def test_verify_with_wheel_action_without_subkeys_in_request():
    #     """Should fail because the wheel action is missing required subkeys."""
    #     try:
    #         server.verify({server.WHEEL_ACTION_KEY: "invalid"})
    #         assert False, "verify() should have failed"
    #     except Exception as error:
    #         assert isinstance(error, server.VerificationError)
    #         assert f"missing {server.LINEAR_X_KEY}" in error.message


    # def test_verify_with_missing_linear_x_key_in_request():
    #     """Should fail because the wheel action has no linear_x."""
    #     try:
    #         server.verify(
    #             {server.WHEEL_ACTION_KEY: {server.ANGULAR_Z_KEY: 0.0}},
    #         )
    #         assert False, "verify() should have failed"
    #     except Exception as error:
    #         assert isinstance(error, server.VerificationError)
    #         assert f"missing {server.LINEAR_X_KEY}" in error.message


    # def test_verify_with_missing_angular_z_key_in_request():
    #     """Should fail because the wheel action has no angular_z."""
    #     try:
    #         server.verify(
    #             {server.WHEEL_ACTION_KEY: {server.LINEAR_X_KEY: 0.0}},
    #         )
    #         assert False, "verify() should have failed"
    #     except Exception as error:
    #         assert isinstance(error, server.VerificationError)
    #         assert f"missing {server.ANGULAR_Z_KEY}" in error.message


    # def test_verify_with_invalid_linear_x_type_in_request():
    #     """Should fail because the wheel action's linear_x is the wrong type."""
    #     try:
    #         server.verify(
    #             {
    #                 server.WHEEL_ACTION_KEY: {
    #                     server.LINEAR_X_KEY: "invalid",
    #                     server.ANGULAR_Z_KEY: 0.0,
    #                 },
    #             },
    #         )
    #         assert False, "verify() should have failed"
    #     except Exception as error:
    #         assert isinstance(error, server.VerificationError)
    #         assert "must be a float" in error.message


    # def test_verify_with_invalid_angular_z_type_in_request():
    #     """Should fail because the wheel action's angular_z is the wrong type."""
    #     try:
    #         server.verify(
    #             {
    #                 server.WHEEL_ACTION_KEY: {
    #                     server.LINEAR_X_KEY: 0.0,
    #                     server.ANGULAR_Z_KEY: "invalid",
    #                 },
    #             },
    #         )
    #         assert False, "verify() should have failed"
    #     except Exception as error:
    #         assert isinstance(error, server.VerificationError)
    #         assert "must be a float" in error.message


    # def test_verify_with_unsupported_front_arm_action_in_request():
    #     """Should fail because the front arm action is not supported."""
    #     try:
    #         server.verify({server.FRONT_ARM_ACTION_KEY: "INVALID"})
    #         assert False, "verify() should have failed"
    #     except Exception as error:
    #         assert isinstance(error, server.VerificationError)
    #         assert "value must be one of" in error.message


    # def test_verify_with_unsupported_back_arm_action_in_request():
    #     """Should fail because the back arm action is not supported."""
    #     try:
    #         server.verify({server.BACK_ARM_ACTION_KEY: "INVALID"})
    #         assert False, "verify() should have failed"
    #     except Exception as error:
    #         assert isinstance(error, server.VerificationError)
    #         assert "value must be one of" in error.message


    # def test_verify_with_unsupported_front_drum_action_in_request():
    #     """Should fail because the front drum action is not supported."""
    #     try:
    #         server.verify({server.FRONT_DRUM_ACTION_KEY: "INVALID"})
    #         assert False, "verify() should have failed"
    #     except Exception as error:
    #         assert isinstance(error, server.VerificationError)
    #         assert "value must be one of" in error.message


    # def test_verify_with_unsupported_back_drum_action_in_request():
    #     """Should fail because the back drum action is not supported."""
    #     try:
    #         server.verify({server.BACK_DRUM_ACTION_KEY: "INVALID"})
    #         assert False, "verify() should have failed"
    #     except Exception as error:
    #         assert isinstance(error, server.VerificationError)
    #         assert "value must be one of" in error.message


    # def test_verify_with_unsupported_routine_action_in_request():
    #     """Should fail because the routine action is not supported."""
    #     try:
    #         server.verify({server.ROUTINE_ACTION_KEY: "INVALID"})
    #         assert False, "verify() should have failed"
    #     except Exception as error:
    #         assert isinstance(error, server.VerificationError)
    #         assert "value must be one of" in error.message


    def test_verify_with_unknown_key_in_request(self):
        """Should fail because the request contains an unknown key."""
        try:
            server.verify({"invalid": "invalid"})
            assert False, "verify() should have failed"
        except Exception as error:
            assert isinstance(error, server.VerificationError)
            assert "unknown key" in error.message


if __name__ == '__main__':
    unittest.main()