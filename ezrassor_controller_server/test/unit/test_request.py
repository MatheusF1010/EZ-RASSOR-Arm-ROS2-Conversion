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
                server.JOINT_1_ACTION_KEY: server.AllJointAction.STOP.name,

                server.JOINT_2_ACTION_KEY: server.AllJointAction.ROTATELEFT.name,
                server.JOINT_2_ACTION_KEY: server.AllJointAction.ROTATERIGHT.name,
                server.JOINT_2_ACTION_KEY: server.AllJointAction.ROTATEUP.name,
                server.JOINT_2_ACTION_KEY: server.AllJointAction.ROTATEDOWN.name,
                server.JOINT_2_ACTION_KEY: server.AllJointAction.STOP.name,

                server.JOINT_3_ACTION_KEY: server.AllJointAction.ROTATELEFT.name,
                server.JOINT_3_ACTION_KEY: server.AllJointAction.ROTATERIGHT.name,
                server.JOINT_3_ACTION_KEY: server.AllJointAction.ROTATEUP.name,
                server.JOINT_3_ACTION_KEY: server.AllJointAction.ROTATEDOWN.name,
                server.JOINT_3_ACTION_KEY: server.AllJointAction.STOP.name,

                server.JOINT_4_ACTION_KEY: server.AllJointAction.ROTATELEFT.name,
                server.JOINT_4_ACTION_KEY: server.AllJointAction.ROTATERIGHT.name,
                server.JOINT_4_ACTION_KEY: server.AllJointAction.ROTATEUP.name,
                server.JOINT_4_ACTION_KEY: server.AllJointAction.ROTATEDOWN.name,
                server.JOINT_4_ACTION_KEY: server.AllJointAction.STOP.name,

                server.JOINT_5_ACTION_KEY: server.AllJointAction.ROTATELEFT.name,
                server.JOINT_5_ACTION_KEY: server.AllJointAction.ROTATERIGHT.name,
                server.JOINT_5_ACTION_KEY: server.AllJointAction.ROTATEUP.name,
                server.JOINT_5_ACTION_KEY: server.AllJointAction.ROTATEDOWN.name,
                server.JOINT_5_ACTION_KEY: server.AllJointAction.STOP.name,
                
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