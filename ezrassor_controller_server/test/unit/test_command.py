"""Tests for the command conversion code in this module."""
import unittest
import ezrassor_controller_server as server

class unitTest(unittest.TestCase):

    def test_create_command_with_an_empty_request(self):
        """Should create a command that does nothing."""
        command = server.create_command({})

        assert command.joint_1_action is None
        assert command.joint_2_action is None
        assert command.joint_3_action is None
        assert command.joint_4_action is None
        assert command.joint_5_action is None
        assert command.claw_action is None
        assert command.partial_autonomy is None


    def test_create_command_with_joint_one_action_in_request(self):
        """Should create a command that starts joint one action."""
        command = server.create_command(
            {server.JOINT_1_ACTION_KEY: server.AllJointAction.ROTATELEFT.name}
        )

        assert command.joint_1_action is server.AllJointAction.ROTATELEFT
        assert command.joint_2_action is None
        assert command.joint_3_action is None
        assert command.joint_4_action is None
        assert command.joint_5_action is None
        assert command.claw_action is None
        assert command.partial_autonomy is None


    def test_create_command_with_joint_two_action_in_request(self):

        """Should create a command that starts joint two action."""
        command = server.create_command(
            {server.JOINT_2_ACTION_KEY: server.AllJointAction.ROTATELEFT.name}
        )

        assert command.joint_1_action is None
        assert command.joint_2_action is server.AllJointAction.ROTATELEFT
        assert command.joint_3_action is None
        assert command.joint_4_action is None
        assert command.joint_5_action is None
        assert command.claw_action is None
        assert command.partial_autonomy is None

    def test_create_command_with_joint_three_action_in_request(self):

        """Should create a command that starts joint three action."""
        command = server.create_command(
            {server.JOINT_3_ACTION_KEY: server.AllJointAction.ROTATELEFT.name}
        )

        assert command.joint_1_action is None
        assert command.joint_2_action is None
        assert command.joint_3_action is server.AllJointAction.ROTATELEFT
        assert command.joint_4_action is None
        assert command.joint_5_action is None
        assert command.claw_action is None
        assert command.partial_autonomy is None

    def test_create_command_with_joint_four_action_in_request(self):

        """Should create a command that starts joint four action."""
        command = server.create_command(
            {server.JOINT_4_ACTION_KEY: server.AllJointAction.ROTATELEFT.name}
        )

        assert command.joint_1_action is None
        assert command.joint_2_action is None
        assert command.joint_3_action is None
        assert command.joint_4_action is server.AllJointAction.ROTATELEFT
        assert command.joint_5_action is None
        assert command.claw_action is None
        assert command.partial_autonomy is None

    def test_create_command_with_joint_five_action_in_request(self):

        """Should create a command that starts joint five action."""
        command = server.create_command(
            {server.JOINT_5_ACTION_KEY: server.AllJointAction.ROTATELEFT.name}
        )

        assert command.joint_1_action is None
        assert command.joint_2_action is None
        assert command.joint_3_action is None
        assert command.joint_4_action is None
        assert command.joint_5_action is server.AllJointAction.ROTATELEFT
        assert command.claw_action is None
        assert command.partial_autonomy is None

    def test_create_command_with_claw_action_in_request(self):

        """Should create a command that starts a claw action."""
        command = server.create_command(
            {server.CLAW_ACTION_KEY: server.ClawAction.OPEN.name}
        )

        assert command.joint_1_action is None
        assert command.joint_2_action is None
        assert command.joint_3_action is None
        assert command.joint_4_action is None
        assert command.joint_5_action is None
        assert command.claw_action is server.ClawAction.OPEN
        assert command.partial_autonomy is None

    def test_create_command_with_autonomy_action_in_request(self):

        """Should create a command that starts a claw action."""
        command = server.create_command(
            {server.PARTIAL_AUTONOMY_KEY: server.PartialAutonomy.HOME.name}
        )

        assert command.joint_1_action is None
        assert command.joint_2_action is None
        assert command.joint_3_action is None
        assert command.joint_4_action is None
        assert command.joint_5_action is None
        assert command.claw_action is None
        assert command.partial_autonomy is server.PartialAutonomy.HOME

    def test_create_command_with_serveral_actions_in_request(self):
        """Should create a command that instructs the EZRASSOR to do many things.

        This test ensures that complex requests are fully translated.
        """
        command = server.create_command(
            {
                server.JOINT_1_ACTION_KEY: server.AllJointAction.ROTATELEFT.name,
                server.JOINT_2_ACTION_KEY: server.AllJointAction.ROTATERIGHT.name,
                server.JOINT_3_ACTION_KEY: server.AllJointAction.ROTATEUP.name,
                server.JOINT_4_ACTION_KEY: server.AllJointAction.ROTATEDOWN.name,
                server.JOINT_5_ACTION_KEY: server.AllJointAction.ROTATELEFT.name,
                server.CLAW_ACTION_KEY: server.ClawAction.CLOSE.name,
                server.PARTIAL_AUTONOMY_KEY: server.PartialAutonomy.PLACE.name
            },
        )

        assert command.joint_1_action is server.AllJointAction.ROTATELEFT
        assert command.joint_2_action is server.AllJointAction.ROTATERIGHT
        assert command.joint_3_action is server.AllJointAction.ROTATEUP
        assert command.joint_4_action is server.AllJointAction.ROTATEDOWN
        assert command.joint_5_action is server.AllJointAction.ROTATELEFT
        assert command.claw_action is server.ClawAction.CLOSE
        assert command.partial_autonomy is server.PartialAutonomy.PLACE

if __name__ == '__main__':
    unittest.main()