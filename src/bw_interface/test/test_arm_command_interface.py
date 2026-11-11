from pathlib import Path


PKG = Path(__file__).resolve().parents[1]
MSG = PKG / "msg" / "ArmCommand.msg"
CMAKE = PKG / "CMakeLists.txt"
README = PKG / "README.md"


def test_arm_command_message_contract():
    text = MSG.read_text(encoding="utf-8")

    assert "uint8 COMMAND_JOINT=1" in text
    assert "uint8 COMMAND_POSE_ABS=2" in text
    assert "uint8 COMMAND_POSE_REL=3" in text
    assert "std_msgs/Header header" in text
    assert "uint8 command_type" in text
    assert "string source" in text
    assert "sensor_msgs/JointState joints" in text
    assert "bool left_valid" in text
    assert "bool right_valid" in text
    assert "geometry_msgs/Pose left_pose" in text
    assert "geometry_msgs/Pose right_pose" in text
    assert "float64[6] left_delta" in text
    assert "float64[6] right_delta" in text
    assert "string command_id" in text
    assert "bool motion_profile_valid" in text
    assert "bw_interface/MotionProfile motion_profile" in text


def test_arm_command_status_message_contract():
    status_msg = PKG / "msg" / "ArmCommandStatus.msg"
    text = status_msg.read_text(encoding="utf-8")

    assert "uint8 STATUS_RECEIVED=1" in text
    assert "uint8 STATUS_ROUTED=2" in text
    assert "uint8 STATUS_RESOLVED=3" in text
    assert "uint8 STATUS_FAILED=4" in text
    assert "uint8 STATUS_COMPLETED=5" in text
    assert "uint8 STATUS_TIMEOUT=6" in text
    assert "string command_id" in text
    assert "string source" in text
    assert "uint8 status" in text
    assert "string message" in text
    assert "sensor_msgs/JointState target" in text


def test_robot_command_message_contract():
    robot_msg = PKG / "msg" / "RobotCommand.msg"
    text = robot_msg.read_text(encoding="utf-8")

    assert "std_msgs/Header header" in text
    assert "string source" in text
    assert "bw_interface/ArmCommand arm" in text
    assert "bool head_valid" in text
    assert "sensor_msgs/JointState head" in text
    assert "bool gripper_valid" in text
    assert "sensor_msgs/JointState gripper" in text
    assert "bool cmd_vel_valid" in text
    assert "geometry_msgs/Twist cmd_vel" in text
    assert "bool pelvis_speed_valid" in text
    assert "float32 pelvis_speed" in text
    assert "bool waist_speed_valid" in text
    assert "float32 waist_speed" in text
    assert "bool pelvis_height_valid" in text
    assert "float32 pelvis_height" in text
    assert "bool pelvis_max_velocity_valid" in text
    assert "float32 pelvis_max_velocity" in text
    assert "bool waist_angle_valid" in text
    assert "float32 waist_angle" in text
    assert "bool waist_max_velocity_valid" in text
    assert "float32 waist_max_velocity" in text
    assert "bool joint_target_valid" in text
    assert "sensor_msgs/JointState joint_target" in text
    assert "bool motion_profile_valid" in text
    assert "bw_interface/MotionProfile motion_profile" in text


def test_arm_and_robot_commands_are_generated_and_documented():
    cmake = CMAKE.read_text(encoding="utf-8")
    assert '"msg/ArmCommand.msg"' in cmake
    assert '"msg/ArmCommandStatus.msg"' in cmake
    assert '"msg/RobotCommand.msg"' in cmake
    readme = README.read_text(encoding="utf-8")
    assert "ArmCommand.msg" in readme
    assert "ArmCommandStatus.msg" in readme
    assert "RobotCommand.msg" in readme
    assert "COMMAND_POSE_REL" in readme
