# Validation test for the urdf youbot model

rosrun xacro xacro.py `rospack find youbot_description`/robots/youbot_base.urdf.xacro -o ./youbot_base.urdf
rosrun urdf check_urdf ./youbot_base.urdf

rosrun xacro xacro.py `rospack find youbot_description`/robots/youbot_arm.urdf.xacro -o ./youbot_arm.urdf
rosrun urdf check_urdf ./youbot_arm.urdf

rosrun xacro xacro.py `rospack find youbot_description`/robots/youbot_gripper.urdf.xacro -o ./youbot_gripper.urdf
rosrun urdf check_urdf ./youbot_gripper.urdf

rosrun xacro xacro.py `rospack find youbot_description`/robots/youbot.urdf.xacro -o ./youbot.urdf
rosrun urdf check_urdf ./youbot.urdf
