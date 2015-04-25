rosrun robot_negotiation world_node &
sleep 1 && rosrun robot_negotiation task_generator.py &
sleep 2 && rosrun robot_negotiation cobot_node

