rosrun robot_negotiation world_node &
sleep 1 && rosrun robot_negotiation task_generator.py &
sleep 2 && rosrun robot_negotiation environment_node &
sleep 3 && rosrun robot_negotiation csp_solver_node & 
sleep 4 && rosrun robot_negotiation test_node 


