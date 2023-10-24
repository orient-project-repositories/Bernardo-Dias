# Bernardo-Dias

Documents related to thesis can be found [here](https://data.isr.tecnico.ulisboa.pt/nextcloud/s/qiCSHo9zEfq5SZN?path=%2FMaster%20Thesis%2F5.%20Beranardo%20Dias)

simulator_post_thesis:
   - Simulator,linearization and control without wrap-around (WA).
   - To run, execute file plotlinearmodel.m

		- You can either run saccades starting from zero orientation (c==0), or starting from the 
		  last orientation (c==1), given that you provide a .mat file with the goals

		- You can also plot results directly from a .mat file (loadsacc_flag == 1), provided it has a 
		  specific structure: struct with statevec -> state evolution throughout the trajectory
						  tau_optimal -> optimal commands (delta) throughout the trajectory
						  saccade_ts -> saccade optimal duration
						  x_des -> saccade goal					

		- If you load 'pre-tension.mat' you can put pt_test = 1 and run a set of pre-define saccades with 
		  different pre-tensions (stated in runlinearmodel.m)

   - To change pre-tension or elasticity coefficient, do it in runlinearmodel.m
   - Linearization is in get_jacobian.m
   - Control algorithm is in optimal_control_with_u_acc.m
   - Simulator itself is in script_eye_head.m
   - Forces and torques are computed in compute_eye_torques2.m

FP_5 muscles:
   - Head neck model 5 insertion points optimization. Since we have 10 muscles, the other 5 are symmetric
   - Run get_insertion_points.m for the full optimization
   - Design.m shows a simplified figure of the muscle setup
   - setup_simscape.m makes a simscape object from the head neck model
   - there is a figure called "muscle_theoretical_setup.png" where you can see which cables correspond to each tendons
   - A more detailed analysis of this folder can be found in "DFA_report" 


ehn_description:
   - In a folder called "SolidWorks 2020" the SolidWorks files for the assembly of the model are present (full_model1.SLDASM)
      - If you desire to change the meshes use sw_urdf_exporter (http://wiki.ros.org/sw_urdf_exporter) or a similar add-in,
		   then change them in ehn_description/urdf/ehn.urdf
   - Head-Neck system modelling for Gazebo simulation. DO NOT USE pre-installed gazebo for ubuntu 18.04.
   - It is advised you go through ROS tutorial in ROS Wiki (http://wiki.ros.org/ROS/Tutorials)
   - in the devel folder, run "source setup.bash"
   - run "$roslaunch ehn_description main_empty.launch"
   - note that simulation doesn't start until you press the play button in Gazebo
   - I disabled gravity initially. To change this, change fake_ocean.world in "worlds" folder
   - For static analysis there is one file in ehn_description/src which is in development (send_forces.cpp)
   - The idea is to create a MATLAB script where you compute static forces and apply them in Gazebo through a topic (static_analysis) (you can check visualization.m to see how this is done)
   - I structured the message data (only on the receiving end/subscriber) as:
			-> data[0:3] - joint thetas (angle of rotation)
			-> data[4:33] - insertion points for each of the 10 muscles (their individual names are stated in the code)
			-> data[34:63] - forces applied on each insertion point.
   - when developing the MATLAB script, use the same structure when advertising to the "static_analysis" topic
   - Objective: at the same time we turn gravity on, we apply the forces given by MATLAB (using FP_5 muscles folder) and check if our force/torque propagation is correct.  


Don't forget that you can press the down arrow in "Current Folder" in MATLAB to get a Dependency Report and check
where every single function is called, for a faster analysis of the code.

Any questions you might have, please contact me at bernardo.colacodias@gmail.com
