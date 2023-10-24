There are 6 sets of functions in this folder:

-Goal generation functions (These functions may be used as base to generate new test sets)
	->generate_gaussian_saccades.m (generates saccade goals in a gaussian distribution)
-------------------------------------------------------------------------------------------------------------
	->generate_MS_matrix.m (generates pure horizontal saccade goals, increasing in amplitude)
-------------------------------------------------------------------------------------------------------------
	->GenerateIncrObliqueSaccades.m (generates oblique saccade goals, increasing in amplitude)

-Simulator + linearization + control + plotting results + auxiliary functions
	->affine_fit.m (gets a plane fit based on the input data)
-------------------------------------------------------------------------------------------------------------
	->compute_eye_torques2.m (computes eye elastic and damping torque)
-------------------------------------------------------------------------------------------------------------
	->get_jacobian.m (linear approximation of the nonlinear simulator dynamics)
-------------------------------------------------------------------------------------------------------------
	->get_STD.m (computes standard deviation for a set of saccades, used for the paper)
-------------------------------------------------------------------------------------------------------------
	->optimal_control_with_u_acc.m (optimal control algorithm)
-------------------------------------------------------------------------------------------------------------
	->plotlinearsimulator.m (FIRST FUNCTION TO RUN IN ORDER TO GET SIMULATOR AND CONTROL RESULTS)
-------------------------------------------------------------------------------------------------------------
	->PlotVelocityScaling.m (Fix vertical component and increase horizontal goal. Used to check velocity profile)
-------------------------------------------------------------------------------------------------------------
	->rotation_to_vee.m (transform a rotation matrix to an angular velocity vector)
-------------------------------------------------------------------------------------------------------------
	->run_optimal_control.m (calls linearization, applies the discretization of the dynamic system and runs the control)
-------------------------------------------------------------------------------------------------------------
	->runlinearmodel.m (initialization of variables before calling run_optimal_control.m)
-------------------------------------------------------------------------------------------------------------
	->script_eye_head.m (simulator, represents eye dynamics)
-------------------------------------------------------------------------------------------------------------
	->ShowPlanes/ShowPlanes1/ShowPlanesThoroughly.m (plotting function for Listing's plane (LP) and velocity 
	   profiles... each has a different objective. ShowPlanes1 is for pre-tension tests, ShowPlanesThoroughly
	   shows, in the LP, goal orientation, reached orientation, and actual orientation at optimum saccade duration)
-------------------------------------------------------------------------------------------------------------
	->skew.m (used to transform an angular velocity vector into a rotation matrix)
-------------------------------------------------------------------------------------------------------------
	->visualization.m (communicates with ROS to use a visual simulator in RVIZ (using_markers package))
-------------------------------------------------------------------------------------------------------------
	->xline.m (plots a line representing perfect LP)

-Lookup Table (to check equilibrium orientations throughout oculomotor range) + plotting equilibrium orientations
	->lookup_table.m (maps motor commands to equilibrium rotations in the oculomotor range)
-------------------------------------------------------------------------------------------------------------
	->plot_eq_rotations.m (scatter plot of all equilibrium rotations in the oculomotor range)
-------------------------------------------------------------------------------------------------------------
	->simulator_get_equ_points.m (given a high nunmber of random motor commands, finds sets that lead to equilibrium orientations)

-Pre-tension (initial thetas) optimization 
	->minimization.m (minimize initial thetas (pre-tension) using a nonlinear optimizer, fmincon)
-------------------------------------------------------------------------------------------------------------
	->get_theta1.m (equality (final torque has to be zero) and non equal (force bigger than a threshold) constraints)
-------------------------------------------------------------------------------------------------------------
	->cost.m (function used to minimize initial set of thetas)

-Force testing according to Hepp, 1985 (Iso-frequency curves of oculomotor neurons in the rhesus monkey)
	->CheckForces.m (based on fixation thetas, computes the final force/torque for each muscle, given the constraints stated in TestForces.m)
-------------------------------------------------------------------------------------------------------------
	->GenerateGridHepp85.m (generate a goal set based on what Hepp showed in 1985)
-------------------------------------------------------------------------------------------------------------
	->TestForces.m (define the parameters to check the force applied by the muscles got from CheckForces.m)
			->WA = 1 -> test done on wrap around results
			->LT = 1 -> test done on lookup table results, if LT=0, it uses results taken from linear control
			->grid = 1 -> get only singular force values for force for a set of specific goals
			->tau = 1 -> to check elastic torque instead of force

-Functions that are not used/used only for very specific scenarios (can be helpful for future work)
	->compute_tau_head.m (computes torques for the head)
-------------------------------------------------------------------------------------------------------------
	->compute_taus_WA.m (computes torques using the WA muscle model)
-------------------------------------------------------------------------------------------------------------
	->ForceApproximation.m (approximates forces using quadratic fit)