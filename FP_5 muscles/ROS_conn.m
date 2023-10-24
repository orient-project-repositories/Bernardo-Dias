function [] = ROS_conn(torque,force)


% rosinit
%rosinit('rosrpc://10.1.22.165:56299','NodeHost',ipaddress);
%rosinit("http://10.1.22.165:36279");
head_pub = rospublisher('/ehn/head_joint_effort_controller/command','std_msgs/Float64'); 
neck3_pub = rospublisher('/ehn/neck_joint3_effort_controller/command','std_msgs/Float64'); 
neck2_pub = rospublisher('/ehn/neck_joint2_effort_controller/command','std_msgs/Float64'); 
neck1_pub = rospublisher('/ehn/neck_joint1_effort_controller/command','std_msgs/Float64'); 
const_force_top_head = rospublisher('/ehn/force/dummy_link1','geometry_msgs/Wrench');
% const_force_head = rospublisher('/ehn/force/head_link','geometry_msgs/Wrench');
% const_force_neck1 = rospublisher('/ehn/force/neck_link1','geometry_msgs/Wrench');
% const_force_neck2= rospublisher('/ehn/force/neck_link2','geometry_msgs/Wrench');
% const_force_neck3 = rospublisher('/ehn/force/neck_link3','geometry_msgs/Wrench');





msg = rosmessage(head_pub);
msg1 = rosmessage(neck1_pub);
msg2 = rosmessage(neck2_pub);
msg3 = rosmessage(neck3_pub);
msg5 = rosmessage(const_force_top_head);
% msg6 = rosmessage(const_force_head);
% % msg7 = rosmessage(const_force_neck1);
% msg8 = rosmessage(const_force_neck2);
% % msg9 = rosmessage(const_force_neck3);




msg.Data = torque(1);
msg1.Data = torque(4);
msg2.Data = torque(3);
msg3.Data = torque(2);
msg5.Force.X = force(1);
msg5.Force.Y = force(2);
msg5.Force.Z = force(3);
% msg6.Torque.X = 0;
% msg6.Torque.Y = 0;
% msg6.Torque.Z = 0.4590;
% % msg7.Force.X = force(1);
% % msg7.Force.Y = force(2);
% % msg7.Force.Z = force(3);
% msg8.Torque.X = 0;
% msg8.Torque.Y = 0;
% msg8.Torque.Z = -0.1440;
% msg9.Force.X = force(1);
% msg9.Force.Y = force(2);
% msg9.Force.Z = force(3);

%Create timer object to define the rate of publishing
% t = timer;
% %Set a fixed rate-> 100 times per second
% set(t,'ExecutionMode','fixedRate');
% set(t,'Period',0.01);
% %Set the messages and publishers to send
% set(t,'TimerFcn',send(head_pub,msg));
% set(t,'TimerFcn',send(neck1_pub,msg3));
% set(t,'TimerFcn',send(neck2_pub,msg2));
% set(t,'TimerFcn',send(neck3_pub,msg1));
% set(t,'TimerFcn',send(const_force_top_head,msg5));
% %Start timer
% start(t);

send(head_pub,msg);
send(neck1_pub,msg3);
send(neck2_pub,msg2);
send(neck3_pub,msg1);
send(const_force_top_head,msg5);
% send(const_force_head,msg6);
% send(const_force_neck1,msg7);
% send(const_force_neck2,msg8);
% send(const_force_neck3,msg9);
msg5
msg
bp = 1;

%tftree = rostf;


