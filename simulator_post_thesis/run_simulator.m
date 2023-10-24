function [] = run_simulator()
prompt = {'Enter the dimensions you want to test the simulator with (real or prototype)'};
dlgtitle = 'Dimensions of the eye';
dims = [1 50];
definput = {'prototype'};
answer = inputdlg(prompt,dlgtitle,dims,definput);
simulator(answer{1});