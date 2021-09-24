clear
close all
clc

addpath('classes');
addpath('tools');

%% test mqtt communication between matlab and another device
address = 'tcp://192.168.100.11';
topic = 'jw/robot_arm/data_send';
comm = MQTTComm(address, zeros(6,1), topic);
% my_mqtt = mqtt('tcp://192.168.100.11', 'Port', 1883);
% my_sub = subscribe(my_mqtt, 'jw/robot_arm/lzx_test', 'Callback', 'show_message', 'Timeout', 50);
% publish(my_mqtt, 'jw/robot_arm/lzx_test', 'hello world!');
% pub_msg = struct('opt', 'open');
% comm.PublishData('jw/robot_arm/claw', pub_msg);

while 1
    pause(3);
    jpos = comm.ReadData();
    disp(jpos);
end

