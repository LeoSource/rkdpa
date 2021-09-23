clear
close all
clc

addpath('classes');
addpath('tools');

%% test mqtt communication between matlab and another device
address = 'tcp://192.168.100.11';
topic = 'jw/robot_arm/lzx_test';
comm = MQTTComm(address, topic);
% my_mqtt = mqtt('tcp://192.168.100.11', 'Port', 1883);
% my_sub = subscribe(my_mqtt, 'jw/robot_arm/lzx_test', 'Callback', 'show_message', 'Timeout', 50);
% publish(my_mqtt, 'jw/robot_arm/lzx_test', 'hello world!');

while 1
    pause(3);
    disp(comm.jdata);
end

