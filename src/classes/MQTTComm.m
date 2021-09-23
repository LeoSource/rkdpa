classdef MQTTComm < handle

properties
    jdata

    my_mqtt
    my_sub
end

methods
    function obj = MQTTComm(address,topic,init_value)
        obj.my_mqtt = mqtt(address, 'Port', 1883);
        obj.my_sub = subscribe(obj.my_mqtt, topic, 'Callback', @obj.ConvertMQTTData);
        if nargin>2
            obj.jdata = init_value;
        else
%             obj.jdata = zeros(6,1);
        end
    end

    function ConvertMQTTData(obj, topic, msg)
        obj.jdata = str2num(msg);
%         disp(topic);
%         disp(msg);
    end

end

end

