function out_data = read_mqtt(address, topic)

persistent comm
if isempty(comm)
    comm = MQTTComm(address, topic, zeros(6,1));
    out_data = zeros(6,1);
else
    out_data = comm.jdata;
end


end