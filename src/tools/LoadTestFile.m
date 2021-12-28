function [jpos,jvel,jtau,t] = LoadTestFile(file_name,dt)
    nj = 6;
    td = load(file_name);
    jpos_idx = 1; jvel_idx = 2; jtor_idx = 3;
    jpos = td(:,1:jpos_idx*nj);
    jvel = td(:,nj+1:jvel_idx*nj);
    jtau = td(:,2*nj+1:jtor_idx*nj);
    t = 0:dt:dt*(size(td,1)-1);
end
