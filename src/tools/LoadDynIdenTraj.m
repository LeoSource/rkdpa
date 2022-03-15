function [jp,jv,ja,jt] = LoadDynIdenTraj(filename,fs,iden)
    test_data = load(filename);
    jp_tmp = test_data(:,1:6)';
    jv_tmp = test_data(:,7:12)';
    jt_tmp = test_data(:,13:18)';

    jdprocessor = JointDataProcessor(fs,10,10,false,false);
    for idx=1:size(jv_tmp,2)
        [jp(:,idx),jv(:,idx),ja(:,idx),jt(:,idx)] = jdprocessor.ProcessData...
                                                            (jp_tmp(:,idx),jv_tmp(:,idx),jt_tmp(:,idx));
    end
    nj = 6;
    if iden
        N = 5; fc_vel = 30; fc_acc = 10; fc_tau = 10;
        [Bvel,Avel] = butter(N,fc_vel/(fs/2),'low');
        [Bacc,Aacc] = butter(N,fc_acc/(fs/2),'low');
        [Btau,Atau] = butter(N,fc_tau/(fs/2),'low');
        for idx=1:nj
            jv(idx,:) = filtfilt(Bvel, Avel, jv(idx,:));
            ja(idx,:) = filtfilt(Bacc, Aacc,ja(idx,:));
            jt(idx,:) = filtfilt(Btau, Atau, jt(idx,:));
        end

        discard_num = 50;
        jp = jp(:,discard_num:end-discard_num);
        jv = jv(:,discard_num:end-discard_num);
        ja = ja(:,discard_num:end-discard_num);
        jt = jt(:,discard_num:end-discard_num);
    else
        jdprocessor = JointDataProcessor(fs,2,10,true,false);
        for idx=1:size(jv_tmp,2)
            [jp(:,idx),jv(:,idx),ja(:,idx),jt(:,idx)] = jdprocessor.ProcessData...
                                                                (jp_tmp(:,idx),jv_tmp(:,idx),jt_tmp(:,idx));
        end
    end
        

end