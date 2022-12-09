function data_collection = get_data_directory(src_directory)
    experiments = dir(src_directory);
    data = {};
    for i = 1:length(experiments)
        if experiments(i).name == "." || experiments(i).name == ".."
            continue
        end
        test = strcat(experiments(i).folder, "/", experiments(i).name);
        files = dir(test);
        for k = 1:length(files)
            if files(k).name == "." || files(k).name == ".."
                continue
            end
            dat = strcat(test,"/", files(k).name);
            if contains(dat, ".dat")
                field = sprintf("test_%d", i);
                data{i} = readtable(dat, "VariableNamingRule", 'modify');
            end
        end
    end
    
    data_collection = {};
    place = 0;
    for tab_idx = 1:length(data)
        tab_size = size(data{tab_idx});
        if (tab_size(1) < 1)
            place = place+1;
            continue
        end
        t = data{tab_idx}.T__ms_/1000;
        y = 101.60-(data{tab_idx}.V_dist__mA_-4.0)*101.60/16.0-7.5;
        comp_bar = (data{tab_idx}.Comp_P__mA_-4)*0.75;
        f = 9.81*data{tab_idx}.LC__grams_/1000;
        piezo_bar = (data{tab_idx}.Piezo_P__mA_-4)*0.75;
        signal_bar = 6*(data{tab_idx}.Piezo_out__mV_-4000)/16000;
    
        shape = (size(data{tab_idx}));
        rows = shape(1);
        T = table(t,signal_bar,piezo_bar, y, f, comp_bar);
        data_collection{tab_idx-place} = T;
    end

end

