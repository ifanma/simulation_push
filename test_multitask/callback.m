function callback(obj, event, id)

    file = fopen(['mytext_' num2str(id) '.txt'], 'a+');
    try
        tic
        load("data.mat", "aaa");
        t = toc;
        disp(num2str(id));
        fprintf(file,'%s .. %f\r\n', datestr(now,'MM.SS.FFF'), t); 
    catch ME
        fprintf(file,'%s \r\n', ME.message);
    end
    fclose(file);
    aaa = aaa +1;
    save("data.mat", "aaa")
    pause(0.1)
end

