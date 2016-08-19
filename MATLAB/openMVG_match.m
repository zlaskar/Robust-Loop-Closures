function openMVG_match(folder)

%% Run script
cmd = ['sh ./script_cse-cn0011.sh ' folder];
system(cmd, '-echo');

end