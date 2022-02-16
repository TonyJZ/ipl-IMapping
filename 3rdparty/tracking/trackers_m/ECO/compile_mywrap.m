
% Compiles the ECO_init and ECO_update functions.
if ~isdeployed
    setup_paths();
end

mcc -W cpplib:ECO_init -T link:lib ECO_init.m -C -d D:\code_ipl\bin\vs2013\debug
mcc -W cpplib:ECO_update -T link:lib ECO_update.m -C -d D:\code_ipl\bin\vs2013\debug