set dest=../../../../OpenNI_openFrameworks
set build_type=Debug
set lib_src_dir=../Bin/%build_type%/
set dest_base=%dest%/ofxOpenNI/win/copy_to_data_path/openni/
set dest_lib=%dest_base%lib
set dest_config=%dest_base%config

:: Create destination + copy files
mkdir "%dest%"
mkdir --parents "%dest_lib%"
mkdir --parents "%dest_config%"
cp %lib_src_dir%*.dll %dest_lib%
cp ../Lib/%build_type%/*.lib %dest_lib%
cp ../../../Data/GlobalDefaults.ini %dest_config%
