#!/bin/bash 

sec=2 
cnt=0 
name=auto_aim_HDUS
program_name=HDUS
cd /home/wpj/RM_Vision_code_US/auto_aim_HDUS/build/
#make clean && 
make -j12
while [ 1 ] 
do 
    count=`ps -ef | grep $program_name | grep -v "grep" | wc -l`
    echo "Thread count: $count" 
    echo "Expection count: $cnt" 
    if [ $count -ge 1 ]; then 
        echo "The $name is still alive!" 
        sleep $sec 
    else  
        echo "Starting $name..." 
        gnome-terminal -- bash -c "cd /home/wpj/RM_Vision_code_US/auto_aim_HDUS/build/;
        ./$program_name;exec bash;" 
        echo "$name has started!"   
        ((cnt=cnt+1)) 
        sleep $sec 
        if [ $cnt -gt 9 ]; then 
            echo "Reboot!" 
            #reboot 
        fi 
    fi 
done
