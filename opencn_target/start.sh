#!/bin/bash

# This script is used to test the basic_pg_test program
# It will run the motor using the basic_pg and sample the important signals

/usr/local/bin/halcmd -f /root/basic_pg_test/basic_pg.hal
sleep 5

echo -e "[epos4] initialize pins/parameters"
/usr/local/bin/halcmd setp lcec.0.EPOS4.set-mode-csv 1
sleep 1


echo -e "[basic_pg] initialize pins/parameters"
/usr/local/bin/halcmd setp basic_pg.best_pg.control_mode 1
sleep 0.1
/usr/local/bin/halcmd setp basic_pg.best_pg.command_type 0
sleep 0.1
/usr/local/bin/halcmd setp basic_pg.best_pg.continus 1
sleep 0.1
# halcmd setp basic_pg.best_pg.joint0.max_speed 3000
# sleep 0.1
# halcmd setp basic_pg.best_pg.joint0.max_accel 100
# sleep 0.1
# halcmd setp basic_pg.best_pg.enable 1
# sleep 0.1
# halcmd sets cmd-in 1000
# sleep 0.1

# echo -e "[sampler] enable"
# halcmd setp sampler.memory.enable 1
# sleep 1

# echo -e "[basic_pg] start"
# halcmd sets start 1
# sleep 0.1

# echo -e "[basic_pg] Wait until done"
# i=0
# while true; do
 #   sleep 1
 #   echo -n "."
 #   actual_vel=$(halcmd gets actual-vel)
 #   if [ "$actual_vel" -ge "1000" ]; then
 #       echo -e "\n"
 #       echo -e "[basic_pg] movement completed"
 #       break
 #   fi

#    i=$((i+1))
#    if [ $i -eq 20 ]; then
#        break
#    fi
#done
#sleep 2

#echo -e "[sampler] disable"
#halcmd setp sampler.memory.enable 0
#sleep 0.1

# echo -e "[epos4] disable"
# halcmd setp lcec.0.EPOS4.set-mode-inactive 1
# sleep 1
