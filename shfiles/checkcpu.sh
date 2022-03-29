cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor & sleep 1;
cat /proc/cpuinfo |grep MHz|uniq & sleep 1;
