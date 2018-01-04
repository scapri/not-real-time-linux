sudo sh -c "echo 1 > /sys/module/rcupdate/parameters/rcu_cpu_stall_suppress "
sudo sh -c 'echo -1 > /proc/sys/kernel/sched_rt_runtime_us'
