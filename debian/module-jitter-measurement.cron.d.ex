#
# Regular cron jobs for the robotkernel-module-posix-timer package
#
0 4	* * *	root	[ -x /usr/bin/robotkernel-module-posix-timer_maintenance ] && /usr/bin/robotkernel-module-posix-timer_maintenance
