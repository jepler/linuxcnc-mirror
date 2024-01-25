# net constraints-ok <= match_all.out => motion.enable

net resetall minmax.0.reset minmax.1.reset minmax.2.reset minmax.3.reset minmax.4.reset minmax.5.reset minmax.6.reset minmax.7.reset minmax.8.reset minmax.9.reset minmax.10.reset minmax.11.reset minmax.12.reset minmax.13.reset minmax.14.reset minmax.15.reset minmax.16.reset minmax.17.reset minmax.18.reset minmax.19.reset => pyvcp.resetmm

loadrt time
loadrt not
addf time.0 servo-thread
addf not.0 servo-thread
net prog-running not.0.in <= halui.program.is-idle
net cycle-timer time.0.start <= not.0.out
net cycle-seconds pyvcp.time-seconds <= time.0.seconds
net cycle-minutes pyvcp.time-minutes <= time.0.minutes
net cycle-hours pyvcp.time-hours <= time.0.hours

