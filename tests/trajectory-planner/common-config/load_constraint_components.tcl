# Load all constraint-checking components used in HAL files for various sim configurations
# NOTE: add only the actively used ones to the RT thread in axis TCL files

loadrt d2dt2 count=10
# load additional blocks
loadrt hypot names=vel_xy,vel_xyz
addf vel_xy servo-thread
addf vel_xyz servo-thread

# Compares computed velocities / accelerations to limits
loadrt wcomp count=20

# Track the overall min / max value of each signal reached (for VCP display)
loadrt minmax count=20

# Ties all limit checks together (used to force a motion enable / disable)
loadrt match8 names=match_xyz,match_abc,match_uvw,match_all

net XYZ-limits-ok <= match_xyz.out => match_all.a0
net ABC-limits-ok <= match_abc.out => match_all.a1
net UVW-limits-ok <= match_uvw.out => match_all.a2

setp match_xyz.b0 0
setp match_xyz.b1 0
setp match_xyz.b2 0
setp match_xyz.b3 0
setp match_xyz.b4 0
setp match_xyz.b5 0
setp match_xyz.b6 0
setp match_xyz.b7 0

setp match_abc.b0 0
setp match_abc.b1 0
setp match_abc.b2 0
setp match_abc.b3 0
setp match_abc.b4 0
setp match_abc.b5 0
setp match_abc.b6 0
setp match_abc.b7 0

setp match_uvw.b0 0
setp match_uvw.b1 0
setp match_uvw.b2 0
setp match_uvw.b3 0
setp match_uvw.b4 0
setp match_uvw.b5 0
setp match_uvw.b6 0
setp match_uvw.b7 0

setp match_all.b0 1
setp match_all.b1 1
setp match_all.b2 1
setp match_all.b3 0
setp match_all.b4 0
setp match_all.b5 0
setp match_all.b6 0
setp match_all.b7 0
