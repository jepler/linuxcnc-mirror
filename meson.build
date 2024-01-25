project('linuxcnc-unit-test','c')

src_root = 'src'
tp_src_dir = join_paths(src_root, 'emc/tp')

inc_dir = include_directories([
  'unit_tests',
  'unit_tests/mock',
  src_root,
  join_paths(src_root, 'emc/kinematics'),
  tp_src_dir,
  'src/libnml/posemath',
  'src/emc/nml_intf',
  'src/emc/motion',
  'src/rtapi',
  'src/hal',
  ])

cc = meson.get_compiler('c')
m_dep = cc.find_library('m', required : true)

add_global_arguments(['-DULAPI','-DUNIT_TEST'], language : 'c')

# Define static libraries for easier linking with unit tests

posemath_obj = static_library('posemath',
  'src/libnml/posemath/_posemath.c',
  include_directories : inc_dir,
  dependencies : [m_dep]
  )

emcpose_obj = static_library('emcpose',
  join_paths(src_root,   'emc/nml_intf/emcpose.c'),
  include_directories : inc_dir
  )

utlib_tp_deps = static_library('tp_deps',
  [
    join_paths(tp_src_dir, 'joint_util.c'), 
    join_paths(tp_src_dir, 'blendmath.c'), 
    join_paths(tp_src_dir, 'spherical_arc.c'), 
    join_paths(tp_src_dir, 'tc.c'), 
    join_paths(tp_src_dir, 'tcq.c'), 
  ],
  include_directories : inc_dir,
  link_with: [emcpose_obj, posemath_obj])

tp_test_files = [
  'test_blendmath',
  'test_joint_util',
  'test_spherical_arc',
  'test_posemath'
  ]

foreach n : tp_test_files
  
test(n, executable(n,
  join_paths('unit_tests/tp', n+'.c'),
  dependencies : [m_dep],
  include_directories : inc_dir,
  link_with : [ utlib_tp_deps]
  ))
endforeach

