#
# Copyright 2022 Yulun Zhuang
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from setuptools import find_packages
from sys import platform as _platform
import os

from distutils.core import setup
from distutils.extension import Extension
from distutils.util import get_platform

# monkey-patch for parallel compilation
import multiprocessing
import multiprocessing.pool


def parallelCCompile(self,
                     sources,
                     output_dir=None,
                     macros=None,
                     include_dirs=None,
                     debug=0,
                     extra_preargs=None,
                     extra_postargs=None,
                     depends=None):
  # those lines are copied from distutils.ccompiler.CCompiler directly
  macros, objects, extra_postargs, pp_opts, build = self._setup_compile(
      output_dir, macros, include_dirs, sources, depends, extra_postargs)
  cc_args = self._get_cc_args(pp_opts, debug, extra_preargs)
  # parallel code
  N = 2 * multiprocessing.cpu_count()  # number of parallel compilations
  try:
    # On Unix-like platforms attempt to obtain the total memory in the
    # machine and limit the number of parallel jobs to the number of Gbs
    # of RAM (to avoid killing smaller platforms like the Pi)
    mem = os.sysconf('SC_PHYS_PAGES') * os.sysconf('SC_PAGE_SIZE')  # bytes
  except (AttributeError, ValueError):
    # Couldn't query RAM; don't limit parallelism (it's probably a well
    # equipped Windows / Mac OS X box)
    pass
  else:
    mem = max(1, int(round(mem / 1024**3)))  # convert to Gb
    N = min(mem, N)

  def _single_compile(obj):
    try:
      src, ext = build[obj]
    except KeyError:
      return
    newcc_args = cc_args
    if _platform == "darwin":
      if src.endswith('.cpp') or src.endswith('.cc'):
        newcc_args = cc_args + [
            "-mmacosx-version-min=10.7", "-std=c++17", "-stdlib=libc++"
        ]
    self._compile(obj, src, ext, newcc_args, extra_postargs, pp_opts)

  # convert to list, imap is evaluated on-demand
  pool = multiprocessing.pool.ThreadPool(N)
  list(pool.imap(_single_compile, objects))
  return objects


import distutils.ccompiler

distutils.ccompiler.CCompiler.compile = parallelCCompile

# see http://stackoverflow.com/a/8719066/295157
import os

platform = get_platform()
print(platform)

CXX_FLAGS = '-D__SUPPRESSANYOUTPUT__'

# libraries += [current_python]

libraries = []
include_dirs = [
    '.',
    'extern',
    'extern/eigen3',
    'extern/osqp/include',
    'extern/osqp/lin_sys/direct/qdldl',
    'extern/osqp/lin_sys/direct/qdldl/qdldl_sources/include',
    'extern/osqp/lin_sys/direct/qdldl/amd/include',
    'extern/qpoases/include',
    'extern/pybind11/include',
]

try:
  import numpy

  NP_DIRS = [numpy.get_include()]
except:
  print("numpy is disabled. getCameraImage maybe slower.")
else:
  print("numpy is enabled.")
  CXX_FLAGS += '-DPYBULLET_USE_NUMPY '
  for d in NP_DIRS:
    print("numpy_include_dirs = %s" % d)
  include_dirs += NP_DIRS

sources = [
    "MPC_Controller/convex_MPC/mpc_osqp.cc",
    "extern/osqp/src/auxil.c",
    "extern/osqp/src/cs.c",
    "extern/osqp/src/ctrlc.c",
    "extern/osqp/src/error.c",
    "extern/osqp/src/kkt.c",
    "extern/osqp/src/lin_alg.c",
    "extern/osqp/src/lin_sys.c",
    "extern/osqp/src/osqp.c",
    "extern/osqp/src/polish.c",
    "extern/osqp/src/proj.c",
    "extern/osqp/src/scaling.c",
    "extern/osqp/src/util.c",
    "extern/osqp/lin_sys/direct/qdldl/qdldl_interface.c",
    "extern/osqp/lin_sys/direct/qdldl/qdldl_sources/src/qdldl.c",
    "extern/osqp/lin_sys/direct/qdldl/amd/src/amd_1.c",
    "extern/osqp/lin_sys/direct/qdldl/amd/src/amd_2.c",
    "extern/osqp/lin_sys/direct/qdldl/amd/src/amd_aat.c",
    "extern/osqp/lin_sys/direct/qdldl/amd/src/amd_control.c",
    "extern/osqp/lin_sys/direct/qdldl/amd/src/amd_defaults.c",
    "extern/osqp/lin_sys/direct/qdldl/amd/src/amd_info.c",
    "extern/osqp/lin_sys/direct/qdldl/amd/src/amd_order.c",
    "extern/osqp/lin_sys/direct/qdldl/amd/src/amd_post_tree.c",
    "extern/osqp/lin_sys/direct/qdldl/amd/src/amd_postorder.c",
    "extern/osqp/lin_sys/direct/qdldl/amd/src/amd_preprocess.c",
    "extern/osqp/lin_sys/direct/qdldl/amd/src/amd_valid.c",
    "extern/osqp/lin_sys/direct/qdldl/amd/src/SuiteSparse_config.c",
    "extern/qpoases/src/BLASReplacement.cpp",
    "extern/qpoases/src/Bounds.cpp",
    "extern/qpoases/src/Constraints.cpp",
    "extern/qpoases/src/Flipper.cpp",
    "extern/qpoases/src/Indexlist.cpp",
    "extern/qpoases/src/LAPACKReplacement.cpp",
    "extern/qpoases/src/Matrices.cpp",
    "extern/qpoases/src/MessageHandling.cpp",
    "extern/qpoases/src/Options.cpp",
    "extern/qpoases/src/OQPinterface.cpp",
    "extern/qpoases/src/QProblem.cpp",
    "extern/qpoases/src/QProblemB.cpp",
    "extern/qpoases/src/SolutionAnalysis.cpp",
    "extern/qpoases/src/SparseSolver.cpp",
    "extern/qpoases/src/SQProblem.cpp",
    "extern/qpoases/src/SQProblemSchur.cpp",
    "extern/qpoases/src/SubjectTo.cpp",
    "extern/qpoases/src/Utils.cpp",
]

if _platform == "linux" or _platform == "linux2":
  print("linux")
  include_dirs += ['extern/osqp/include/linux']
  CXX_FLAGS += '-fpermissive '
  libraries = ['dl', 'pthread']
  CXX_FLAGS += '-D_LINUX '
  CXX_FLAGS += '-DGLEW_STATIC '
  CXX_FLAGS += '-DGLEW_INIT_OPENGL11_FUNCTIONS=1 '
  CXX_FLAGS += '-DGLEW_DYNAMIC_LOAD_ALL_GLX_FUNCTIONS=1 '
  CXX_FLAGS += '-DDYNAMIC_LOAD_X11_FUNCTIONS '
  CXX_FLAGS += '-DHAS_SOCKLEN_T '
  CXX_FLAGS += '-fno-inline-functions-called-once '
  CXX_FLAGS += '-fvisibility=hidden '
  CXX_FLAGS += '-fvisibility-inlines-hidden '
  CXX_FLAGS += '-std=c++1z '
  CXX_FLAGS += '-Wno-sign-compare '
  CXX_FLAGS += '-Wno-reorder '
  CXX_FLAGS += '-Wno-unused-local-typedefs '
  CXX_FLAGS += '-Wno-unused-variable '
  CXX_FLAGS += '-Wno-unused-but-set-variable '

elif _platform == "win32":
  print("win32!")
  include_dirs += ['extern/osqp/include/windows']
  print(include_dirs)
  libraries = ['User32', 'kernel32']
  #CXX_FLAGS += '-DIS_WINDOWS '
  CXX_FLAGS += '-DWIN32 '
  CXX_FLAGS += '-DGLEW_STATIC '
  CXX_FLAGS += '/std:c++17 '
elif _platform == "darwin":
  print("darwin!")
  CXX_FLAGS += '-fpermissive '
  include_dirs += ['extern/osqp/include/macosx']
  os.environ[
      'LDFLAGS'] = '-framework Cocoa -mmacosx-version-min=10.7 -stdlib=libc++ -framework OpenGL'
  CXX_FLAGS += '-DB3_NO_PYTHON_FRAMEWORK '
  CXX_FLAGS += '-DHAS_SOCKLEN_T '
  CXX_FLAGS += '-D_DARWIN '
  CXX_FLAGS += '-stdlib=libc++ '
  CXX_FLAGS += '-mmacosx-version-min=10.7 '
  #    CXX_FLAGS += '-framework Cocoa '
else:
  print("bsd!")
  include_dirs += ['extern/osqp/include/linux']
  libraries = ['GL', 'GLEW', 'pthread']
  os.environ['LDFLAGS'] = '-L/usr/X11R6/lib'
  CXX_FLAGS += '-D_BSD '
  CXX_FLAGS += '-I/usr/X11R6/include '
  CXX_FLAGS += '-DHAS_SOCKLEN_T '
  CXX_FLAGS += '-fno-inline-functions-called-once'

setup_py_dir = os.path.dirname(os.path.realpath(__file__))

extensions = []

mpc_osqp_ext = Extension("mpc_osqp",
                         sources=sources,
                         libraries=libraries,
                         extra_compile_args=CXX_FLAGS.split(),
                         include_dirs=include_dirs + ["."])

extensions.append(mpc_osqp_ext)

print(find_packages('.'))

setup(
    name='rl_mpc_locomotion',
    version='1.0.0',
    description=
    'Convex MPC controller with python bindings for MPC using the osqp solver',
    long_description=
    'Convex MPC controller with python bindings for MPC using the Isaac Gym and the OSQP solver',
    url='https://github.com/silvery107/rl_mpc_locomotion',
    author='Yulun Zhuang',
    author_email='z1079931505@gmail.com',
    license='mixed',
    platforms='any',
    keywords=['robotics', 'control', 'physics simulation'],
    install_requires=[
        'numpy'
    ],
    ext_modules=extensions,
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'License :: OSI Approved :: zlib/libpng License',
        'Operating System :: Microsoft :: Windows',
        'Operating System :: POSIX :: Linux', 'Operating System :: MacOS',
        'Intended Audience :: Science/Research',
        "Programming Language :: Python",
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3.4',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
        'Programming Language :: Python :: 3.8',
        'Topic :: Games/Entertainment :: Simulation',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Framework :: Robot Framework'
    ],
    packages=[x for x in find_packages('.')],
)
