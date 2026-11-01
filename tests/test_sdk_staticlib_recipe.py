"""FC validates the SDK's actual subdirectory command arguments, not labels.

These fixtures configure a Python-only SDK stand-in and inspect its generated
commands without executing build targets: no compiler jobs,
network, SDK cache mutation or physical outputs. SDK's own suite separately
covers its real CMake forwarding and unchanged Rust-consumer defaults.
"""
from pathlib import Path
import hashlib
import json
import shutil
import shlex
import subprocess
import sys
import tempfile
import unittest
ROOT=Path(__file__).resolve().parents[1]
MODULE=ROOT/'cmake/fc_sedsnet_artifact.cmake'

class StaticlibRecipeTests(unittest.TestCase):
    def configure(self, sdk_args, *, select=True, gate=True):
        directory=tempfile.TemporaryDirectory(prefix='fc-sdk-artifact-')
        self.addCleanup(directory.cleanup)
        root=Path(directory.name);sdk=root/'sdk';sdk.mkdir()
        (sdk/'record.py').write_text('import json,sys\nfrom pathlib import Path\nPath(sys.argv[1]).write_text(json.dumps(sys.argv[2:]))\n')
        sdk_cmake='''# A misleading comment must never count as a forwarding capability:
# --crate-type staticlib
set(SEDSNET_CRATE_TYPE "" CACHE STRING "Generic SDK default")
set(RS_ARGS_BOOTSTRAP build --features embedded)
'''+sdk_args+'''
add_custom_command(OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/argv.json"
    COMMAND "${PYTHON_FOR_TEST}" "${CMAKE_CURRENT_SOURCE_DIR}/record.py"
            "${CMAKE_CURRENT_BINARY_DIR}/argv.json" ${RS_ARGS_BOOTSTRAP}
    VERBATIM)
add_custom_target(sdk-archive DEPENDS "${CMAKE_CURRENT_BINARY_DIR}/argv.json")
'''
        (sdk/'CMakeLists.txt').write_text(sdk_cmake)
        before=hashlib.sha256((sdk/'CMakeLists.txt').read_bytes()).hexdigest()
        project='''cmake_minimum_required(VERSION 3.22)
project(fc_sdk_recipe_test NONE)
'''+f'include("{MODULE.as_posix()}")\n'
        # Deliberately leave a plausible parent variable. Only the SDK child's
        # actual RS_ARGS may satisfy the check.
        project+='set(RS_ARGS_BOOTSTRAP build --crate-type staticlib)\n'
        if select:project+='fc_select_sedsnet_staticlib()\n'
        project+=f'set(PYTHON_FOR_TEST "{Path(sys.executable).as_posix()}")\nadd_subdirectory(sdk)\n'
        if gate:project+='fc_verify_sedsnet_staticlib("${CMAKE_CURRENT_SOURCE_DIR}/sdk")\n'
        (root/'CMakeLists.txt').write_text(project)
        command=['cmake','-S',str(root),'-B',str(root/'build')]
        self.assertIsNotNone(shutil.which('ninja'),'Ninja is required for generated-command inspection')
        command+=['-G','Ninja']
        result=subprocess.run(command,text=True,capture_output=True)
        self.assertEqual(before,hashlib.sha256((sdk/'CMakeLists.txt').read_bytes()).hexdigest())
        return root,result

    def generated_arguments(self, root):
        # Ninja tool mode only reads the generated graph; it launches no build
        # target, Python recorder or compiler. Check the command SDK would run.
        command=subprocess.run(['ninja','-C',str(root/'build'),'-t','commands','sdk-archive'],text=True,capture_output=True)
        self.assertEqual(command.returncode,0,command.stdout+command.stderr)
        lines=[line for line in command.stdout.splitlines() if 'record.py' in line]
        self.assertEqual(len(lines),1,command.stdout)
        argv=shlex.split(lines[0]);record=next(i for i,value in enumerate(argv) if value.endswith('/record.py'))
        return argv[record+2:]

    def reject(self, args):
        _,result=self.configure(args)
        self.assertNotEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertIn('requires SEDSnet 4.3.6',result.stdout+result.stderr)
        self.assertIn('--crate-type staticlib',result.stdout+result.stderr)

    def test_old_sdk_cache_label_comment_and_parent_variable_do_not_count(self):
        self.reject('')

    def test_wrong_artifact_is_rejected(self):
        self.reject('list(APPEND RS_ARGS_BOOTSTRAP --crate-type rlib)')

    def test_missing_selector_value_is_actionable(self):
        self.reject('list(APPEND RS_ARGS_BOOTSTRAP --crate-type)')

    def test_joined_label_does_not_replace_exact_forwarded_argument(self):
        self.reject('list(APPEND RS_ARGS_BOOTSTRAP "--crate-type=staticlib")')

    def test_selected_staticlib_reaches_generated_build_command(self):
        root,result=self.configure('''if(NOT SEDSNET_CRATE_TYPE STREQUAL "")
list(APPEND RS_ARGS_BOOTSTRAP --crate-type "${SEDSNET_CRATE_TYPE}")
endif()''')
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual(self.generated_arguments(root),['build','--features','embedded','--crate-type','staticlib'])

    def test_sdk_generic_default_has_no_selector_without_fc_opt_in(self):
        root,result=self.configure('''if(NOT SEDSNET_CRATE_TYPE STREQUAL "")
list(APPEND RS_ARGS_BOOTSTRAP --crate-type "${SEDSNET_CRATE_TYPE}")
endif()''',select=False,gate=False)
        self.assertEqual(result.returncode,0,result.stdout+result.stderr)
        self.assertEqual(self.generated_arguments(root),['build','--features','embedded'])
