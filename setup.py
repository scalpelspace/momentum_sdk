"""Momentum SDK - Python Package Setup with Auto-Generated Stubs"""

from pybind11.setup_helpers import Pybind11Extension, build_ext
from setuptools import setup
import pybind11
import os
import sys
import subprocess
import shutil
from pathlib import Path
import re

class CustomBuildExt(build_ext):
    
    """Builds extensions and generates .pyi stubs"""
    
    def build_extensions(self):
        super().build_extensions()
        self.generate_stubs()

    def generate_stubs(self):
        """Generate .pyi stub files using pybind11-stubgen"""
        print("Generating .pyi stub files...")
        build_lib = None 
        for ext in self.extensions:
            module_name = ext.name
            try:
                build_lib = os.path.abspath(self.build_lib)
                print(f"  Build directory: {build_lib}")

                so_files = list(Path(build_lib).glob(f"{module_name}*.so"))
                if not so_files:
                    print(f"  Warning: No .so file found for {module_name}")
                    continue
                print(f"  Found .so file: {so_files[0]}")

                if build_lib not in sys.path:
                    sys.path.insert(0, build_lib)

                cmd = [
                    sys.executable, "-m", "pybind11_stubgen",
                    module_name,
                    "-o", build_lib,
                    "--ignore-invalid-expressions", ".*",
                    "--ignore-all-errors"
                ]
                
                print(f"  Running: {' '.join(cmd)}")
                result = subprocess.run(cmd, capture_output=True, text=True, cwd=build_lib)
                
                if result.returncode == 0:
                    print(f"  Success: pybind11-stubgen succeeded")
                    self._finalize_stubs(module_name, build_lib)
                    print(f"Stub generation complete for {module_name}")
                else:
                    print(f"Error: Stub generation failed for {module_name}")
                    print(f"  stdout: {result.stdout}")
                    print(f"  stderr: {result.stderr}")
            except Exception as e:
                print(f"Error generating stubs for {module_name}: {e}")
            finally:
                 if build_lib is not None and build_lib in sys.path: 
                    sys.path.remove(build_lib)

    def _finalize_stubs(self, module_name, build_lib):
        
        """Finalize stub files in build directory for installation"""

        build_lib_path = Path(build_lib)
        
        stub_patterns = [
            build_lib_path / module_name / "__init__.pyi",
            build_lib_path / f"{module_name}.pyi"
        ]
        stub_src = None
        for pattern in stub_patterns:
            if pattern.exists():
                stub_src = pattern
                break
        
        if stub_src:
            final_stub = build_lib_path / f"{module_name}.pyi"
            if stub_src != final_stub:
                shutil.move(str(stub_src), str(final_stub))
                if stub_src.parent != build_lib_path and stub_src.parent.is_dir() and not any(stub_src.parent.iterdir()):
                     stub_src.parent.rmdir()
                print(f"  Moved stub to {final_stub}")
            else:
                print(f"  Stub file ready: {final_stub}")
            
            self._post_process_stub(final_stub, module_name)
            
            py_typed = build_lib_path / "py.typed"
            py_typed.write_text("# PEP 561 marker\n")
            print(f"  Created {py_typed}")
        else:
            print(f"  Error: No stub file found for {module_name}")

    def _post_process_stub(self, stub_file_path, module_name):
        
        """Fix key stub issues for VS Code autocomplete"""

        print(f"  Post-processing stub file...")
        try:
            with open(stub_file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Fix return types (remove module prefix, handle Optional)
            content = self._fix_return_types(content, module_name)
            
            # Fix tuple types by parsing docstrings
            content = self._fix_tuple_types(content)
            
            # Fix property setter argument types
            content = self._fix_property_setter_types(content)
            
            # Ensure necessary typing imports are present
            content = self._ensure_imports(content)

            with open(stub_file_path, 'w', encoding='utf-8') as f:
                f.write(content)
                
            print(f"  Stub file post-processing complete")
        except Exception as e:
            print(f"  Error post-processing stub file: {e}")

    def _fix_return_types(self, content: str, module_name: str) -> str:
        # Fix "mod.Class | None" -> "Optional[Class]"
        content = re.sub(rf"{re.escape(module_name)}\.(\w+)\s*\|\s*None", r"Optional[\1]", content)
        # Fix "mod.Class" -> "Class" (for non-None returns)
        content = re.sub(rf"{re.escape(module_name)}\.(\w+)(?!\w)", r"\1", content)
        # Fix "Union[Type, None]" -> "Optional[Type]"
        content = re.sub(r"Union\[([^,\[\]]+),\s*None\]", r"Optional[\1]", content)
        return content

    def _fix_tuple_types(self, content: str) -> str:
       
        # Function to determine specific Tuple type from docstring
        def get_tuple_type_from_docstring(match):
            full_text = match.group(0)
            if 'quaternion' in full_text.lower() or '(w, x, y, z)' in full_text:
                return full_text.replace('-> tuple:', '-> Tuple[float, float, float, float]:')
            elif any(p in full_text.lower() for p in ['atmospheric', '(pressure', 'velocity as (speed', '(speed,', 'course)']):
                 return full_text.replace('-> tuple:', '-> Tuple[float, float]:')
            elif any(p in full_text.lower() for p in ['(x, y, z)', '(lat, lon, alt)', 'acceleration', 'gyroscope', 'gravity', 'linear acceleration', 'position as']):
                return full_text.replace('-> tuple:', '-> Tuple[float, float, float]:')
            else:
                return full_text.replace('-> tuple:', '-> Tuple[Any, ...]:')

        # Pattern to match "-> tuple:" followed by a docstring (handles multiline/quotes)
        pattern = r'-> tuple:\s*""".*?"""'
        content = re.sub(pattern, get_tuple_type_from_docstring, content, flags=re.DOTALL)
        
        # Fallback for any remaining "-> tuple:" without easily parseable docstrings
        content = re.sub(r'-> tuple:', '-> Tuple[Any, ...]:', content) 
        return content

    def _fix_property_setter_types(self, content: str) -> str:
        
        content = re.sub(r'arg1:\s*typing\.SupportsFloat', 'arg1: float', content)
        content = re.sub(r'arg1:\s*typing\.SupportsInt', 'arg1: int', content)
        content = re.sub(r'arg1:\s*typing\.SupportsBytes', 'arg1: bytes', content)
       
        # Also fix direct references like just "SupportsFloat"
        content = re.sub(r'arg1:\s*SupportsFloat', 'arg1: float', content)
        content = re.sub(r'arg1:\s*SupportsInt', 'arg1: int', content)
        content = re.sub(r'arg1:\s*SupportsBytes', 'arg1: bytes', content)
        return content

    def _ensure_imports(self, content: str) -> str:
        
        # Check for existing 'from typing import ...' line
        typing_import_pattern = r'^from typing import (.+)$'
        typing_match = re.search(typing_import_pattern, content, re.MULTILINE)
        
        required_imports = {'Optional', 'Tuple', 'Any'}
        if typing_match:
           
            # Parse existing imports
            existing_imports_str = typing_match.group(1)
            existing_imports = {imp.strip() for imp in existing_imports_str.split(',')}
           
            # Merge required and existing, remove duplicates
            all_imports = sorted(required_imports.union(existing_imports))
            
            # Replace the old import line
            new_import_line = f"from typing import {', '.join(all_imports)}"
            content = re.sub(typing_import_pattern, new_import_line, content, flags=re.MULTILINE)
        else:

            # No typing import line found, add one
            needed_import_line = f"from typing import {', '.join(sorted(required_imports))}\n"
            
            # Try to find a good place to insert, e.g., after __future__ imports
            future_import_match = re.search(r'^from __future__ import .+$', content, re.MULTILINE)
            if future_import_match:
                insert_pos = future_import_match.end() + 1

            else:

                # Insert after module docstring if it exists, otherwise at the top
                docstring_match = re.search(r'^""".*?"""', content, re.DOTALL)
                if docstring_match:
                    insert_pos = docstring_match.end()
                    if insert_pos < len(content) and content[insert_pos] != '\n':
                         needed_import_line = '\n' + needed_import_line
                else:
                    insert_pos = 0
                    needed_import_line += '\n'
            
            content = content[:insert_pos] + needed_import_line + content[insert_pos:]
            
        return content


# Extension module configuration
ext_modules = [
    Pybind11Extension(
        "momentum_sdk",
        sources=[
            "python/momentum_bindings.cpp",
            "src/can/can_interface.cpp",
            "src/can/can_utilities.cpp",
            "src/protocol/message_parser.cpp",
            "momentum_driver/momentum_can_driver.c",
            "momentum_driver/momentum_can_dbc.c",
            "src/momentum_sdk.cpp",
        ],
        include_dirs=["include", "momentum_driver", pybind11.get_include()],
        language='c++',
        cxx_std=17,
        define_macros=[("VERSION_INFO", '\"1.0.0\"')],
    ),
]

setup(
    name="momentum-sdk",
    version="1.0.0",
    author="ScalpelSpace",
    author_email="info@scalpelspace.com",
    description="Python bindings for Momentum SDK",
    long_description="Modern pybind11 package with automatic .pyi stub generation for VS Code IntelliSense.",
    long_description_content_type="text/plain",
    url="https://github.com/scalpelspace/momentum_sdk",
    ext_modules=ext_modules,
    cmdclass={"build_ext": CustomBuildExt},
    packages=[],  
    zip_safe=False,
    python_requires=">=3.10",
    install_requires=["pybind11>=2.6.0"],
    package_data={
        "": ["*.pyi", "py.typed"],
    },
    include_package_data=True,
    classifiers=[
        "Development Status :: 2 - Pre-Alpha",
        "Intended Audience :: Developers",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "Programming Language :: C++",
    ],
)
