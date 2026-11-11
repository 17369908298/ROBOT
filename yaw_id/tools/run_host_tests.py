"""Run host regression using existing MSVC (no MCU connection)."""
from pathlib import Path
import os
import shutil
import subprocess
import sys

root=Path(__file__).resolve().parents[1]
build=root/'build/host_tests';build.mkdir(parents=True,exist_ok=True)
subprocess.run([sys.executable,'-m','unittest','discover','-s',str(root/'tests'),'-p','test_*.py','-v'],
               cwd=root,check=True)
env={key.upper():value for key,value in os.environ.items()}
if not shutil.which('cl.exe'):
    vswhere=Path(os.environ.get('ProgramFiles(x86)','C:/Program Files (x86)'))/'Microsoft Visual Studio/Installer/vswhere.exe'
    location=subprocess.check_output([str(vswhere),'-latest','-products','*','-requires',
                                     'Microsoft.VisualStudio.Component.VC.Tools.x86.x64',
                                     '-property','installationPath'],text=True).strip()
    vcvars=Path(location)/'VC/Auxiliary/Build/vcvars64.bat'
    bootstrap=build/'msvc_environment.cmd'
    bootstrap.write_text('@echo off\ncall "'+str(vcvars)+'" >nul\nif errorlevel 1 exit /b 1\nset\n',encoding='utf-8')
    output=subprocess.check_output(['cmd','/d','/c',str(bootstrap)],text=True)
    for line in output.splitlines():
        key,sep,value=line.partition('=')
        if sep and key:env[key.upper()]=value
exe=build/'test_core.exe'
compiler=shutil.which('cl.exe',path=env['PATH'])
if not compiler:raise SystemExit('MSVC cl.exe not found after loading vcvars64')
subprocess.run([compiler,'/nologo','/std:c++17','/EHsc','/W4','/O2','/utf-8','/I'+str(root),
                str(root/'tests/test_core.cpp'),'/Fe:'+str(exe),'/Fo:'+str(build/'test_core.obj')],
               cwd=build,env=env,check=True)
subprocess.run([str(exe)],check=True)
