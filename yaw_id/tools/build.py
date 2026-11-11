"""Build the Keil target with installed Arm Compiler 6, without opening uVision.

python tools/build.py --toolchain D:/ARM/ARMCLANG
Only compiles/links files; never connects to a probe or flashes hardware.
"""
from pathlib import Path
import argparse
import concurrent.futures
import hashlib
import os
import subprocess
import sys
import xml.etree.ElementTree as ET


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--toolchain',default=os.environ.get('ARMCLANG_ROOT','D:/ARM/ARMCLANG'))
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    project=root/'MDK-ARM/Hero_gimbal.uvprojx'
    target=ET.parse(project).find('.//Target')
    build=root/'build/armclang';build.mkdir(parents=True,exist_ok=True)
    output=root/'MDK-ARM/YawId';output.mkdir(exist_ok=True)
    binpath=Path(args.toolchain)/'bin'
    for name in ['armclang.exe','armasm.exe','armlink.exe','fromelf.exe']:
        if not (binpath/name).is_file():raise SystemExit('Missing tool: '+str(binpath/name))
    controls=target.find('.//Cads/VariousControls')
    includes=[(project.parent/part.replace('\\','/')).resolve()
              for part in controls.findtext('IncludePath').split(';') if part]
    definitions=controls.findtext('Define').split(',')
    sources=[(project.parent/f.findtext('FilePath').replace('\\','/')).resolve()
             for f in target.findall('Groups/Group/Files/File')
             if f.findtext('FileType') in ['1','2','8']]
    missing=[str(p) for p in sources if not p.is_file()]
    if missing:raise SystemExit('Missing project sources:\n'+'\n'.join(missing))
    def compile_file(source):
        object_file=build/(source.stem+'_'+hashlib.sha1(str(source).encode()).hexdigest()[:8]+'.o')
        if source.suffix.lower()=='.s':
            command=[str(binpath/'armasm.exe'),'--cpu','Cortex-M4.fp.sp','--fpu','FPv4-SP',
                     '--apcs=interwork','--debug',str(source),'-o',str(object_file)]
        else:
            command=[str(binpath/'armclang.exe'),'--target=arm-arm-none-eabi','-mcpu=cortex-m4',
                     '-mfpu=fpv4-sp-d16','-mfloat-abi=hard','-fshort-enums','-fshort-wchar',
                     '-O1','-g','-ffunction-sections','-fdata-sections','-Wall','-Wextra',
                     '-Wno-unused-parameter','-Wno-missing-field-initializers',
                     *['-I'+str(p) for p in includes],*['-D'+d for d in definitions]]
            if source.suffix.lower()=='.cpp':
                command+=['-std=c++17','-fno-exceptions','-fno-rtti','-fno-threadsafe-statics']
            else:command+=['-std=c11']
            command+=['-c',str(source),'-o',str(object_file)]
        result=subprocess.run(command,cwd=project.parent,capture_output=True)
        return source,object_file,result
    objects=[];logs=[];failed=False
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
        for source,obj,result in executor.map(compile_file,sources):
            text=(result.stdout+result.stderr).decode('utf-8',errors='replace')
            logs.append(str(source)+'\n'+text)
            if result.returncode:failed=True;print(text)
            elif text.strip():print(text)
            objects.append(obj)
    (build/'compile.log').write_text('\n'.join(logs),encoding='utf-8')
    if failed:raise SystemExit('Compilation failed; see build/armclang/compile.log')
    image=output/'yaw_id.axf'
    command=[str(binpath/'armlink.exe'),'--cpu','Cortex-M4.fp.sp','--scatter',str(root/'app/yaw_id.sct'),
             '--entry','Reset_Handler','--map','--symbols','--info','sizes,totals',
             '--list',str(output/'yaw_id.map'),'--output',str(image),*map(str,objects)]
    result=subprocess.run(command,cwd=project.parent,capture_output=True)
    text=(result.stdout+result.stderr).decode('utf-8',errors='replace')
    (build/'link.log').write_text(text,encoding='utf-8');print(text)
    if result.returncode:raise SystemExit('Link failed')
    for fmt,suffix in [('--bin','.bin'),('--i32combined','.hex')]:
        subprocess.run([str(binpath/'fromelf.exe'),fmt,'--output',str(image.with_suffix(suffix)),str(image)],check=True)
    print('BUILD PASS: '+str(image))
    print('SHA256: '+hashlib.sha256(image.read_bytes()).hexdigest())


if __name__=='__main__':
    sys.stdout.reconfigure(encoding='utf-8')
    main()
