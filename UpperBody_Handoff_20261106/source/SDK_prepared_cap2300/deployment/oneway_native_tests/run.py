from pathlib import Path
import subprocess,tempfile
p=Path(__file__).resolve().parent;root=p.parents[1]
with tempfile.TemporaryDirectory() as d:
 out=Path(d)/'native'
 subprocess.run(['g++','-std=c++17','-pthread','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-I',str(p/'stubs'),'-I',str(root/'dexhand_control/firmware/DexHandRight'),str(p/'oneway_pwm_native_test.cpp'),'-o',str(out)],check=True)
 subprocess.run([str(out)],check=True,timeout=10)
