from setuptools import setup
from pathlib import Path
p='ur7e_nvblox_in_hand'
data=[('share/ament_index/resource_index/packages',['resource/'+p]),('share/'+p,['package.xml'])]
for folder in ['launch','config','rviz']:
    for d in sorted({str(x.parent) for x in Path(folder).rglob('*') if x.is_file() and '__pycache__' not in str(x)}):
        data.append(('share/'+p+'/'+d,[str(x) for x in Path(d).iterdir() if x.is_file()]))
setup(name=p,version='0.1.0',packages=[p],data_files=data,install_requires=['setuptools'],zip_safe=True,
      entry_points={'console_scripts':['mapper_node = '+p+'.node:main']})
