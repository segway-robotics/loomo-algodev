import os
import shutil

cur_dir = os.path.dirname(os.path.realpath(__file__))
cfg_file = os.path.join(cur_dir,"lib_list.cfg")
file = open(cfg_file,'r')
file_contain = file.read()
print file_contain

file_contain = file_contain.split('\n')
print file_contain

file_list = file_contain[0]
print file_list

file_list = file_list.split(';')
print len(file_list)

for name in file_list:
    print name 

target_dir = file_contain[1]
print target_dir

if(os.path.exists(target_dir)):
    shutil.rmtree(target_dir)
    
if not os.path.exists(target_dir):
    os.makedirs(target_dir)


for name in file_list :
    (path , file_name) = os.path.split(name)
    libsrc = os.path.join(path,file_name)
    target_name = os.path.join(target_dir,file_name)
    print target_name
    shutil.copy(libsrc, target_dir)
    