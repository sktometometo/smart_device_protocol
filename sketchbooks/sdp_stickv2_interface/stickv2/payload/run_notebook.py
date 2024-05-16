import os
import time
import sys

pid = sys.argv[1]

pid_server = '0'
try:
    pid_server = sys.argv[2]
except:
    pass

time.sleep(1)
if pid_server != '0':
    os.system("kill " + pid_server)
os.system("kill " + pid)
os.system("jupyter notebook --no-browser --ip=0.0.0.0 --port=80 --allow-root --NotebookApp.token='' --NotebookApp.notebook_dir='/home/notebook' &")