import os
import sys
import subprocess
import time
import signal

KEY_PATH = '/sys/class/gpio/gpio2/value'
f_key = open(KEY_PATH, 'r')

def killProcess(process):
    # process.stdout.close()
    os.kill(process.pid, signal.SIGKILL)
    timeout = 0
    while True:
        poll = process.poll()
        if poll is None:
            timeout += 0.1
            time.sleep(0.1)
            if timeout > 5:
                print('kill process (%d) failed.' %(process.pid))
                exit('system exit.')
        else:
            # print('kill process (%d) used %.1f sec.' %(process.pid, timeout))
            break
    process.terminate()
    process.kill()
    time.sleep(2)

process = subprocess.Popen(['python3','server_core.py', str(os.getpid())], stdout=subprocess.PIPE, stdin=subprocess.PIPE, close_fds = True, bufsize = 1)

press_cnt = 0
try:
    while True:
        x = f_key.read()
        key_is_pressed = x[0] == '0'

        if key_is_pressed:
            press_cnt += 0.1
            if press_cnt > 2:
                press_cnt = 0
                print("\n\nSTOPPING PROCESS\n\n")
                killProcess(process)

                print('RESTART PROCESS')
                process = subprocess.Popen(['python3','server_core.py', str(os.getpid())], stdout=subprocess.PIPE, stdin=subprocess.PIPE, close_fds = True, bufsize = 1)
        else:
            press_cnt = 0

        f_key.seek(0, 0)
        time.sleep(0.1)
        
except KeyboardInterrupt:
    killProcess(process)
    print('system exit.')
