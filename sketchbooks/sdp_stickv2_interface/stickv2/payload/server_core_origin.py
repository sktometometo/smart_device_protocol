import sys
from flask import Flask, render_template, Response, request, send_from_directory, url_for
from werkzeug import secure_filename
import subprocess
import time
import base64
import json
import serial.threaded
import serial
import threading  
import queue
import traceback
import logging
import os
import shutil
import random
import signal
import logging

logger = logging.getLogger()
logger.setLevel(logging.DEBUG)
fh = logging.FileHandler('server_log.txt', mode='w')
fh.setLevel(logging.INFO)
formatter = logging.Formatter("%(asctime)s - [line:%(lineno)d] - %(levelname)s: %(message)s")
fh.setFormatter(formatter)
logger.addHandler(fh)
ch = logging.StreamHandler()
ch.setLevel(logging.DEBUG)
ch.setFormatter(formatter)
logger.addHandler(ch)

PNAME_MAP = {
    'audio_fft': 'Audio FFT',
    'code_detector': 'Code Detector',
    'face_detector': 'Face Detector',
    'lane_line_tracker': 'Lane Line Tracker',
    'motion_tracker': 'Motion Tracker',
    'shape_matching': 'Shape Matching',
    'camera_stream': 'Camera Stream',
    'online_classifier': 'Online Classifier',
    'color_tracker': 'Color Tracker',
    'face_recognition': 'Face Recognition',
    'target_tracker': 'Target Tracker',
    'shape_detector': 'Shape Detector',
    'object_recognition': 'Object Recognition',

    'Audio FFT': 'audio_fft',
    'Code Detector': 'code_detector',
    'Face Detector': 'face_detector',
    'Lane Line Tracker': 'lane_line_tracker',
    'Motion Tracker': 'motion_tracker',
    'Shape Matching': 'shape_matching',
    'Camera Stream': 'camera_stream',
    'Online Classifier': 'online_classifier',
    'Color Tracker': 'color_tracker',
    'Face Recognition': 'face_recognition',
    'Target Tracker': 'target_tracker',
    'Shape Detector': 'shape_detector',
    'Object Recognition': 'object_recognition',
}

FUNCTIONS = set(['audio_fft', 'code_detector', 'face_detector', 'lane_line_tracker', 'motion_tracker', 'shape_matching', 'camera_stream', 'online_classifier', 'color_tracker', 'face_recognition', 'target_tracker', 'shape_detector', 'object_recognition'])
FLASK_ALLOWED_EXTENSIONS = set(['png', 'jpg', 'jpeg', 'gif', 'zip', 'tar', 'gz'])
FLASK_ALLOWED_EXTENSIONS_MODELS = set(['tar', 'm5m'])#set(['zip', 'tar', 'gz'])
FLASK_ALLOWED_EXTENSIONS_FIRMWARE = set(['zip', 'tar', 'gz'])
FLASK_ALLOWED_EXTENSIONS_IMAGES = set(['png', 'jpg', 'jpeg', 'gif'])
app = Flask(__name__)
app.config['UPLOAD_FOLDER_TEMP'] = './uploads/temp'
app.config['UPLOAD_FOLDER_MODELS'] = './uploads/models'
app.config['UPLOAD_FOLDER_FIRMWARE'] = './uploads/firmware'
app.config['UPLOAD_FOLDER_IMAGE'] = './uploads/image'
app.config['MAX_CONTENT_LENGTH'] = 1024 * 1024 * 1024 # 1GB

serial_rx_queue = queue.Queue(10)
pipe_meta_rx_queue = queue.Queue(10)
pipe_img_rx_queue = queue.Queue(10)
pipe_msg_rx_queue = queue.Queue(10)
pipe_label_rx_queue = queue.Queue(10)
pipe_result_rx_queue = queue.Queue(10)
process = None
process_name = None
protocol = None
client_is_connected = False
description_doc = {}
system_config_doc = {}
thread_pipe_recv = None
net_models = []
server_pid = 0

def enableStream():
    global process
    if not process is None:
        process.stdin.write("_{\"stream\":1}\r\n".encode('utf-8'))
        process.stdin.flush()

def checkNetModels():
    files = os.listdir(app.config['UPLOAD_FOLDER_MODELS'])
    models = []
    for f in files:
        if os.path.isdir(os.path.join(app.config['UPLOAD_FOLDER_MODELS'], f)):
            models.append(f)
        # g = f.rsplit('.', 1)
    return models

def clearQueue(q):
    while True:
        if q.empty():
            break
        else:
            q.get(False)
            q.task_done()

def switchFunction(pname, args):
    global process, process_name, protocol, client_is_connected, thread_pipe_recv, net_models, data_from_device_last_msg
    if not pname in FUNCTIONS:
        logger.error('process %s not exist.' %(pname))
        protocol.write(("{\"error\":\"process %s not exist.\"}\r\n"  %(pname)).encode('utf-8'))
        return
    thread_pipe_recv.pause()
    prev_process_name = process_name
    process_name = pname
    process_path = './bin/' + pname
    
    data_from_device_last_msg = '{\'running\':\'null\'}'
    clearQueue(pipe_meta_rx_queue)
    clearQueue(pipe_img_rx_queue)
    clearQueue(pipe_msg_rx_queue)
    clearQueue(pipe_result_rx_queue)
    clearQueue(pipe_label_rx_queue)

    # Specify model for yolo detection
    if pname == 'object_recognition':
        try:
            if args[0] in net_models:
                modelpath = os.path.join(app.config['UPLOAD_FOLDER_MODELS'], args[0].rsplit('.', 1)[0])
                args = [modelpath]
                system_config_doc['yolo_last_model_path'] = modelpath
                saveConfig()
            else:
                raise Exception
        except:
            try:
                args = [system_config_doc['yolo_last_model_path']]
            except:
                args = [os.path.join(app.config['UPLOAD_FOLDER_MODELS'], 'yolo_20class')]
        logger.debug('Yolo Model: %s' %args[0])
        pipe_msg_rx_queue.put(json.dumps({'running':'Object Recongnition', 'running_model':args[0], 'models':net_models, 'web':1}))

    pipe_label_rx_queue.put(json.dumps({'render':0}).encode('ascii'))
    pipe_label_rx_queue.put(json.dumps({'render':0}).encode('ascii'))
    pipe_label_rx_queue.put(json.dumps({'render':0}).encode('ascii'))
    pipe_label_rx_queue.put(json.dumps({'render':0}).encode('ascii'))

    if process is not None:
        process.stdout.close()
        os.kill(process.pid, signal.SIGKILL)
        timeout = 0
        while True:
            poll = process.poll()
            if poll is None:
                timeout += 0.1
                time.sleep(0.1)
                if timeout > 5:
                    logger.critical('kill process (%d)%s failed.' %(process.pid, prev_process_name))
                    return
            else:
                logger.debug('kill process (%d)%s used %.1f sec.' %(process.pid, prev_process_name, timeout))
                break
        process.terminate()
        process.kill()

        process = None
        time.sleep(1)

    try:
        if(len(args)):
            logger.debug('popen command: ' + str([process_path] + args))
            process = subprocess.Popen([process_path] + args, stdout=subprocess.PIPE, stdin=subprocess.PIPE, close_fds = True, bufsize = 1)
        else:
            logger.debug('popen command: ' + str(process_path))
            process = subprocess.Popen(process_path, stdout=subprocess.PIPE, stdin=subprocess.PIPE, close_fds = True, bufsize = 1)

        if process is None:
            logger.critical('popen %s error.' %process_path)
            raise Exception
    except:
        protocol.write("{\"error\":\"invalid function.\"}\r\n".encode('utf-8'))
        logger.critical('invalid function.')
    process.stdout.flush()
    if client_is_connected is True:
        enableStream()
    
    thread_pipe_recv.resume()
    protocol.write(("{\"msg\":\"function switched to %s.\"}\r\n" %process_name).encode('utf-8'))

class SerialRX(serial.threaded.LineReader):
    def connection_made(self, transport):
        super(SerialRX, self).connection_made(transport)
        sys.stdout.write('port opened\n')

    def handle_line(self, data):
        global process, process_name, protocol, system_config_doc
        try:
            logger.debug('Serial recive: ' + str(data))
            doc = json.loads(data)
            if 'function' in doc:
                process_name = doc['function']
                if process_name.islower() == False:
                    process_name = process_name.lower().replace(' ', '_')
                logger.debug('Serial open ' + process_name)
                switchFunction(process_name, doc['args'])
            elif 'config' in doc:
                if doc['config'] != PNAME_MAP[process_name]:
                    protocol.write("{\"error\":\"Config does not match the running function\"}\r\n".encode('utf-8'))
                    return
                process.stdin.write((json.dumps(doc) + "\r\n").encode('utf-8'))
                process.stdin.flush()
            elif 'system_config' in doc:
                del doc['system_config']
                system_config_doc = dict(system_config_doc, **doc)
                saveConfig()
            else:
                logger.warning('Command is not supported')
                protocol.write("{\"error\":\"Command is not supported\"}\r\n".encode('utf-8'))
            # serial_rx_queue.put(doc)
        except:
            protocol.write("{\"error\":\"invalid json format.\"}\r\n".encode('utf-8'))
            logger.error('invalid json format.')

    def connection_lost(self, exc):
        if exc:
            traceback.print_exc(exc)
        sys.stdout.write('port closed\n')


class Thread_PIPERecv (threading.Thread):
    def __init__(self, queue):
        threading.Thread.__init__(self)
        self.queue = queue
        self.recv = True

    def run(self):
        self.PipeRecive()

    def pause(self):
        self.recv = False
    
    def resume(self):
        self.recv = True

    def PipeRecive(self):
        global process, protocol
        while True:
            try:
                output = process.stdout.readline()
                if process.poll() is not None and output == '':
                    logger.info('pipe exit.')
                    break
            except:
                time.sleep(0.1)
                continue

            if self.recv == False:
                time.sleep(0.01)
                continue
            
            if output:
                payload = output.strip()
                try:
                    doc = json.loads(payload)
                    if doc is None:
                        continue
                    # video preview
                    if "img" in doc:
                        if client_is_connected == True:
                            byte_data = base64.b64decode(doc["img"])
                            frame = bytes(byte_data)
                            # print('[server] pipe_img_rx_queue size = %d' %pipe_img_rx_queue.qsize())
                            if pipe_img_rx_queue.qsize() >= 10:
                                continue 
                            pipe_img_rx_queue.put(b'--frame\r\n'
                                            b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
                    # meta data
                    elif "meta" in doc:
                        if client_is_connected == True:
                            if pipe_meta_rx_queue.qsize() >= 10:
                                continue
                            pipe_meta_rx_queue.put(payload)
                    # image render
                    elif "render" in doc:
                        # print('[server] pipe_label_rx_queue size = %d' %pipe_label_rx_queue.qsize())
                        if client_is_connected == True:
                            if pipe_label_rx_queue.qsize() >= 10:
                                continue 
                            pipe_label_rx_queue.put(payload)#json.dumps(doc['items'])
                    # web data
                    elif "web" in doc:
                        if client_is_connected == True:
                            # print('[server] pipe_msg_rx_queue size = %d' %pipe_msg_rx_queue.qsize())
                            if pipe_msg_rx_queue.qsize() >= 10:
                                x = pipe_msg_rx_queue.get()
                                logger.warning('pipe_msg_rx_queue full, discard: ' + str(x))
                            pipe_msg_rx_queue.put(payload)
                            logger.debug('pipe_msg_rx_queue %d, recive: %s' %(pipe_msg_rx_queue.qsize(), str(payload)))
                            # print(payload)
                    # serial data
                    else:
                        if client_is_connected == True:
                            if pipe_result_rx_queue.qsize() >= 10:
                                x = pipe_result_rx_queue.get()
                                logger.warning('pipe_result_rx_queue full, discard: ' + str(x))
                            pipe_result_rx_queue.put(payload)
                        
                        protocol.write(payload + '\r\n'.encode('ascii'))
                        logger.debug('send to serial: ' + str(doc))
                except:
                    logger.warning('except raw =' + str(payload))
                    print(traceback.print_exc())
                    continue
            else:
                time.sleep(0.01)
                continue

@app.route('/data_to_device', methods=['POST'])
def data_to_device():
    try:
        postdata = request.get_json()
        postdata['config'] = 'web_update_data'
        # print(postdata)
        data = json.dumps(postdata) + "\r\n"
        process.stdin.write(data.encode('utf-8'))
        process.stdin.flush()
        return Response('ok')
    except Exception:
        return Response(traceback.print_exc())

data_from_device_last_msg = '{\'running\':\'null\'}'
@app.route('/data_from_device', methods=['POST'])
def data_from_device():
    global data_from_device_last_msg
    try:
        if pipe_msg_rx_queue.qsize() != 0:
            msg = pipe_msg_rx_queue.get()
            pipe_msg_rx_queue.task_done()
            doc = json.loads(msg)
            # print('[server] >>>>> ')
            # print(doc)
            if 'web' in doc:
                data_from_device_last_msg = msg
                logger.debug('data_from_device return: ' + str(msg))
                return Response(msg)
        logger.debug('data_from_device return: ' + str(data_from_device_last_msg))
        return Response(data_from_device_last_msg)
    except Exception:
        return Response(traceback.print_exc())

@app.route('/color_lab', methods=['POST'])
def color_lab():
    try:
        postdata = request.get_json()
        postdata['config'] = 'web_update_data'
        # print(postdata)
        data = json.dumps(postdata) + "\r\n"
        process.stdin.write(data.encode('utf-8'))
        process.stdin.flush()
        return Response('ok')
    except Exception:
        return Response(traceback.print_exc())

    # try:
    #     while True:
    #         if pipe_msg_rx_queue.qsize() != 0:
    #             msg = pipe_msg_rx_queue.get()
    #             pipe_msg_rx_queue.task_done()
    #             doc = json.loads(msg)
    #             if 'web' in doc:
    #                 return Response(msg)
    #         time.sleep(0.1)
    # except Exception:
    #     return Response(traceback.print_exc())

'''
key value
'type_name': func name
'args': [arg1, arg2, argn]
'''

@app.route('/func', methods=['POST'])
def func():
    global description_doc, process_name
    try:
        postdata = request.get_json()
        logger.info('switch to %s' %postdata['type_name'])
        switchFunction(postdata['type_name'], postdata['args'])
        despdoc = description_doc[postdata['type_name']]
        if process_name == 'audio_fft':
            pass
        else:
            time.sleep(3) # wait for process start
        return Response(json.dumps(despdoc))
    except Exception:
        return Response(traceback.print_exc())

@app.route('/')
def index():
    global process, client_is_connected
    try:
        client_is_connected = True
        if not process is None:
            process.stdout.flush()
        # process.stdin.write("_{\"stream\":1}\r\n".encode('utf-8'))
        # process.stdin.flush()
        logger.info('Client connected.')
        # return render_template('index.html')
        return render_template('mainpage.html')
    except Exception:
        return Response(traceback.print_exc())

'''
doc['render'] = 1

---line---
doc['items'][index]['type'] = 'line'
doc['items'][index][x1]
doc['items'][index][y1]
doc['items'][index][x2]
doc['items'][index][y2]
doc['items'][index][thickness]
doc['items'][index][color]

---rectangle---
doc['items'][index]['type'] = 'rectangle'
doc['items'][index][x]
doc['items'][index][y]
doc['items'][index][w]
doc['items'][index][h]
doc['items'][index][color]

---circle---
doc['items'][index]['type'] = 'circle'
doc['items'][index][x]
doc['items'][index][y]
doc['items'][index][r]
doc['items'][index][color]

---string---
doc['items'][index]['type'] = 'string'
doc['items'][index][x]
doc['items'][index][y]
doc['items'][index][payload]
doc['items'][index][color]

---point---
doc['items'][index]['type'] = 'point'
doc['items'][index][x]
doc['items'][index][y]
doc['items'][index][color]

---polygon---
doc['items'][index]['type'] = 'polygon'
doc['items'][index][x][n] //Nth X of points
doc['items'][index][y][n] //Nth Y of points
doc['items'][index][color]
'''
last_render_items = None
@app.route('/render_items', methods=['POST'])
def render_items():

    def genItem():
        while True:
            if pipe_label_rx_queue.qsize() != 0:
                item = pipe_label_rx_queue.get()
                pipe_label_rx_queue.task_done()
                item += bytes('|'.encode('ascii'))
                yield item
            else:
                time.sleep(0.01)
                continue
    
    return Response(genItem(), mimetype='multipart/x-mixed-replace; boundary=render_items')

@app.route('/func/result', methods=['POST'])
def func_result():

    def genFuncResult():
        while True:
            if pipe_result_rx_queue.qsize() != 0:
                item = pipe_result_rx_queue.get()
                pipe_result_rx_queue.task_done()
                item += bytes('|'.encode('ascii'))
                yield item
            else:
                time.sleep(0.01)
                continue
    
    return Response(genFuncResult(), mimetype='multipart/x-mixed-replace; boundary=func_result')

    # global last_render_items
    # try:
    #     if pipe_label_rx_queue.qsize() != 0:
    #         last_render_items = pipe_label_rx_queue.get()
    #         pipe_label_rx_queue.task_done()
    #     return Response(last_render_items, mimetype='multipart/x-mixed-replace; boundary=frame')
    # except Exception:
    #     return Response(traceback.print_exc())
'''
meta[0~511]: 0~255
'''
@app.route('/meta', methods=['GET'])
def Meta():
    def genMeta():
        while True:
            if pipe_meta_rx_queue.qsize() != 0:
                meta = pipe_meta_rx_queue.get()
                pipe_meta_rx_queue.task_done()
                meta += bytes('|'.encode('ascii'))
                yield meta
            else:
                time.sleep(0.01)
                continue
    
    return Response(genMeta(), content_type='application/json')
    # return Response(genMeta(), mimetype='multipart/x-mixed-replace; boundary=meta')

'''
[key]:[function]

boot_func: Boot-up function

'''

def saveConfig():
    global system_config_doc
    f = open('./server_system_config.json', 'w')
    f.write(json.dumps(system_config_doc))
    f.close()

@app.route('/system_config', methods=['POST'])
def system_config():
    global system_config_doc
    try:
        doc = request.get_json()
        system_config_doc = dict(system_config_doc, **doc)
        saveConfig()
        return Response('ok')
    except Exception:
        return Response(traceback.print_exc())

@app.route('/get_system_config', methods=['POST'])
def get_system_config():
    global system_config_doc
    return Response(json.dumps(system_config_doc))

@app.route('/get_last_func',methods=['POST'])
def get_last_func():
    global process_name
    if(process_name == None):
        return Response('null')
    despdoc = description_doc[process_name]
    despdoc['last_func']= process_name
    return Response(json.dumps(despdoc))

@app.route('/video_feed')
def video_feed():
    enableStream()
    
    def genFrame():
        global pipe_img_rx_queue, process, client_is_connected
        logger.info('Streaming start.')
        try:
            timeout = 0
            while True:
                if timeout > 3:
                    logger.warning('Streaming timeout.')
                    timeout = 0
                    enableStream()

                if pipe_img_rx_queue.qsize() != 0:
                    timeout = 0
                    frame = pipe_img_rx_queue.get()
                    pipe_img_rx_queue.task_done()
                    yield frame
                else:
                    time.sleep(0.01)
                    timeout += 0.01
                    continue

        except GeneratorExit:
            logger.info('Video stream is closed')
            # client_is_connected = False
            # process.stdin.write("_{\"stream\":0}\r\n".encode('utf-8')) # prefix '_' used for discern system info and device data.
            # process.stdin.flush()
            # print('Client disconnected.')

    return Response(genFrame(), mimetype='multipart/x-mixed-replace; boundary=frame')

def allowed_file(filename, allowed_extensions):
    return '.' in filename and \
           filename.rsplit('.', 1)[1] in allowed_extensions

# @app.route('/download/<filename>')
# def uploaded_file(filename):
#     return send_from_directory(app.config['UPLOAD_FOLDER'], filename)

@app.route('/upload/delmodels', methods=['POST'])
def upload_delmodels():
    global net_models
    logger.debug('yolo models: ' + str(net_models))
    try:
        doc = request.get_json()
        logger.debug('del ' + doc['delmodel'])
        if doc['delmodel'] in net_models:
            modelpath = os.path.join(app.config['UPLOAD_FOLDER_MODELS'], doc['delmodel'])
            logger.debug('del model path: ' + modelpath)
            ret = os.system('rm -rf %s' %(modelpath))
            assert ret == 0
            net_models = checkNetModels()
            return Response('ok')
        else:
            return Response(json.dumps({'error':'Model does not exist'}))
    except:
        return Response(traceback.print_exc())

# upload models
@app.route('/upload/models', methods=['POST'])
def upload_models():
    global net_models, process
    try:
        os.kill(process.pid, signal.SIGSTOP)
    except:
        pass
    try:
        file = request.files['file']
        if file and allowed_file(file.filename, FLASK_ALLOWED_EXTENSIONS_MODELS):
            # filename = secure_filename(file.filename)
            filepath = os.path.join(app.config['UPLOAD_FOLDER_TEMP'], file.filename)
            file.save(filepath)

            modelname = file.filename.rsplit('.', 1)[0]
            modelpath = os.path.join(app.config['UPLOAD_FOLDER_MODELS'], modelname)
            if modelname in net_models:
                ret = os.system('rm -rf %s' %(modelpath))
                assert ret == 0

            ret = os.system('mkdir %s' %(modelpath))
            assert ret == 0

            ret = os.system('tar -xvf %s --directory %s' %(filepath, modelpath))
            assert ret == 0

            files = os.listdir(modelpath)
            if 'model.json' in files:
                pass
            else:
                ret = os.system('rm -rf %s' %(modelpath))
                assert ret == 0
                try:
                    os.kill(process.pid, signal.SIGCONT)
                except:
                    pass
                return Response(json.dumps({'error':'Invalid file format'}))
            
            net_models = checkNetModels()
            # file_url = url_for('uploaded_file', filename=filename)
            try:
                os.kill(process.pid, signal.SIGCONT)
            except:
                pass
            pipe_msg_rx_queue.put(json.dumps({'running':'Object Recongnition', 'models':net_models, 'web':1}))
            return Response('ok')
        return Response(json.dumps({'error':'Invalid file format'}))
    except:
        try:
            os.kill(process.pid, signal.SIGCONT)
        except:
            pass
        return Response(traceback.print_exc())
    
@app.route('/switch_sys_mode', methods=['POST'])
def switch_sys_mode():
    global server_pid
    logger.info('End of test.')
    pid = os.getpid()
    os.system('python3 run_notebook.py %d %d' %(pid, server_pid))
    return Response(json.dumps({'msg':'ok'}))

# upload firmware
@app.route('/upload/firmware', methods=['POST'])
def upload_firmware():
    file = request.files['file']
    if file and allowed_file(file.filename, FLASK_ALLOWED_EXTENSIONS_FIRMWARE):
        # filename = secure_filename(file.filename)
        filename = 'firmware.' + file.filename.rsplit('.', 1)[1]
        file.save(os.path.join(app.config['UPLOAD_FOLDER_FIRMWARE'], filename))
        return Response('ok')
    return Response(json.dumps({'error':'Invalid file format'}))

@app.route('/upload/shape', methods=['POST'])
def upload_shape():
    file = request.files['file']
    if file and allowed_file(file.filename, FLASK_ALLOWED_EXTENSIONS_IMAGES):
        file.save(os.path.join(app.config['UPLOAD_FOLDER_TEMP'], 'shape.png'))
        data = json.dumps({"config":"web_update_data","operation":"addshape","name":file.filename.rsplit('.', 1)[0]}) + "\r\n"
        process.stdin.write(data.encode('utf-8'))
        process.stdin.flush()
        return Response('ok')
    return Response(json.dumps({'error':'Invalid file format'}))

# upload image
@app.route('/upload/image', methods=['POST'])
def upload_image():
    file = request.files['file']
    if file and allowed_file(file.filename, FLASK_ALLOWED_EXTENSIONS_IMAGES):
        filename = secure_filename(file.filename)
        file.save(os.path.join(app.config['UPLOAD_FOLDER_IMAGE'], filename))
        # file_url = url_for('uploaded_file', filename=filename)
        return Response('ok')
    return Response(json.dumps({'error':'Invalid file format'}))

# !!! update firmware
@app.route('/update_firmware', methods=['POST'])
def update_firmware():
    filename = ''
    # check file
    try:
        files = os.listdir(app.config['UPLOAD_FOLDER_FIRMWARE'])
        if len(files) == 0:
            return Response(json.dumps({'error':'Firmware does not exist'}))
        filename = files[0]
    except:
        return Response(json.dumps({'error':'Firmware does not exist'}))

    logger.info('updating firmware %s' %filename)

    # move file
    try:
        ret = os.system('mv %s/%s /temp' %(app.config['UPLOAD_FOLDER_FIRMWARE'], filename))
        assert ret == 0
    except:
        return Response(traceback.print_exc())

    # unzip file
    try:
        filetype = filename.rsplit('.', 1)[1]
        if filetype == 'zip':
            ret = os.system('unzip /tmp/%s' %filename)
            assert ret == 0
        elif filetype == 'tar':
            ret = os.system('tar -xvf /tmp/%s' %filename)
            assert ret == 0
        elif filetype == 'gz':
            ret = os.system('tar -xf /tmp/%s' %filename)
            assert ret == 0
        ret = os.system('rm /tmp/%s' %filename)
        assert ret == 0
    except:
        return Response(traceback.print_exc())

    # run update script
    try:
        ret = os.system('python3 /tmp/*.py')
        assert ret == 0
    except:
        return Response(traceback.print_exc())

    return Response(json.dumps('updating'))

def checkFolder(path):
    if not os.path.exists(path):
        os.mkdir(path)

log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

if __name__ == '__main__':
    try:
        x = os.system('rm %s' %os.path.join(app.config['UPLOAD_FOLDER_TEMP'], '*'))
        x = os.system('cd ./bin')
        x = os.system('chmod +x *')
        x = os.system('cd ..')
        x = os.system('amixer set Mic 100db-')
    except:
        pass
    checkFolder('uploads')
    checkFolder(app.config['UPLOAD_FOLDER_TEMP'])
    checkFolder(app.config['UPLOAD_FOLDER_MODELS'])
    checkFolder(app.config['UPLOAD_FOLDER_FIRMWARE'])
    checkFolder(app.config['UPLOAD_FOLDER_IMAGE'])

    try:
        payload = sys.argv[1]
        if payload.isnumeric():
            server_pid = int(payload)
        else:
            raise Exception
    except:
        server_pid = 0
    logger.info('Server PID = %d, Core PID = %d' %(server_pid, os.getpid()))
    
    try:
        f = open('./static/description.json', 'r')
        s = f.read()
        f.close()
        description_doc = json.loads(s)
    except:
        logger.error('can not load description file.')

    try:
        f = open('./server_system_config.json', 'r')
        s = f.read()
        f.close()
        system_config_doc = json.loads(s)
    except:
        logger.warning('can not load config file, use default config.')
        system_config_doc = {'boot_func':'camera_stream'}
        f = open('./server_system_config.json', 'w')
        f.write(json.dumps(system_config_doc))
        f.close()

    net_models = checkNetModels()
    # if system_config_doc['boot_func'] is 'object_recognition':
    #     pipe_msg_rx_queue.put(json.dumps({'running':'Object Recongnition', 'running_model':system_config_doc['yolo_last_model_path'].rsplit('/', 1)[1], 'models':net_models, 'web':1}))


    ser = serial.Serial('/dev/ttyS1', 115200, timeout=0.5)
    protocol = serial.threaded.ReaderThread(ser, SerialRX)
    protocol.start()
    protocol.write("{\"msg\":\"server start.\"}\r\n".encode('utf-8'))
    thread_pipe_recv = Thread_PIPERecv(pipe_img_rx_queue)
    thread_pipe_recv.start()

    # Try to start the function specified on the command line
    try:
        process = subprocess.Popen(sys.argv[1], stdout=subprocess.PIPE, stdin=subprocess.PIPE, close_fds = True, bufsize = 1)
    except:
        logger.info('Wait for process to start')
        pass
        # Try to start the default function set by the user
        # try:
        #     process = subprocess.Popen('./bin/' + system_config_doc['boot_func'], stdout=subprocess.PIPE, stdin=subprocess.PIPE, close_fds = True, bufsize = 1)
        # except:
        #     process = subprocess.Popen('./bin/camera_stream', stdout=subprocess.PIPE, stdin=subprocess.PIPE, close_fds = True, bufsize = 1)

    # if client_is_connected is True:
    #     process.stdin.write("_{\"stream\":1}\r\n".encode('utf-8')) # '_' means parse by framework, otherwise parse by program
    #     process.stdin.flush()
    app.run('0.0.0.0', 80, threaded=True)