import time
import os
import asyncio
import websockets
import json
from robot.RobotInfo import RobotInfo
from robot.RobotMove import RobotMove
from robot.RobotServos import ServoCtrl
from robot.RobotLight import RobotLight
from robot.functions import Functions
import webApp.app as app
from settings import *

# Global variables
functionMode = 0
speed_set = 100
rad = 0.5
turnWiggle = 60
modeSelect = 'PT'
direction_command = 'no'
turn_command = 'no'

try:
    RL = RobotLight()
    RL.start()
    RL.breath(70, 70, 255)
    robot_info = RobotInfo()
    robot_move = RobotMove()
    sc = ServoCtrl()
    sc.start()
    robot_move.start()  # Start the thread
    fuc = Functions()
    fuc.start()
    flask_app = app.webapp()
    flask_app.startthread()
except:
    print('Use "sudo pip3 install rpi_ws281x" to install WS_281x package\n"sudo pip3 install rpi_ws281x"')
    pass

curpath = os.path.realpath(__file__)
thisPath = "/" + os.path.dirname(curpath)

def replace_num(initial, new_num):
    """
    Replace a specific setting in the settings.py file.
    
    Args:
        initial (str): The setting key to replace.
        new_num (int/float): The new value for the setting.
    """
    newline = ""
    str_num = str(new_num)
    with open(thisPath + "/settings.py", "r") as f:
        for line in f.readlines():
            if line.find(initial) == 0:
                line = initial + "%s" % (str_num + "\n")
            newline += line
    with open(thisPath + "/settings.py", "w") as f:
        f.writelines(newline)

def functionSelect(command_input, response):
    """
    Handle function selection based on the command input.
    
    Args:
        command_input (str): The command received.
        response (dict): The response dictionary to be updated.
    """
    global functionMode
    print(command_input,response,modeSelect)
    if 'scan' == command_input:
        if modeSelect == 'PT':
            radar_send = sc.radarScan()
            print(radar_send)
            response['title'] = 'scanResult'
            response['data'] = radar_send
            time.sleep(0.3)

    elif 'findColor' == command_input:
        if modeSelect == 'PT':
            flask_app.modeselect('findColor')

    elif 'motionGet' == command_input:
        flask_app.modeselect('watchDog')

    elif 'stopCV' == command_input:
        flask_app.modeselect('none')
        RL.switch(1, 0)
        RL.switch(2, 0)
        RL.switch(3, 0)
        robot_move.motor_stop()

    elif 'KD' == command_input:
        sc.moveInit()
        fuc.keepDistance()
        RL.police()

    elif 'automaticOff' == command_input:
        flask_app.modeselect('none')
        RL.switch(1, 0)
        RL.switch(2, 0)
        RL.switch(3, 0)
        robot_move.motor_stop()

    elif 'automatic' == command_input:
        flask_app.modeselect('automatic')

    elif 'automaticOff' == command_input:
        fuc.pause()
        robot_move.motor_stop()
        time.sleep(0.2)
        robot_move.motor_stop()

    elif 'trackLine' == command_input:
        sc.moveInit()
        fuc.trackLine()

    elif 'trackLineOff' == command_input:
        fuc.pause()
        robot_move.motor_stop()

    elif 'steadyCamera' == command_input:
        fuc.steady(sc.lastPos[2])

    elif 'steadyCameraOff' == command_input:
        fuc.pause()
        robot_move.motor_stop()

    elif 'speech' == command_input:
        RL.both_off()
        fuc.speech()

    elif 'speechOff' == command_input:
        RL.both_off()
        fuc.pause()
        robot_move.motor_stop()
        time.sleep(0.3)
        robot_move.motor_stop()

def switchCtrl(command_input, response):
    """
    Handle switch control based on the command input.
    
    Args:
        command_input (str): The command received.
        response (dict): The response dictionary to be updated.
    """
    if 'Switch_1_on' in command_input:
        RL.switch(1, 1)

    elif 'Switch_1_off' in command_input:
        RL.switch(1, 0)

    elif 'Switch_2_on' in command_input:
        RL.switch(2, 1)

    elif 'Switch_2_off' in command_input:
        RL.switch(2, 0)

    elif 'Switch_3_on' in command_input:
        RL.switch(3, 1)

    elif 'Switch_3_off' in command_input:
        RL.switch(3, 0)

def robotCtrl(command_input, response):
    """
    Handle robot control based on the command input.
    
    Args:
        command_input (str): The command received.
        response (dict): The response dictionary to be updated.
    """
    global direction_command, turn_command
    if 'forward' == command_input:
        direction_command = 'forward'
        robot_move.move(speed_set, 'forward')
        RL.both_on()

    elif 'backward' == command_input:
        direction_command = 'backward'
        robot_move.move(speed_set, 'backward')
        RL.red()

    elif 'DS' in command_input:
        direction_command = 'no'
        robot_move.motor_stop()
        if turn_command == 'left':
            RL.both_off()
            RL.turn_left()
        elif turn_command == 'right':
            RL.both_off()
            RL.turn_right()
        elif turn_command == 'no':
            RL.both_off()

    elif 'left' == command_input:
        turn_command = 'left'
        sc.moveAngle(2, 30)
        RL.both_off()
        RL.turn_left()

    elif 'right' == command_input:
        turn_command = 'right'
        sc.moveAngle(2, -30)
        RL.both_off()
        RL.turn_right()

    elif 'TS' in command_input:
        turn_command = 'no'
        sc.moveAngle(2, 0)
        if direction_command == 'forward':
            RL.both_on()
        elif direction_command == 'backward':
            RL.both_off()
            RL.red()
        elif direction_command == 'no':
            RL.both_off()

    elif 'lookleft' == command_input:
        sc.singleServo(1, 1, 7)

    elif 'lookright' == command_input:
        sc.singleServo(1, -1, 7)

    elif 'LRstop' in command_input:
        sc.stopWiggle()

    elif 'up' == command_input:
        sc.singleServo(0, 1, 7)

    elif 'down' == command_input:
        sc.singleServo(0, -1, 7)

    elif 'UDstop' in command_input:
        sc.stopWiggle()

    elif 'home' == command_input:
        sc.moveServoInit([init_pwm1])
        sc.moveServoInit([init_pwm0])

def configPWM(command_input, response):
    """
    Handle PWM configuration based on the command input.
    
    Args:
        command_input (str): The command received.
        response (dict): The response dictionary to be updated.
    """
    global init_pwm0, init_pwm1, init_pwm2, init_pwm3, init_pwm4

    if 'SiLeft' in command_input:
        numServo = int(command_input[7:])
        if numServo == 0:
            init_pwm0 -= 1
            sc.setPWM(0, init_pwm0)
        elif numServo == 1:
            init_pwm1 -= 1
            sc.setPWM(1, init_pwm1)
        elif numServo == 2:
            init_pwm2 -= 1
            sc.setPWM(2, init_pwm2)

    if 'SiRight' in command_input:
        numServo = int(command_input[8:])
        if numServo == 0:
            init_pwm0 += 1
            sc.setPWM(0, init_pwm0)
        elif numServo == 1:
            init_pwm1 += 1
            sc.setPWM(1, init_pwm1)
        elif numServo == 2:
            init_pwm2 += 1
            sc.setPWM(2, init_pwm2)

    if 'PWMMS' in command_input:
        numServo = int(command_input[6:])
        if numServo == 0:
            sc.initConfig(0, init_pwm0, 1)
            replace_num('init_pwm0 = ', init_pwm0)
        elif numServo == 1:
            sc.initConfig(1, init_pwm1, 1)
            replace_num('init_pwm1 = ', init_pwm1)
        elif numServo == 2:
            sc.initConfig(2, init_pwm2, 2)
            replace_num('init_pwm2 = ', init_pwm2)

    if 'PWMINIT' == command_input:
        print(init_pwm1)
        sc.moveInit()

    elif 'PWMD' == command_input:
        init_pwm0, init_pwm1, init_pwm2, init_pwm3, init_pwm4 = 285, 285, 285, 285, 285
        sc.initConfig(0, 285, 1)
        replace_num('init_pwm0 = ', 285)
        sc.initConfig(1, 285, 1)
        replace_num('init_pwm1 = ', 285)
        sc.initConfig(2, 285, 1)
        replace_num('init_pwm2 = ', 285)

def update_code():
    """
    Update local code to be consistent with the remote repository.
    """
    projectPath = thisPath[:-7]
    with open(f'{projectPath}/config.json', 'r') as f1:
        config = json.load(f1)
        if not config['production']:
            print('Update code')
            # Force overwriting local code
            if os.system(f'cd {projectPath} && sudo git fetch --all && sudo git reset --hard origin/master && sudo git pull') == 0:
                print('Update successfully')
                print('Restarting...')
                os.system('sudo reboot')

async def check_permit(websocket):
    """
    Check user credentials for websocket connection.
    
    Args:
        websocket (WebSocket): The websocket connection.
    
    Returns:
        bool: True if credentials are correct, False otherwise.
    """
    while True:
        recv_str = await websocket.recv()
        cred_dict = recv_str.split(":")
        if cred_dict[0] == "admin" and cred_dict[1] == "123456":
            response_str = "congratulation, you have connect with server\r\nnow, you can do something else"
            await websocket.send(response_str)
            return True
        else:
            response_str = "sorry, the username or password is wrong, please submit again"
            await websocket.send(response_str)

async def recv_msg(websocket):
    """
    Receive and process messages from the websocket.
    
    Args:
        websocket (WebSocket): The websocket connection.
    """
    global speed_set, modeSelect
    direction_command = 'no'
    turn_command = 'no'

    while True: 
        response = {
            'status' : 'ok',
            'title' : '',
            'data' : None
        }

        data = ''
        data = await websocket.recv()
        try:
            data = json.loads(data)
        except Exception as e:
            print('not A JSON')

        if not data:
            continue

        if isinstance(data, str):
            robotCtrl(data, response)
            switchCtrl(data, response)
            functionSelect(data, response)
            configPWM(data, response)

            if 'get_info' == data:
                response['title'] = 'get_info'
                response['data'] = [robot_info.get_cpu_temp(), robot_info.get_cpu_usage(), robot_info.get_ram_usage()]

            if 'wsB' in data:
                try:
                    set_B = data.split()
                    speed_set = int(set_B[1])
                except:
                    pass

            # CVFL
            elif 'CVFL' == data:
                flask_app.modeselect('findlineCV')

            elif 'CVFLColorSet' in data:
                color = int(data.split()[1])
                app.camera.colorSet(color)

            elif 'CVFLL1' == data:
                pos = int(data.split()[1])
                app.camera.linePosSet_1(pos)

            elif 'CVFLL2' == data:
                pos = int(data.split()[1])
                app.camera.linePosSet_2(pos)

            elif 'CVFLSP' in data:
                err = int(data.split()[1])
                app.camera.errorSet(err)

        elif isinstance(data, dict):
            if data['title'] == "findColorSet":
                color = data['data']
                app.colorFindSet(color[0], color[1], color[2])

        print(data)
        response = json.dumps(response)
        await websocket.send(response)

async def main_logic(websocket, path):
    """
    Main logic for handling websocket connections.
    
    Args:
        websocket (WebSocket): The websocket connection.
        path (str): The URL path of the websocket connection.
    """
    await check_permit(websocket)
    await recv_msg(websocket)

def cleanup():
    """
    Stop all systems and clean up.
    """
    robot_move.terminate()
    RL.pause()
    sc.terminate()
    robot_move.join()
    sc.join()

def run():


    RL.set_all_switch_off()

    while 1:
        try:
            # Start server, waiting for client
            start_server = websockets.serve(main_logic, '0.0.0.0', 8888)
            asyncio.get_event_loop().run_until_complete(start_server)
            print('waiting for connection...')
            # print('...connected from :', addr)
            break
        except Exception as e:
            print(e)
            RL.setColor(0, 0, 0)

        try:
            RL.setColor(0, 80, 255)
        except:
            pass

    try:
        asyncio.get_event_loop().run_forever()
    except Exception as e:
        print(e)
        RL.setColor(0, 0, 0)
        cleanup()  # Ensure the thread is properly terminated
