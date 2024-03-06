import socket
import json
import time

def connectETController(ip, port=8055):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((ip, port))
        return (True, sock)
    except Exception as e:
        sock.close()
        return (False, sock)  # Return the socket even in case of failure

def disconnectETController(sock):
    if sock:
        sock.close()
        sock = None
    else:
        sock = None

def sendCMD(sock, cmd, params=None, id=1):
    if not params:
        params = []
    else:
        params = json.dumps(params)
    sendStr = "{{\"method\":\"{0}\",\"params\":{1},\"jsonrpc\":\"2.0\",\"id\":{2}}}".format(cmd, params, id) + "\n"
    try:
        sock.sendall(bytes(sendStr, "utf-8"))
        ret = sock.recv(1024)
        jdata = json.loads(str(ret, "utf-8"))
        if "result" in jdata.keys():
            return (True, json.loads(jdata["result"]), jdata["id"])
        elif "error" in jdata.keys():
            return (False, jdata["error"], jdata["id"])
        else:
            return (False, None, None)
    except Exception as e:
        return (False, None, None)

def inverseKinematicSolver(robot_ip, target_poses):
    P000 = [0, -90, 90, -90, 90, 0]

    conSuc,sock=connectETController( robot_ip )
    
    if not conSuc:
        return None  # Connection failed
    
    try:
        joint_list = []
        
        for target_pose in target_poses:
            # Get the current pose information of the robot
            success, result, _ = sendCMD(sock, "get_tcp_pose")
            
            if success:
                # Inverse solution function 2.0, with reference point position
                success, result, _ = sendCMD(sock, "inverseKinematic", {"targetPose": target_pose, "referencePos": P000})
                
                if success:
                    joint_list.append(result)
                else:
                    joint_list.append(None)  # Inverse kinematics failed for this pose
            else:
                joint_list.append(None)  # Unable to get current pose for this pose
        
        return joint_list
    finally:
        disconnectETController(sock)
def moveRobotToPoint(robot_ip, points):
    conSuc,sock=connectETController( robot_ip )
    
    if not conSuc:
        return None  # Connection failed
    
    try:
        # Set the servo status of the robotic arm to ON
        suc, result, id = sendCMD(sock, "set_servo_status", {"status": 1})
        time.sleep(1)

        for point in points:
            # Linear motion
            suc, result, id = sendCMD(sock, "moveByLine", {
                "targetPos": point,
                "speed_type": 0,
                "speed": 50,
                "cond_type": 0,
                "cond_num": 7,
                "cond_value": 1
            })

        while True:
            # Get robot status
            suc, result, id = sendCMD(sock, "getRobotState")
            if result == 0:
                break

    finally:
        disconnectETController(sock)
if __name__ == "__main__":
    # Example usage with a list of poses
    robot_ip = "192.168.1.200"
    
    # List of target poses
    target_poses = [
        [70, 370, 300, -0.785398, 0, -1.57]
        
    ]
    
    con_success, sock = connectETController(robot_ip)
    
    if con_success:
        jointlist = inverseKinematicSolver(robot_ip, target_poses)
        moveRobotToPoint(robot_ip, jointlist)
        
        if jointlist is not None:
            print("Inverse kinematics results:", jointlist)
        else:
            print("Inverse kinematics failed for one or more poses.")
    else:
        print("Connection to the robot failed.")