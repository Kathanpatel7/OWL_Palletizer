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
    conSuc, sock = connectETController(robot_ip)

    if not conSuc:
        return None  # Connection failed

    try:
        suc, result, id = sendCMD(sock, "set_servo_status", {"status": 1})
        time.sleep(1)

        
        # Linear motion
        print("Moving to point:", points)
        suc, result, id = sendCMD(sock, "moveByLine", {
            "targetPos": points,
            "speed_type": 0,
            "speed": 200,
            "cond_type": 0,
            "cond_num": 7,
            "cond_value": 1})

        if not suc:
            print("Error in moveByLine:", result)
            return

        while True:
            # Get robot status
            suc, result, id = sendCMD(sock, "getRobotState")
            if result == 0:
                break

    except Exception as e:
        print("Error in moveRobotToPoint:", e)
    finally:
        disconnectETController(sock)




def getMasterPoint(robot_ip):
    # Function to get the master point from the user
    conSuc, sock = connectETController(robot_ip)
    input("Press enter after reaching desired pickup location ")
    if conSuc:
        suc , pickup , id=sendCMD(sock,"get_tcp_pose",{"coordinate_num": 0,"tool_num": 0})
        print(pickup)
    input("Press enter after reaching desired Transfer location ")
    if conSuc:
        suc , transfer , id=sendCMD(sock,"get_tcp_pose",{"coordinate_num": 0,"tool_num": 0})
        print(transfer)
    input("Press enter after reaching desired master location and Switch robot to Remote mode")
    if conSuc:
        suc , result , id=sendCMD(sock,"get_tcp_pose",{"coordinate_num": 0,"tool_num": 0})
        print(result)
    return pickup,transfer,result

def offsetPoses(master_point, target_poses):
    # Function to calculate offset poses based on the master point
    offset_poses = []
    for pose in target_poses:
        offset_pose = [master_point[i] + pose[i] if i < 2 else master_point[i] for i in range(6)]
        offset_poses.append(offset_pose)
        print(offset_pose)
    return offset_poses

if __name__ == "__main__":
    robot_ip = "192.168.1.200"

    # Get the master point and pickup point from the user
    pickup_point,transfer_point, master_point = getMasterPoint(robot_ip)

    # List of target poses
    target_poses = [
        [15.0, 20.0, 0],
        [45.0, 20.0, 0],
        [75.0, 20.0, 0],
        [105.0, 20.0, 0],
        [15.0, 60.0, 0],
        [45.0, 60.0, 0],
        [75.0, 60.0, 0],
        [105.0, 60.0, 0],
        [15.0, 100.0, 0],
        [45.0, 100.0, 0],
        [75.0, 100.0, 0],
        [105.0, 100.0, 0]
    ]

    # Calculate offset poses based on the master point
    offset_poses = offsetPoses(master_point, target_poses)
    con_success, sock = connectETController(robot_ip)

    if con_success:
        try:
            for pose in offset_poses:
                # Move to the pickup point first
                jointlist_pick = inverseKinematicSolver(robot_ip, [pickup_point])
                moveRobotToPoint(robot_ip, jointlist_pick[0])
                
                # Move to the pickup point first
                jointlist_transfer = inverseKinematicSolver(robot_ip, [transfer_point])
                moveRobotToPoint(robot_ip, jointlist_transfer[0])

                # Then move to the current target pose
                jointlist_place = inverseKinematicSolver(robot_ip, [pose])  # Pass pose as a list
                moveRobotToPoint(robot_ip, jointlist_place[0])  # Extract the first element from the list
                time.sleep(1)   # Optional delay between movements

            print("All poses reached successfully.")

            # Perform other actions if needed
        except Exception as e:
            print("Error:", e)
        finally:
            disconnectETController(sock)
    else:
        print("Connection to the robot failed.")
