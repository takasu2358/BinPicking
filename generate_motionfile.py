import time

start_time = time.time()

initial_pose = "0 2 JOINT_ABS 0 0 0 -10 -25.7 -127.5 0 0 0 23 -25.7 -133.7 -7 0 0 0 0 0 0"

def GenerateMotionfile(filepath, grasp_pose, goal_position, action):
    x, y, z, yaw = grasp_pose
    goal_x, goal_y, goal_z = goal_position

    if z < 0.020:
        print("Warning: z-coodinate is too low!")
        z = 0.020
    if z <= 0.1 and z > 0.5:
        z -= 0.02
    if z > 0.1:
        z -= 0.03
    z += 0.013


    if z < 0.03 or z > 0.4:
        flag = 0
        print("Error: z-coodinate is out of safe range!")
    else:
        flag = 1
        if action == 0:
            print("action is 0")
            fp = open(filepath, 'wt')
            print("0 1 LHAND_JNT_OPEN",file=fp)
            print("0 3 LARM_XYZ_ABS {:.3f} {:.3f} 0.3 -180 -90 {:.3f}".format(x,y,33.0+yaw),file=fp)
            print("0 1 LARM_XYZ_REL 0 0 {:.3f} 0 0 0".format(z - 0.3 + 0.05),file=fp)
            print("0 1.5 LARM_XYZ_REL 0 0 -0.05 0 0 0",file=fp)
            print("0 0.5 LHAND_JNT_CLOSE 0 0 0 0 0 0",file=fp)
            print("0 1.5 LARM_XYZ_REL 0 0 0.45 0 0 0",file=fp)
            # place
            print("0 1.5 LARM_XYZ_ABS {:.3f} {:.3f} {:.3f} -180 -90 145".format(goal_x, goal_y, goal_z-0.3),file=fp)
            print("0 1.5 LARM_XYZ_REL 0 0 -0.10 0 0 0",file=fp)
            print("0 0.5 LHAND_JNT_OPEN",file=fp)
            print(initial_pose,file=fp)
            fp.close()

        elif action == 1:
            fp = open(filepath, 'wt')
            print("0 0.5 LHAND_JNT_OPEN",file=fp)
            print("0 1 LARM_XYZ_ABS {:.3f} {:.3f} 0.3 -180 -90 {:.3f}".format(x,y,33.0+yaw),file=fp)
            print("0 1 LARM_XYZ_REL 0 0 -0.15 0 0 0",file=fp)
            print("0 1.5 LARM_XYZ_REL 0 0 {:.3f} 0 0 0".format(z - 0.15),file=fp)
            print("0 0.5 LHAND_JNT_CLOSE 0 0 0 0 0 0",file=fp)
            print("0 1.5 LARM_XYZ_ABS {:.3f} {:.3f} 0.3 -180 -90 {:.3f}".format(x,y,33.0+yaw),file=fp)

            # withdraw
            print("0 1.5 LARM_XYZ_ABS {:.3f} {:.3f} 0.3 -180 -90 {:.3f}".format(x,y-0.15,33.0+yaw),file=fp)

            # place
            print("0 1 LARM_XYZ_ABS {:.3f} {:.3f} {:.3f} -180 -90 145".format(goal_x, goal_y, goal_z-0.3),file=fp)
            print("0 0.5 LARM_XYZ_REL 0 0 -0.10 0 0 0",file=fp)
            print("0 0.5 LHAND_JNT_OPEN",file=fp)
                
            print(initial_pose, file=fp)
            fp.close()
        else:
            flag = 0
            print("Error: Action nuber must be set to 0 or 1")

    elapsed_time = time.time() - start_time
    print("Run time costs to generate motionfile is {}\n".format(elapsed_time))
    
    return flag