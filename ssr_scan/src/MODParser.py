import re
import numpy as np

#   MODParser.py
#   
#   Author: Logan Yoder
#
#   Contributors: Tom Geveke

def isMoveL(line):
    expr = re.compile('MoveL')
    return expr.search(line)

def getPose(line):
    expr = re.compile(r'\[(\d+\W*)+\]')
    pose = expr.search(line).group(0)
    return pose 

def getQuaternions(pose):
    expr = re.compile(r'(-{0,1}\d+\.{1}\d+,){3}(-{0,1}\d+\.{1}\d+)')

    return np.array(expr.search(pose).group(0).split(','),dtype=np.float32)

def getPosition(pose):
    expr = re.compile(r'(-{0,1}\d+\.{1}\d+,){2}(-{0,1}\d+\.{1}\d+)')

    return np.array(expr.search(pose).group(0).split(','),dtype=np.float32)

def read_file(filepath):
    with open(filepath, "r") as f:
        l = f.readline()
        
        output = []

        while len(l) > 0:
            if (isMoveL(l)):
                pose = getPose(l)
                position = getPosition(pose)
                quat = getQuaternions(pose)
                quat_right = [quat[1], quat[2], quat[3], quat[0]]

                output.append((position, quat_right))

            l = f.readline()
        return output
def main():
    filepath = input('Input the full path and filename of the .mod pose position file') # filepath = r"/home/logan/ROS/abb_ros/AugmentedRoboticFabrication/T_ROB/doTest_100_T_ROB1.mod"
    read_file(filepath)

if __name__ == '__main__':
    main() 


            

            

