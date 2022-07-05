import numpy as np
import sys, os

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kinematic.paint_kinematics import PaintKinematicsKDL

def main():
    urdf_path = "conf/vander_bot_v2.urdf"
    pk = PaintKinematicsKDL()
    pk.load_URDF(urdf_file_path = urdf_path)
    start_pose = np.array([0, 0])
    cp_pose = np.array([0.6, -0.])

    new_jpos = pk.ik_pose(new_pos = cp_pose, old_pos = start_pose)
    print(new_jpos)
    fk_pose = pk.fk_pose(q = np.array(new_jpos))
    # new_jpos = np.array([np.pi/4, np.pi/2])
    print(fk_pose)




if __name__=="__main__":
    main()
