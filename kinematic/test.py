from PaintKinematics import PaintKinematicsKDL
import numpy as np

def main():
    urdf_path = "./robot_config.urdf"
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
