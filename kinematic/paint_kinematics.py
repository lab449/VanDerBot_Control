import numpy as np
from urdfpy import URDF
import PyKDL as kdl
from kdl_parser.kdl_parser_py.kdl_parser_py import urdf


class PaintKinematicsKDL:
    '''
    Class with kinematics of 2 doff painting robot

    '''
    def __init__(self, urdf_file_path = None):
        '''
        If urdf_file_path is not None, then read URDF file and initialize KDL
        kinematics.

        Optional keyword arguments:
        urdf_file_path:  path to URDF file
        '''
        self.__urdf_file_path = urdf_file_path
        self.__z_offset = 0
        self.__robot_urdf = None
    def __initialize_kdl(self):
        '''
        initialize kdl kenimatics solvers
        '''

        urdf_file = open(self.__urdf_file_path, 'r')
        urdf_str = urdf_file.read()
        urdf_file.close()
        (ok, self.__tree) = urdf.treeFromString(urdf_str)

        if not ok:
            raise RuntimeError('Tree is not valid')

        self.__chain = self.__tree.getChain(
            self.__robot_urdf.links[0].name,
            self.__robot_urdf.links[-1].name
        )
        self.__fk_posesolver    = kdl.ChainFkSolverPos_recursive(self.__chain)
        self.__ik_posesolver    = kdl.ChainIkSolverPos_LMA(
            self.__chain
        )

        self.__z_offset = self.__robot_urdf.joints[0].origin[2,3] \
                        + self.__robot_urdf.joints[1].origin[2,3]



    def ik_pose(self, new_pos: np.ndarray, old_pos: np.ndarray):
        pos = kdl.Vector(new_pos[0],new_pos[1],self.__z_offset)
        goal = kdl.Frame(pos)
        result = kdl.JntArray(len(self.__robot_urdf.joints) - 1)
        joints_angles = to_jnt_array(old_pos)

        self.__ik_posesolver.CartToJnt(joints_angles, goal, result)
        return np.array(list(result))

    def fk_pose(self, q: np.ndarray):
        end_frame = kdl.Frame()
        self.__fk_posesolver.JntToCart(to_jnt_array(q),end_frame)
        pos = end_frame.p
        return np.array([pos[0], pos[1], pos[2]])


    def load_URDF(self, urdf_file_path = None):
        '''
        Read URDF file and initialize KDL kinematics.

        Optional keyword arguments:
        urdf_file_path:  path to URDF file
        '''
        if not isinstance(urdf_file_path,type(None)):
            self.__urdf_file_path = urdf_file_path

        if isinstance(self.__urdf_file_path,type(None)):
            exit('Nothing to load...')
        self.__urdf_file_path = urdf_file_path
        self.__robot_urdf = URDF.load(self.urdf)
        self.__initialize_kdl()

    @property
    def urdf(self):
        return self.__urdf_file_path

def to_np_matrix(kdl_data, size: int) -> np.ndarray:
    if isinstance(kdl_data, (kdl.JntSpaceInertiaMatrix, kdl.Jacobian, kdl.Rotation)):
        out = np.zeros((size, size))
        for i in range(0, size):
            for j in range(0, size):
                out[i,j] = kdl_data[i,j]
        return out
    elif isinstance(kdl_data, (kdl.JntArray, kdl.JntArrayVel)):
        out = np.zeros(size)
        for i in range(0, size):
            out[i] = kdl_data[i]
        return out
    else:
        out = np.zeros(size)
        for i in range(0, size):
            out[i] = kdl_data[i]
        return out

def to_jnt_array(np_vector: np.ndarray)-> kdl.JntArray:
    size = np_vector.shape[0]
    ja = kdl.JntArray(size)
    for i in range(0, size):
        ja[i] = np_vector[i]
    return ja
