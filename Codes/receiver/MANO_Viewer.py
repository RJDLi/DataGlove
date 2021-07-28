#!/usr/bin/python2.7
"""
@file:  MANO_Viewer.py
@author: Jiangfan Li
@mail:  rdemezerl@gmail.com
@description: Visualize Hand Mesh, based on pose data.
    @Input:
        - Pose
        - MANO model
            -* todo unify left and right
    @Output:
        Visulization using MeshViewer.
@detail:
    - Pose
        - finger(from thumb)
        - joint(p-m-d)
        - Euler(r-p-y)
"""
import sys, os.path
rfp = os.path.dirname(sys.argv[0])
sys.path.append(rfp+'/mano_v1_2')

DEFAULT_MODEL_PATH_L = rfp+'/mano_v1_2/models/MANO_LEFT.pkl'
DEFAULT_MODEL_PATH_R = rfp+'/mano_v1_2/models/MANO_RIGHT.pkl'
DEFAULT_POSE_PATH_L = rfp+'/mano_poses_v1_0/handsOnly_REGISTRATIONS_r_lm___POSES___L.npy'
DEFAULT_POSE_PATH_R = rfp+'/mano_poses_v1_0/handsOnly_REGISTRATIONS_r_lm___POSES___R.npy'
DEFAULT_PKL_FILE = '01_01r.pkl'
DEFAULT_PKL_DIR = rfp+'/mano_poses_v1_0/handsOnly_REGISTRATIONS_r_lm___POSES/'
JOINT_RADIUS = .005

import numpy as np
import pickle
from numpy.core.defchararray import lower, upper
from numpy.core.fromnumeric import shape
# 3D viewer
# depends on 
from psbody.mesh import Mesh, MeshViewers
from psbody.mesh.sphere import Sphere

# loader for the full-pose space
from webuser.serialization import load_model

"""
quat: wxyz
order:
BigAI: 16*3 or 16*4
    5   8   11  14
    |   |   |   |
    4   7   10  13
    |   |   |   |
    3   6   9  12
2
  1
    0        15
MANO: 48 = 16*3
- 0-2:      root                0
- 3-11:     finger 1 - Index   123
- 12-20:    finger 2 - Middle  456
- 21-29:    finger 4 - Pinky   789
- 30-38:    finger 3 - Ring    101112
- 39-47:    finger 0 - Thumb   131415
"""
def styleMANO(data, order = 'BigAI'): 
    # orientation type
    shape = data.shape
    manodata = np.zeros((16, 3))
    if len(shape) == 1:
        if shape[0]!=48:
            raise ValueError('your input shape is', shape)
    elif len(shape) == 2:
        if data.shape[1] == 4:
            def quat2euler(quat):
                euler = np.zeros((3))
                euler[0] = np.arctan2(2*(quat[0]*quat[1]+quat[2]*quat[3]), 1-2*(quat[1]*quat[1]+quat[2]*quat[2]))
                euler[1] = np.arcsin(2*(quat[0]*quat[2] - quat[3]*quat[1]))
                euler[2] = np.arctan2(2*(quat[0]*quat[3]+quat[1]*quat[2]), 1-2*(quat[3]*quat[3]+quat[2]*quat[2]))
                return euler
            for i in range(shape[0]):
                manodata[i] = quat2euler(data[i])
        elif not data.shape[1] == 3: # error
            raise ValueError('can only handle euler or quaternion. your input shape is', shape)
        # order
        if upper(order) == 'BIGAI':
            ori = np.copy(manodata)
            manodata[1:4, :] = ori[3:6, :]
            manodata[4:7, :] = ori[6:9, :]
            manodata[7:10, :] = ori[12:15,:]
            manodata[10:13, :] = ori[9:12,:]
            manodata[13:16,:] = ori[0:3,:]
            manodata[0, :] = ori[15, :]
        # flat
        manodata = np.reshape(manodata,-1)
    else:
        raise ValueError('your input shape is', shape)
    return manodata

WINDOWS_UNIT = 800
class MANO_Viewer:
    def __init__(self, dir = 'right',modelpath = None, pklpath = None, mode='single_mesh'):
        """
        @dir: 'left' or 'right'
        @mode:
            - 'single_mesh' - show mesh only
            - 'single_mesh_joints' - show joints
            - 'rest_mesh' - show rest pose
            - 'rest_mesh_joints'
        """
        # model
        if lower(dir) == 'left':
            self.dir = 'left'
        else:
            self.dir = 'right'
        if modelpath is None:
            if self.dir == 'left':
                modelpath = DEFAULT_MODEL_PATH_L
            else:
                modelpath = DEFAULT_MODEL_PATH_R
        self.model = load_model(fname_or_dict=modelpath)
        # pkl
        if pklpath is None:
            pklpath = DEFAULT_PKL_DIR+DEFAULT_PKL_FILE
        with open(pklpath, 'rb') as f:
            self.pkl = pickle.load(f)
        self.model.betas[:] = self.pkl['betas']
        self.model.trans[:] = self.pkl['trans']
        # view
        self.mode = mode
        if self.mode == 'single_mesh':
            self.mvs = MeshViewers(window_width=WINDOWS_UNIT, window_height=WINDOWS_UNIT, shape=[1,1])
        elif self.mode == 'single_mesh_joints':
            self.mvs = MeshViewers(window_width=WINDOWS_UNIT, window_height=WINDOWS_UNIT*2, shape=[1,2])
        elif self.mode == 'rest_mesh':
            self.mvs = MeshViewers(window_width=WINDOWS_UNIT*2, window_height=WINDOWS_UNIT, shape=[2,1])
        elif self.mode == 'rest_mesh_joints':
            self.mvs = MeshViewers(window_width=WINDOWS_UNIT*2, window_height=WINDOWS_UNIT*2, shape=[2,2])

    def _get_pose_valid(self, seed=None, path=None, world=np.zeros((3))):
        """
        return a valid pose from pose data
        - use default data if not provided
        - use random pose if not specified
        """
        if seed is None:
            seed = np.random.randint(1554) # remind to change this if using other database
        if path is None:
            if self.dir == 'left':
                path = DEFAULT_POSE_PATH_L
            else:
                path = DEFAULT_POSE_PATH_R
        poses = np.load(path)
        pose = np.concatenate((world, poses[seed]))
        return pose

    def _get_mesh_pose(self, pose):
        self.model.pose[:] = pose
        return Mesh(v=self.model.r, f=self.model.f)
    
    def _get_mesh_pose_joint(self, pose):
        self.model.pose[:] = pose
        return Mesh(v=self.model.r), [Sphere(np.array(jointPos), JOINT_RADIUS).to_mesh(
                    np.eye(3)[0 if jointID == 0 else 1]) 
                for jointID, jointPos in enumerate(self.model.J_transformed)]

    def _get_rest_pose(self):
        return np.zeros((48))
    
    def _get_rest_mesh(self):
        if self.rest_mesh is None:
            self.rest_mesh = self._get_mesh_pose(self._get_rest_pose())
        return self.rest_mesh

    def _get_rest_mesh_joint(self):
        if self.rest_mesh_joint is None:
            self.rest_mesh_joint, self.rest_joint = self._get_mesh_pose_joint(self._get_rest_pose())
        return self.rest_mesh_joint, self.rest_joint


    def view(self, pose):
        mesh = self._get_mesh_pose(pose)
        if self.mode == 'single_mesh':
            self.mvs[0][0].set_static_meshes([mesh], blocking = True)
        elif self.mode == 'single_mesh_joints':
            self.mvs[1][0].set_static_meshes([mesh], blocking = True)
            mesh.f = []
            joints_R = [Sphere(np.array(jointPos), JOINT_RADIUS).to_mesh(
                    np.eye(3)[0 if jointID == 0 else 1]) 
                for jointID, jointPos in enumerate(self.model.J_transformed)]
            self.mvs[1][0].set_static_meshes([mesh] + joints_R, blocking = True)
        elif self.mode == 'rest_mesh':
            self.mvs[0][0].set_static_meshes([mesh], blocking = True)
            self.mvs[0][1].set_static_meshes([self._get_rest_mesh()], blocking = True)
        elif self.mode == 'rest_mesh_joints':
            self.mvs[1][0].set_static_meshes([mesh], blocking=True)
            self.mvs[1][1].set_static_meshes([self._get_rest_mesh()], blocking=True)
            meshj, j = self._get_mesh_pose_joint(pose)
            restmesh, restj = self._get_rest_mesh_joint()
            self.mvs[0][0].set_static_meshes([meshj] + j, blocking=True)
            self.mvs[0][1].set_static_meshes([restmesh] + restj, blocking=True)


if __name__ == '__main__':
    mViewer = MANO_Viewer()

    print("Showing data from test npy")
    path = '/home/liuhx/Documents/data_glove/test'
    quats = np.load(path+'.npy')
    
    with open(path+'.txt', 'r') as f:
        timestamps = [(int)(l.strip()) for l in f]
        
    for i in range(len(timestamps)):
        print(timestamps[i])
        p = styleMANO(quats[i])
        mViewer.view(p)
        raw_input('Press Enter to Continue ...')

    """
    print("Showing random poses")
    while True:
        mViewer.view(mViewer._get_pose_valid())
        raw_input('Press Enter to Continue ...')
    """