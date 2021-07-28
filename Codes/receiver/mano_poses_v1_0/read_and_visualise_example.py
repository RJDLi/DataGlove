import numpy as np
import pickle
# 3D viewer
# depends on 
from psbody.mesh import Mesh, MeshViewers
from psbody.mesh.sphere import Sphere


# replace these paths if you did not follow the exact instructions in the READ_ME.txt
load_model_PATH_L = '/home/liuhx/Documents/data_glove/mano_v1_2/models/MANO_LEFT.pkl'
load_model_PATH_R = '/home/liuhx/Documents/data_glove/mano_v1_2/models/MANO_RIGHT.pkl'

# loader for the full-pose space
from webuser.serialization import load_model
model_L = load_model(fname_or_dict=load_model_PATH_L)
model_R = load_model(fname_or_dict=load_model_PATH_R)
# # loader for the low-D pose space with ncomps components
# from webuser.smpl_handpca_wrapper_HAND_only import load_model
# model_L = load_model(fname_or_dict=load_model_PATH_L, ncomps=ncomps, flat_hand_mean=False)
# model_R = load_model(fname_or_dict=load_model_PATH_R, ncomps=ncomps, flat_hand_mean=False)


def load_and_show(pathPKLL, idx):

    pathMESH = pathPKLL.replace('.pkl', '.ply')

    print pathPKLL
    print pathMESH

    poseR = np.load('/home/liuhx/Documents/data_glove/MANO_data/manoposesv10/mano_poses_v1_0/handsOnly_REGISTRATIONS_r_lm___POSES___R.npy')
    poseL = np.load('/home/liuhx/Documents/data_glove/MANO_data/manoposesv10/mano_poses_v1_0/handsOnly_REGISTRATIONS_r_lm___POSES___L.npy')
    # print poseR.shape  # (1554, 45)
    # print poseL.shape  # (1554, 45)

    with open(pathPKLL, 'rb') as f:
        pkl = pickle.load(f)
    # print pkl.keys()  
    # print pkl.values()
    # print pkl['pose'].shape
    # print pkl['betas'].shape
    # print pkl['v'].shape
    # ['J_transformed', 'pose', 'betas', 'v', 'trans', 'ncomps', 'model_name']
    # 16*3,             45,     10,     778*3,  3,       0(=full),  
    # From CV,        , IMU,    data,   data,   000     0

    mesh = Mesh(filename=pathMESH)

    if 'LEFT' in pkl['model_name']:
        model = model_L
    else:
        model = model_R

    model.pose[:] = pkl['pose']
    model.betas[:] = pkl['betas']
    model.trans[:] = pkl['trans']
    print '~~~~~~~~~~~~~~~~ vertices'
    print 'test =', np.allclose(model.r, mesh.v)        # model.r == computed by MANO
    print 'test =', np.allclose(model.r, pkl['v'])      # mesh.v  == registration data
    print 'test =', np.allclose(pkl['v'], mesh.v)       # pkl     == 
    print '~~~~~~~~~~~~~~~~ J_transformed'
    print 'test =', np.allclose(model.J_transformed, pkl['J_transformed'])
    print '~~~~~~~~~~~~~~~~ pose'
    print 'test =', np.allclose(model.pose, pkl['pose'])
    print 'test =', np.allclose(model.pose[3:], poseR[idx])


    # mirror poses Right-->Left
    def right2left_aangle(right_aangle):
        from cv2 import Rodrigues as Rod
        xmirror = np.asarray([[-1, 0, 0], [0, 1, 0], [0, 0, 1]])
        return(Rod((xmirror.dot(Rod(right_aangle)[0])).dot(xmirror))[0].ravel())
    def fullrightpose2leftpose(rightpose):
        return(np.asarray([right2left_aangle(right_aangle)
                           for right_aangle in rightpose.reshape(-1, 3)]).ravel())
    leftp1 = fullrightpose2leftpose(model.pose.r[3:])
    leftp2 = fullrightpose2leftpose(model.pose.r[:])
    print '~~~~~~~~~~~~~~~~ mirror R-->L pose'
    print 'test =', np.allclose(leftp1, leftp2[3:])
    print 'test =', np.allclose(poseL[idx], leftp1)
    print 'test =', np.allclose(poseL[idx], leftp2[3:])
    #
    model_R.pose[:3] = 0
    model_L.pose[:3] = 0
    model_R.pose[:] = model.pose.r[:]
    model_L.pose[:] = fullrightpose2leftpose(model.pose.r[:])
    # model_R.pose[3:] = poseR[idx]
    # model_L.pose[3:] = poseL[idx]
    mesh_R = Mesh(v=model_R.r, f=model_R.f)
    mesh_L = Mesh(v=model_L.r, f=model_L.f)
    #
    radius = .005
    joints_L = [Sphere(np.array(jointPos), radius).to_mesh(np.eye(3)[0 if jointID == 0 else 1]) for jointID, jointPos in enumerate(model_L.J_transformed)]
    joints_R = [Sphere(np.array(jointPos), radius).to_mesh(np.eye(3)[0 if jointID == 0 else 1]) for jointID, jointPos in enumerate(model_R.J_transformed)]
    #
    return mesh_L, mesh_R, joints_L, joints_R


if __name__ == '__main__':

    from glob import glob
    pathPKLs = sorted(glob('/home/liuhx/Documents/data_glove/MANO_data/manoposesv10/mano_poses_v1_0/handsOnly_REGISTRATIONS_r_lm___POSES/*.pkl'))

    mvs = MeshViewers(window_width=1600, window_height=1600, shape=[2, 2])

    for ii, pathPKL in enumerate(pathPKLs):
        print '\n\n\n'
        print '-------------------------------------------------------------> %4d / %4d' % (ii + 1, len(pathPKLs))
        mesh_L, mesh_R, joints_L, joints_R = load_and_show(pathPKL, ii)
        mvs[1][0].set_static_meshes([mesh_L], blocking=True)
        mvs[1][1].set_static_meshes([mesh_R], blocking=True)
        mesh_L.f = []
        mesh_R.f = []
        mvs[0][0].set_static_meshes([mesh_L] + joints_L, blocking=True)
        mvs[0][1].set_static_meshes([mesh_R] + joints_R, blocking=True)
        print
        raw_input('Press Enter to Continue ...')

    print
    print 'FINITO'
    print
