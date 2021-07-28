Dependencies:
Under the dropdown menu "Models & code" please download "Models & Code" and follow the instructions.

The contents of this zip file correspond to the files found
at http://mano.is.tue.mpg.de/downloads
by clicking on the "MANO: Scans & Registrations" drop-down menu
and downloading the file:
"Training Scans Registrations (Right hand, Left hand, Left hand mirrored)".

In the file above there are 1554 scans and model registrations:
- for the Right hand
- for the Left hand, mirrored to appear as Right.

In the current zip file there are:
- The folder "handsOnly_REGISTRATIONS_r_lm___POSES", contains the Right-hand model parameters, computed by re-fitting the model to the above registrations (meshes might have tiny differences).
  For each scan/registration there is a PKL file containing a dictionary with keys:
  - ncomps:         Indicates the size of the pose space; 0 indicates the full pose space.
  - pose:           Pose parameters; here (48,) for the full pose space (ncomps=0). For each joint there are 3 parameters, the first three correspond to global wrist rotation.
  - betas:          Shape parameters (10,).
  - trans:          Translation parameters (3,).
  - model_name:     Right/Left hand mode; here it is always Right.
  - v:              Vertices of the model after applying shape, pose and translation.
  - J_transformed:  Joints   of the model after applying shape, pose and translation.
- handsOnly_REGISTRATIONS_r_lm___POSES___R.npy
  - Contains only the poses, gathered from all pkl files (Right hands as Rights ones, Left hands mirrored to appear as Right ones), after removing the global rotation to keep only articulation (45,).
- handsOnly_REGISTRATIONS_r_lm___POSES___L.npy
  - Mirrors the all poses of "handsOnly_REGISTRATIONS_r_lm___POSES___R.npy" so that they can be used for the Left hand model.
- read_and_visualise_example.py
  - example that reads the files for two poses (uncomment the one you want), runs checks, shows 3D visualization.
  - it contains the code snippets for mirroring Right poses to Left ones.

The parameters in this file are valid for (they have the same model files):
- mano_v1_1
- mano_v1_2

They are not valid for:
- mano_v1_1
