"""Offline FK and IK in CAD-zero coordinates; no encoder or hardware mapping."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares
ROOT=Path(__file__).resolve().parents[1]
MODEL=json.loads((ROOT/'model/model_manifest.json').read_text())
NAMES=['shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_yaw','wrist_roll','gripper_left_joint']
JOINTS={j['name']:j for j in MODEL['joints']}
def fk(q):
 vals=dict(zip(NAMES,q));vals['gripper_right_joint']=vals.get('gripper_left_joint',0)
 frames={'base_link':np.eye(4)}
 for j in MODEL['joints']:
  T=np.eye(4);T[:3,3]=j['xyz'];T[:3,:3]=Rotation.from_euler('xyz',j.get('rpy',[0,0,0])).as_matrix()
  motion=np.eye(4);v=vals.get(j['name'],0)
  if j['type']=='revolute':motion[:3,:3]=Rotation.from_rotvec(np.array(j['axis'])*v).as_matrix()
  elif j['type']=='prismatic':motion[:3,3]=np.array(j['axis'])*v
  frames[j['child']]=frames[j['parent']]@T@motion
 return frames

def inverse(target,seed=None):
 lo=np.array([JOINTS[n]['lower'] for n in NAMES[:6]]);hi=np.array([JOINTS[n]['upper'] for n in NAMES[:6]])
 q0=np.clip(np.array(seed if seed is not None else (lo+hi)/2),lo+1e-7,hi-1e-7)
 def residual(q):
  T=fk(q)['tool0'];return np.r_[5*(T[:3,3]-target[:3,3]),Rotation.from_matrix(target[:3,:3].T@T[:3,:3]).as_rotvec()]
 sol=least_squares(residual,q0,bounds=(lo,hi),xtol=1e-11,ftol=1e-11,gtol=1e-11,max_nfev=1000)
 return sol.x,{'success':bool(sol.success),'residual_norm':float(np.linalg.norm(sol.fun)),'evaluations':sol.nfev}
