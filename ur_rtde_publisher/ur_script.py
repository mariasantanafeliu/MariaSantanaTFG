#!/usr/bin/env python
import rospy
import time
import numpy as np
import math

def movej_rad(q,a,v,t,r):
    if len(q) != 6:
        print("Error: Inconsistent dimension of position vector")
        return 
    if not a:
        a = 1
    if not v:
        v = 1
    if not t:
        t = 0
    if not r:
        r = 0
    ls = ','.join(str(e) for e in q)
    st = "movej(["+ ls +"], a="+ str(a) +", v="+ str(v) + ", t="+ str(t) +", r="+ str(r) + ")"
    #print(st)
    return st

def movej_deg(q,a,v,t,r):
    if len(q) != 6:
        print("Error: Inconsistent dimension of position vector")
        return 
    if not a:
        a = 1
    if not v:
        v = 1
    if not t:
        t = 0
    if not r:
        r = 0
    qd = np.radians(q)    
    ls = ','.join(str(e) for e in qd)
    st = "movej(["+ ls +"], a="+ str(a) +", v="+ str(v) + ", t="+ str(t) +", r="+ str(r) + ")"
    #print(st)
    return st

def movel_car(pos,a,v):
    if len(pos) != 6:
        print("Error: Inconsistent dimension of position vector")
        return 
    if not a:
        a = 1
    if not v:
        v = 0.1
    ls = ','.join(str(e) for e in pos)
    st = "movel(p["+ ls +"], a="+ str(a) +", v="+ str(v) +")"
    #print(st)
    return st

def movel_art(pos,a,v):
    if len(pos) != 6:
        print("Error: Inconsistent dimension of position vector")
        return 
    if not a:
        a = 1
    if not v:
        v = 0.1
    ls = ','.join(str(e) for e in pos)
    st = "movel(["+ ls +"], a="+ str(a) +", v="+ str(v) +")"
    #print(st)
    return st

def speedl(pos,a,t):
    if len(pos) != 6:
        print("Error: Inconsistent dimension of position vector")
        return 
    if not a:
        a = 0.5
    if not t:
        t = 0.8
    ls = ','.join(str(e) for e in pos)
    st = "speedl(["+ ls +"], a="+ str(a) + ", t=" + str(t) + ")"
    #print(st)
    return st

def speedj(pos,a,t):
    if len(pos) != 6:
        print("Error: Inconsistent dimension of position vector")
        return 
    if not a:
        a = 0.5
    ls = ','.join(str(e) for e in pos)
    st = "speedj(["+ ls +"], a="+ str(a) + ", t=" + str(t) + ")"
    #print(st)
    return st

def publisher(pos, pub,rate):
    try:
        #time.sleep(0.1)
        if not rospy.is_shutdown():
            pub.publish(pos)
            rate.sleep()
    except KeyboardInterrupt:
            rospy.signal_shutdown("KeyboardInterrupt")
            raise

def shutdown(pub): # APAGAR ROBOT POR COMPLETO
    msg = "powerdown()"
    pub.publish(msg)

#def cam2ur3():
    # Transf de pos-or camara laparoscopica a base ur3

#def ur32cam(pos):
    # Transf de pos-or base ur3 a punta camara laparoscopica

def ur32OR(pos):
    # punto del UR a coordenadas del sistema de ref global
    URor = [-.63087,.28278,.08577] # Modificar de acuerdo al sist ref del robot
    theta = np.radians(135)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s, 0 , URor[0]), (s, c, 0, URor[1]), (0, 0, 1, URor[2]), (0, 0, 0, 1)))
    p = [pos[0], pos[1], pos[2], 1]
    C = np.matmul(R,p)
    return C[0], C[1], C[2]
    
def joint2twist(Axis, Point, JointType):
  if JointType == 'rot':
    aux = -np.cross(Axis, Point)
    twist = np.concatenate((aux, Axis), axis=0)
  elif JointType == 'tra':
    aux2 = [0, 0, 0]
    twist = np.concatenate((Axis, aux2), axis=0)
  else:
    twist = np.array([0, 0, 0, 0, 0, 0])
  return twist

def axis2skew(u):
  # Generate a skew symmetric matrix from a vector (axis u) 
  return np.matrix((
      (0.0, -u[2], u[1]),
      (u[2], 0.0, -u[0]),
      (-u[1], u[0], 0.0),
      ))

def expAxAng(AxAng):
  #Matrix Exponential of a Rigid Body ORIENTATION
  #AxAng is a  [x y z theta] == [W theta] (1x4)
  axis = (AxAng[0], AxAng[1], AxAng[2])
  u = axis2skew(axis)
  theta = AxAng[3]
  I = np.eye(3)
  A = I + u*np.sin(theta)  + u*u*(1-np.cos(theta)) #expresion general de la formula de Rodrigues
  return A

def expScrew(TwMag):
  # Matrix Exponential of a  Rigid Body Motion by SCREW movement
  # "Twist-Mangitude" (TwMag) [xi; theta] (7x1)
  # Returns a homogeneous "H" matrix (4x4)
  v = np.array([TwMag[0], TwMag[1], TwMag[2]])
  w = np.array([TwMag[3], TwMag[4], TwMag[5]])
  theta = TwMag[6]
  if np.linalg.norm(w) == 0:
    R = np.eye(3)
    P = v.dot(theta)
  else:
    R = expAxAng(np.append(w, theta))
    I = np.eye(3) 
    P_aux = np.matmul((I-R),(np.cross(w,v)))
    P = np.array([P_aux[0,0], P_aux[0,1], P_aux[0,2]])
    #print ("P = ", P)
  
  return np.matrix((
      (R[0,0], R[0,1], R[0,2], P[0]),
      (R[1,0], R[1,1], R[1,2], P[1]),
      (R[2,0], R[2,1], R[2,2], P[2]),
      (0, 0, 0, 1),
      ))

def NearZero(z):
    return abs(z) < 1e-6

def TransToRp(T):
    T = np.array(T)
    return T[0: 3, 0: 3], T[0: 3, 3]

def VecToso3(omg):
    return np.array([[0,      -omg[2],  omg[1]],
                     [omg[2],       0, -omg[0]],
                     [-omg[1], omg[0],       0]])

def VecTose3(V):
    return np.r_[np.c_[VecToso3([V[0], V[1], V[2]]), [V[3], V[4], V[5]]],
                 np.zeros((1, 4))]

def MatrixExp3(so3mat):
    omgtheta = so3ToVec(so3mat)
    if NearZero(np.linalg.norm(omgtheta)):
        return np.eye(3)
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = so3mat / theta
        return np.eye(3) + np.sin(theta) * omgmat \
               + (1 - np.cos(theta)) * np.dot(omgmat, omgmat)

def MatrixExp6(se3mat):
    se3mat = np.array(se3mat)
    omgtheta = so3ToVec(se3mat[0: 3, 0: 3])
    if NearZero(np.linalg.norm(omgtheta)):
        return np.r_[np.c_[np.eye(3), se3mat[0: 3, 3]], [[0, 0, 0, 1]]]
    else:
        theta = AxisAng3(omgtheta)[1]
        omgmat = se3mat[0: 3, 0: 3] / theta
        return np.r_[np.c_[MatrixExp3(se3mat[0: 3, 0: 3]),
                           np.dot(np.eye(3) * theta \
                                  + (1 - np.cos(theta)) * omgmat \
                                  + (theta - np.sin(theta)) \
                                    * np.dot(omgmat,omgmat),
                                  se3mat[0: 3, 3]) / theta],
                     [[0, 0, 0, 1]]]

def so3ToVec(so3mat):
    return np.array([so3mat[2][1], so3mat[0][2], so3mat[1][0]])

def AxisAng3(expc3):
    return (Normalize(expc3), np.linalg.norm(expc3))

def Normalize(V):
    return V / np.linalg.norm(V)

def joint2twist(Axis, Point, JointType):
  if JointType == 'rot':
    aux = -np.cross(Axis, Point)
    twist = np.concatenate((aux, Axis), axis=0)
  elif JointType == 'tra':
    aux2 = [0, 0, 0]
    twist = np.concatenate((Axis, aux2), axis=0)
  else:
    twist = np.array([0, 0, 0, 0, 0, 0])
  return twist

def Adjoint(T):
    """Computes the adjoint representation of a homogeneous transformation
    matrix
    :param T: A homogeneous transformation matrix
    :return: The 6x6 adjoint representation [AdT] of T
    Example Input:
        T = np.array([[1, 0,  0, 0],
                      [0, 0, -1, 0],
                      [0, 1,  0, 3],
                      [0, 0,  0, 1]])
    Output:
        np.array([[1, 0,  0, 0, 0,  0],
                  [0, 0, -1, 0, 0,  0],
                  [0, 1,  0, 0, 0,  0],
                  [0, 0,  3, 1, 0,  0],
                  [3, 0,  0, 0, 0, -1],
                  [0, 0,  0, 0, 1,  0]])
    """
    R, p = TransToRp(T)
    return np.r_[np.c_[R, np.zeros((3, 3))],
                 np.c_[np.dot(VecToso3(p), R), R]]

def JacobianSpace(Slist, thetalist):
    """Computes the space Jacobian for an open chain robot
    :param Slist: The joint screw axes in the space frame when the
                  manipulator is at the home position, in the format of a
                  matrix with axes as the columns
    :param thetalist: A list of joint coordinates
    :return: The space Jacobian corresponding to the inputs (6xn real
             numbers)
    Example Input:
        Slist = np.array([[0, 0, 1,   0, 0.2, 0.2],
                          [1, 0, 0,   2,   0,   3],
                          [0, 1, 0,   0,   2,   1],
                          [1, 0, 0, 0.2, 0.3, 0.4]]).T
        thetalist = np.array([0.2, 1.1, 0.1, 1.2])
    Output:
        np.array([[  0, 0.98006658, -0.09011564,  0.95749426]
                  [  0, 0.19866933,   0.4445544,  0.28487557]
                  [  1,          0,  0.89120736, -0.04528405]
                  [  0, 1.95218638, -2.21635216, -0.51161537]
                  [0.2, 0.43654132, -2.43712573,  2.77535713]
                  [0.2, 2.96026613,  3.23573065,  2.22512443]])
    """
    Js = np.array(Slist).copy().astype(np.float)
    T = np.eye(4)
    for i in range(1, len(thetalist)):
        T = np.dot(T, MatrixExp6(VecTose3(np.array(Slist)[:, i - 1] \
                                * thetalist[i - 1])))
        Js[:, i] = np.dot(Adjoint(T), np.array(Slist)[:, i])
    return Js

def rotation_matrix(theta1, theta2, theta3, order='xyz'):
    """
    input
        theta1, theta2, theta3 = rotation angles in rotation order (radias)
        oreder = rotation order of x,y,z e.g. XZY rotation -- 'xzy'
    output
        3x3 rotation matrix (numpy array)
    """
    c1 = np.cos(theta1)
    s1 = np.sin(theta1)
    c2 = np.cos(theta2)
    s2 = np.sin(theta2)
    c3 = np.cos(theta3)
    s3 = np.sin(theta3)

    if order == 'xzx':
        matrix=np.array([[c2, -c3*s2, s2*s3],
                         [c1*s2, c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3],
                         [s1*s2, c1*s3+c2*c3*s1, c1*c3-c2*s1*s3]])
    elif order=='xyx':
        matrix=np.array([[c2, s2*s3, c3*s2],
                         [s1*s2, c1*c3-c2*s1*s3, -c1*s3-c2*c3*s1],
                         [-c1*s2, c3*s1+c1*c2*s3, c1*c2*c3-s1*s3]])
    elif order=='yxy':
        matrix=np.array([[c1*c3-c2*s1*s3, s1*s2, c1*s3+c2*c3*s1],
                         [s2*s3, c2, -c3*s2],
                         [-c3*s1-c1*c2*s3, c1*s2, c1*c2*c3-s1*s3]])
    elif order=='yzy':
        matrix=np.array([[c1*c2*c3-s1*s3, -c1*s2, c3*s1+c1*c2*s3],
                         [c3*s2, c2, s2*s3],
                         [-c1*s3-c2*c3*s1, s1*s2, c1*c3-c2*s1*s3]])
    elif order=='zyz':
        matrix=np.array([[c1*c2*c3-s1*s3, -c3*s1-c1*c2*s3, c1*s2],
                         [c1*s3+c2*c3*s1, c1*c3-c2*s1*s3, s1*s2],
                         [-c3*s2, s2*s3, c2]])
    elif order=='zxz':
        matrix=np.array([[c1*c3-c2*s1*s3, -c1*s3-c2*c3*s1, s1*s2],
                         [c3*s1+c1*c2*s3, c1*c2*c3-s1*s3, -c1*s2],
                         [s2*s3, c3*s2, c2]])
    elif order=='xyz':
        matrix=np.array([[c2*c3, -c2*s3, s2],
                         [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
                         [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]])
    elif order=='xzy':
        matrix=np.array([[c2*c3, -s2, c2*s3],
                         [s1*s3+c1*c3*s2, c1*c2, c1*s2*s3-c3*s1],
                         [c3*s1*s2-c1*s3, c2*s1, c1*c3+s1*s2*s3]])
    elif order=='yxz':
        matrix=np.array([[c1*c3+s1*s2*s3, c3*s1*s2-c1*s3, c2*s1],
                         [c2*s3, c2*c3, -s2],
                         [c1*s2*s3-c3*s1, c1*c3*s2+s1*s3, c1*c2]])
    elif order=='yzx':
        matrix=np.array([[c1*c2, s1*s3-c1*c3*s2, c3*s1+c1*s2*s3],
                         [s2, c2*c3, -c2*s3],
                         [-c2*s1, c1*s3+c3*s1*s2, c1*c3-s1*s2*s3]])
    elif order=='zyx':
        matrix=np.array([[c1*c2, c1*s2*s3-c3*s1, s1*s3+c1*c3*s2],
                         [c2*s1, c1*c3+s1*s2*s3, c3*s1*s2-c1*s3],
                         [-s2, c2*s3, c2*c3]])
    elif order=='zxy':
        matrix=np.array([[c1*c3-s1*s2*s3, -c2*s1, c1*s3+c3*s1*s2],
                         [c3*s1+c1*s2*s3, c1*c2, s1*s3-c1*c3*s2],
                         [-c2*s3, s2, c2*c3]])

    return matrix

def OrientaPtoFulV1(R, F, L): # PENDIENTE DE CORREGIR
  ZYZ = np.zeros(3)
  Df = L - np.sqrt(((R[0]-F[0])**2) + ((R[1]-F[1])**2) + ((R[2]-F[2])**2))
  distxy = np.sqrt(((R[0]-F[0])**2) + ((R[1]-F[1])**2))
  z1 = np.arctan2((R[1]-F[1]),(R[0]-F[0]))
  y2 = np.arctan2(distxy,(R[2]-F[2]))
  z3 = 0
  ZYZ = [z1, y2, z3]
  #print(ZYZ)
  rotZYZ = rotation_matrix(ZYZ[0],ZYZ[1],ZYZ[2],order='zyz')
  r1 = np.arctan2((rotZYZ[1,2]),(rotZYZ[0,2]))
  r2 = np.arctan2(((np.sqrt(rotZYZ[0,2]**2 + rotZYZ[1,2]**2))),(rotZYZ[2,2]))
  r3 = np.arctan2((rotZYZ[2,1]),(-rotZYZ[2,0]))
  r = np.concatenate((r1,r2,r3), axis=None)

  check = np.all((r == 0))
  if check:
    r = np.array([0,3.14159265,0])

  x1 = np.concatenate((R,r), axis=None)
  #x2 = np.matmul(rotZYZ,[0,0,-L])

  return x1

def OrientaPtoFul(R, F, L): # PENDIENTE DE CORREGIR
  ZYZ = np.zeros(3)
  Df = L - np.sqrt(((R[0]-F[0])**2) + ((R[1]-F[1])**2) + ((R[2]-F[2])**2))
  distxy = np.sqrt(((R[0]-F[0])**2) + ((R[1]-F[1])**2))
  z1 = np.arctan2((R[1]-F[1]),(R[0]-F[0]))
  y2 = np.arctan2(distxy,(R[2]-F[2]))
  z3 = 0
  ZYZ = [z1, y2, z3]
  print("ZYZ")
  print(ZYZ)
  #print(ZYZ)
  rotZYZ = rotation_matrix(ZYZ[0],ZYZ[1],ZYZ[2],order='zyz')
  r1 = np.arctan2((rotZYZ[1,2]),(rotZYZ[0,2]))
  r2 = np.arctan2(((np.sqrt(rotZYZ[0,2]**2 + rotZYZ[1,2]**2))),(rotZYZ[2,2]))
  r3 = np.arctan2((rotZYZ[2,1]),(-rotZYZ[2,0]))
  r = np.concatenate((r1,r2,r3), axis=None)

  p = Rot.from_matrix(rotZYZ)
  r = p.as_rotvec()
  print("rotvec")
  print(r)
  #check = np.all((r == 0))
  #if check:
  #  r = np.array([0,3.14159265,0])

  x1 = np.concatenate((R,r), axis=None)
  #x2 = np.matmul(rotZYZ,[0,0,-L])

  return x1

def OrientaPtoFulOR(R, F, L): # PENDIENTE DE CORREGIR
  ZYZ = np.zeros(3)
  Df = L - np.sqrt(((R[0]-F[0])**2) + ((R[1]-F[1])**2) + ((R[2]-F[2])**2))
  distxy = np.sqrt(((R[0]-F[0])**2) + ((R[1]-F[1])**2))
  z1 = np.arctan2((R[1]-F[1]),(R[0]-F[0]))
  y2 = np.arctan2(distxy,(R[2]-F[2]))
  z3 = 0
  ZYZ = [z1, y2, z3]
  print("ZYZ")
  print(ZYZ)
  #print(ZYZ)
  rotZYZ = rotation_matrix(ZYZ[0],ZYZ[1],ZYZ[2],order='zyz')
  r1 = np.arctan2((rotZYZ[1,2]),(rotZYZ[0,2]))
  r2 = np.arctan2(((np.sqrt(rotZYZ[0,2]**2 + rotZYZ[1,2]**2))),(rotZYZ[2,2]))
  r3 = np.arctan2((rotZYZ[2,1]),(-rotZYZ[2,0]))
  r = np.concatenate((r1,r2,r3), axis=None)

  #p = Rot.from_matrix(rotZYZ)
  #r = p.as_rotvec()
  print("rotvec")
  print(r)
  check = np.all((r == 0))
  if check:
    r = np.array([0,3.14159265,0])

  x1 = np.concatenate((R,r), axis=None)
  #x2 = np.matmul(rotZYZ,[0,0,-L])

  return x1

def OrientaPtoFulJULIANA(R, F, L): # PENDIENTE DE CORREGIR
    dx = R[0]-F[0]
    dy = R[1]-F[1]
    dz = R[2]-F[2]
    alpha = np.arctan2(dy,dz)
    beta = math.pi - np.arctan2(dx,dz)
    p2 = 2*math.pi
    if abs(beta) > np.pi:
        if beta > 0:
            beta = beta - p2
        else:
            beta = beta + p2
    if abs(alpha) > np.pi:
        if alpha > 0:
            alpha = alpha - p2
        else:
            alpha = alpha + p2
    
    return [alpha, beta, 0]