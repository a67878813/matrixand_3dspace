import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt


def Rz(TT):
    TT = float(TT)
    _Rz =np.array([[np.cos(TT*np.pi/180),-np.sin(TT*np.pi/180),0,0],
                 [np.sin(TT*np.pi/180),np.cos(TT*np.pi/180),0,0],
                    [0,0,1,0],
                    [0,0,0,1],
                    ])
    return _Rz
def Rx(TT):
    TT = float(TT)
    _Rx =np.array([[1,0,0,0],
                 [0,np.cos(TT*np.pi/180),-np.sin(TT*np.pi/180),0],
                    [0,np.sin(TT*np.pi/180),np.cos(TT*np.pi/180),0],
                    [0,0,0,1],
                    ])
    return _Rx
def Ry(TT):
    TT = float(TT)
    _Ry =np.array([[np.cos(TT*np.pi/180),0,np.sin(TT*np.pi/180),0],
                 [0,1,0,0],
                    [-np.sin(TT*np.pi/180),0,np.cos(TT*np.pi/180),0],
                    [0,0,0,1],
                    ])
    return _Ry
def Trans_(c):
    c= np.array(c)
    dx = float(c[0])
    dy = float(c[1])
    dz = float(c[2])
    _Txyz =np.array([[1,0,0,dx],
                 [0,1,0,dy],
                [0,0,1,dz],
                [0,0,0,1],
                    ])
    return _Txyz

#模型变换  局部坐标系 到 世界坐标系
def get_Model2World_matrix(_center,_roter,_position):
    '''
    模型到世界坐标系
    先旋转再平移
    矩阵点乘，左乘
    数学从左向右，
    矩阵从右向左乘
    
    '''
    M1 = Trans_ (_center * -1)
    M2 = Rx(_roter[0])
    M3 = Ry(_roter[1])
    M4 = Rz(_roter[2])
    M5 = Trans_ (_center)
    M6 = Trans_ (_position)
    n = np.dot(M2,M1)
    n = np.dot(M3,n)
    n = np.dot(M4,n)
    n = np.dot(M5,n)
    n = np.dot(M6,n)
    return n


#模型变换  世界坐标系 到 相机坐标系
def get_World2Camera_matrix(_eye,_target,_Vup):
    '''
    eye 目标坐标，上向量
    世界到相机坐标系
    先旋转再平移
    矩阵点乘，左乘
    数学从左向右，
    矩阵从右向左乘
    
    '''
    eye = np.array(_eye)[:3]
    _target = np.array(_target)[:3]
    viewUp = np.array([_Vup[0],_Vup[1],_Vup[2]])
    f = np.array([_target[0]-_eye[0],_target[1]-_eye[1],_target[2]-_eye[2]])
    f = f/np.sqrt((f*f).sum())
    #f 摄像机方向向量
    print("f",f)
    #print(viewUp)
    #s 侧向量  =  方向向量 叉乘 上向量
    s  =  np.cross(f,viewUp)
    s = s/np.sqrt((s*s).sum())
    print("s",s)
    u = np.cross(s,f)
    u = u/np.sqrt((u*u).sum())
    print("u",u)
    T_world_to_camera = np.array([
        [s[0],s[1],s[2], -np.dot(s,eye)],
        [u[0],u[1],u[2], -np.dot(u,eye)],
        [-f[0],-f[1],-f[2], np.dot(f,eye)],
        [0,0,0,1]
    
    
    
        ]
    )
    
    return T_world_to_camera


#模型变换  世界坐标系 到 相机坐标系
def get_World2Camera_matrix2(_eye,_front,_Vup):
    '''
    eye ，前向量，上向量
    
    
    #可动态更新front Vup 向量

    front 左乘R(pitch) 左乘R（yaw）
    Vup 左乘R（roll)


    世界到相机坐标系
    先旋转再平移
    矩阵点乘，左乘
    数学从左向右，
    矩阵从右向左乘
    https://blog.csdn.net/wangdingqiaoit/article/details/51586007
    forwardx = -sin (yaw)  *cos(pitch)
    fy = sin(pitch)
    fz = -cos(yaw)* cos(pitch)
    f = normalize(f)
    '''
    eye = np.array(_eye)[:3]
    
    viewUp = np.array([_Vup[0],_Vup[1],_Vup[2]])
    f = np.array([_front[0],_front[1],_front[2]])
    f = f/np.sqrt((f*f).sum())
    #f 摄像机方向向量
    print("f",f)
    #print(viewUp)
    #s 侧向量  =  方向向量 叉乘 上向量
    s  =  np.cross(f,viewUp)
    s = s/np.sqrt((s*s).sum())
    print("s",s)
    u = np.cross(s,f)
    u = u/np.sqrt((u*u).sum())
    print("u",u)
    T_world_to_camera = np.array([
        [s[0],s[1],s[2], -np.dot(s,eye)],
        [u[0],u[1],u[2], -np.dot(u,eye)],
        [-f[0],-f[1],-f[2], np.dot(f,eye)],
        [0,0,0,1]
    
    
    
        ]
    )
    
    return T_world_to_camera

def update_front(pitch,yaw,old_front):
    #计算
    pitch = float(pitch)
    yaw = float(yaw)
    
    fx = -np.sin(yaw*np.pi/180) *np.cos(pitch*np.pi/180)
    fy = np.sin(pitch*np.pi/180)
    fz = -np.cos(yaw*np.pi/180) *np.cos(pitch*np.pi/180)
    f = np.array([fx,fy,fz])
    updated_front = f/np.sqrt((f*f).sum())
    return updated_front
    


#模型变换  世界坐标系 到 相机坐标系
def get_World2Camera_matrix3(_eye,_pitch,_yaw,_roll):

    '''
    相比4计算方法 偏航角偏移-90度
    eye , 俯仰角 ，  yaw偏航角 ，roll滚转角
    
    也许有问题
    俯仰偏航滚转计算也许有问题
    
    
    世界到相机坐标系
    先旋转再平移
    矩阵点乘，左乘
    数学从左向右，
    矩阵从右向左乘
    pitch俯仰角
    yaw偏航角
    roll滚转角
    np.cos(TT*np.pi/180),-np.sin(TT*np.pi/180)
    '''
    pitch = float(_pitch)
    yaw = float(_yaw)
    roll = float(_roll)

    eye = np.array(_eye)[:3]
    
    viewUp_x = np.sin(roll*np.pi/180)
    viewUp_y = np.cos(roll*np.pi/180)
    #viewUp上向量
    viewUp = np.array([viewUp_x,viewUp_y,0])
    fx = np.cos(pitch*np.pi/180) * np.cos(yaw*np.pi/180)
    fy =np.sin(pitch*np.pi/180)
    fz = np.cos(pitch*np.pi/180) * np.sin(yaw*np.pi/180)
    f= np.array([fx,fy,fz])
    f = f/np.sqrt((f*f).sum())
    #f 摄像机方向向量
    print("f",f)
    #print(viewUp)
    #s 侧向量  =  方向向量 叉乘 上向量
    s  =  np.cross(f,viewUp)
    s = s/np.sqrt((s*s).sum())
    print("s",s)
    u = np.cross(s,f)
    u = u/np.sqrt((u*u).sum())
    print("u",u)
    T_world_to_camera = np.array([
        [s[0],s[1],s[2], -np.dot(s,eye)],
        [u[0],u[1],u[2], -np.dot(u,eye)],
        [-f[0],-f[1],-f[2], np.dot(f,eye)],
        [0,0,0,1]
    
    
    
        ]
    )
    
    return T_world_to_camera



#模型变换  世界坐标系 到 相机坐标系
def get_World2Camera_matrix4(_eye,_pitch,_yaw,_roll):

    '''
    eye , 俯仰角 ，  yaw偏航角 ，roll滚转角
    
    也许有问题
    俯仰偏航滚转计算也许有问题
    
    
    世界到相机坐标系
    先旋转再平移
    矩阵点乘，左乘
    数学从左向右，
    矩阵从右向左乘
    pitch俯仰角
    yaw偏航角
    roll滚转角
    np.cos(TT*np.pi/180),-np.sin(TT*np.pi/180)
    '''
    pitch = float(_pitch)
    yaw = float(_yaw)
    roll = float(_roll)

    eye = np.array(_eye)[:3]
    
    M1 = Rx(pitch)
    M2 = Ry(yaw)
    M3 = Rz(roll)
    M4 = Trans_(eye)
    
    n = np.dot(M2,M1)
    n = np.dot(M3,n)
    n = np.dot(M4,n)
    n = np.linalg.inv(n) 
    T_world_to_camera = n
    
    return T_world_to_camera


def get_ProjectionT_matrix_cameraSpace_minusZ(_l,_r,_b,_t,_nearVal,_farVal):
    l = float(_l)
    r = float(_r)
    b = float(_b)
    t = float(_t)
    n = float(_nearVal)
    f = float(_farVal)
    
    
    T = np.array([n/r,0,0,
                  0,0,n/t,0,0,
                  0,0,-(f+n)/(f-n),-2*f*n/(f-n),
                  0,0,-1,0])
    print(T)
    T_camera_to_clipspace = T.reshape(4,4)
    return T_camera_to_clipspace



def get_Projection_matrix_cameraSpace2ClipSpace_minusZ(_l,_r,_b,_t,_nearVal,_farVal):
    
    #camera space to clip space
    
    l = float(_l)
    r = float(_r)
    b = float(_b)
    t = float(_t)
    n = float(_nearVal)
    f = float(_farVal)
    
    
    P = np.array([n/r,0,0,0,
                  0,n/t,0,0,
                  0,0,-(f+n)/(f-n),-2*f*n/(f-n),
                  0,0,-1,0])
    print(P)
    P_camera_to_clipspace = P.reshape(4,4)
    return P_camera_to_clipspace


def get_Projection_matrix_cameraSpace2ClipSpace_minusZ_FOV(_fov,_aspect,_near,_far):
    #camera space to clip space
    #_aspect宽高比 宽/高
    fov = float(_fov)
    aspect = float(_aspect)
    n = float(_near)
    f = float(_far)
    #h = near*np.tan(fov/2*np.pi/180)
    h = n*np.tan(fov/360*np.pi)
    w = h*aspect
    
    
    P = np.array([1/aspect/np.tan(fov/360*np.pi),0,0,0,
                  0,1/np.tan(fov/360*np.pi),0,0,
                  0,0,-(f+n)/(f-n),-2*f*n/(f-n),
                  0,0,-1,0])
    print(P)
    P_camera_to_clipspace = P.reshape(4,4)
    return P_camera_to_clipspace


def projection_division(_array):
    #clip_space to NDC
    a = np.array(_array)
    ax = a[0]
    ay = a[1]
    az = a[2]
    aw = a[3]
    cx = ax/aw
    cy = ay/aw
    cz = az/aw
    cw = aw/aw
    array2 = np.array([[cx],[cy],[cz],[cw]])
    return array2

def get_Orthographic_Projection_matrix_cameraSpace2NDC_minusZ(_l,_r,_b,_t,_nearVal,_farVal):
    l = float(_l)
    r = float(_r)
    b = float(_b)
    t = float(_t)
    n = float(_nearVal)
    f = float(_farVal)
    
    
    P = np.array([1/r,0,0,0,
                  0,1/t,0,0,
                  0,0,-2/(f-n),-(f+n)/(f-n),
                  0,0,0,1])
    print(P)
    P_camera_to_clipspace = P.reshape(4,4)
    return P_camera_to_clipspace


def Viewport_NDC_to_screen(sx,sy,_width,_hight,ns,fs):
    #sx sy 屏幕左下角
    #ns近场 fs远场
    #线性映射 -1，sx      1, sx +width
    #-1,sy       1,sy+hight
    #-1,ns           1,fs
    sx = float(sx)
    sy = float(sy)
    _width = float(_width)
    _hight = float(_hight)
    ns = float(ns)
    fs = float(fs)
    
    ViewPort = np.array([_width/2,0,0,sx+_width/2,
                  0,_hight/2,   0,   sy+_hight/2,
                  0,0,     (fs-ns)/2,  (fs+ns)/2,
                  0,0,0,1])
    
    ViewPort = ViewPort.reshape(4,4)
    return ViewPort



#模型顶点array 左乘模型变换矩阵    世界坐标
#                                 搞事情需要转换到这个坐标系
# 
#                                       左乘 视变换矩阵 左乘 投影变换矩阵  ——透视除法（除齐次坐标欧米伽分量） 左乘视口变换矩阵                                   -----屏幕
#SPACE：   local                  world             eye              clip                              NDC (normalized Device Coordinates)([-1,1] * 3)          screen