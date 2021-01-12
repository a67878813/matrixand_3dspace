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