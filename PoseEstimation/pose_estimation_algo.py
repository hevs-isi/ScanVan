def pose_estimation(p3d_1,p3d_2,p3d_3,iterations):
    if len(p3d_1)==len(p3d_2)==len(p3d_3):
        longueur=len(p3d_1)
    sv_u=list(np.ones(longueur))
    sv_v=list(np.ones(longueur))
    sv_w=list(np.ones(longueur))
    for i in range(iterations):
        sv_r,sv_t=estimation_rot_trans(p3d_1,p3d_2,p3d_3,sv_u,sv_v,sv_w)
        sv_u,sv_v,sv_w=estimation_rayons(p3d_1,p3d_2,p3d_3,sv_u,sv_v,sv_w,sv_r,sv_t)
    sv_scene,positions=pose_scene(p3d_1,p3d_2,p3d_3,sv_u,sv_v,sv_w,sv_r,sv_t)
    return [sv_scene,positions]

def svd_rotation(v,u):
    vu=np.dot(v,u)
    det=round(np.linalg.det(vu),4)
    m=np.identity(3)
    m[2,2]=det
    vm=np.dot(v,m)
    vmu=np.dot(vm,u)
    return vmu

def block_diag(a,b,c):
    if a.shape==b.shape==c.shape:
        s=a.shape
        x1=[a,np.zeros(s),np.zeros(s)]
        x2=[np.zeros(s),b,np.zeros(s)]
        x3=[np.zeros(s),np.zeros(s),c]
        x=np.block([x1,x2,x3])
        return x
    else:
        print('error: shape')

def estimation_rot_trans(p3d_1,p3d_2,p3d_3,sv_u,sv_v,sv_w):
    if len(p3d_1)==len(p3d_2)==len(p3d_3)==len(sv_u)==len(sv_v)==len(sv_w):
        longueur=len(p3d_1)
    p3d_1_exp=[]
    p3d_2_exp=[]
    p3d_3_exp=[]
    for i in range(longueur):
        p3d_1_exp.append(np.multiply(p3d_1[i],sv_u[i]))
        p3d_2_exp.append(np.multiply(p3d_2[i],sv_v[i]))
        p3d_3_exp.append(np.multiply(p3d_3[i],sv_w[i]))
    sv_corr_12=np.zeros((3,3))
    sv_corr_23=np.zeros((3,3))
    sv_corr_31=np.zeros((3,3))
    sv_cent_1=np.zeros(3)
    sv_cent_2=np.zeros(3)
    sv_cent_3=np.zeros(3)
    sv_diff_1=np.zeros(3)
    sv_diff_2=np.zeros(3)
    sv_diff_3=np.zeros(3)
    for i in range(longueur):
        sv_cent_1+=p3d_1_exp[i]
        sv_cent_2+=p3d_2_exp[i]
        sv_cent_3+=p3d_3_exp[i]
    sv_cent_1/=longueur
    sv_cent_2/=longueur
    sv_cent_3/=longueur
    for i in range(longueur):
        sv_diff_1=p3d_1_exp[i]-sv_cent_1
        sv_diff_2=p3d_2_exp[i]-sv_cent_2
        sv_diff_3=p3d_3_exp[i]-sv_cent_3
        sv_corr_12+=np.outer(sv_diff_1,sv_diff_2)
        sv_corr_23+=np.outer(sv_diff_2,sv_diff_3)
        sv_corr_31+=np.outer(sv_diff_3,sv_diff_1)       
    svd_U_12,svd_s_12,svd_Vt_12=np.linalg.svd(sv_corr_12)
    sv_r_12=svd_rotation(svd_Vt_12.transpose(),svd_U_12.transpose())
    svd_U_23,svd_s_23,svd_Vt_23=np.linalg.svd(sv_corr_23)
    sv_r_23=svd_rotation(svd_Vt_23.transpose(),svd_U_23.transpose())
    svd_U_31,svd_s_31,svd_Vt_31=np.linalg.svd(sv_corr_31)
    sv_r_31=svd_rotation(svd_Vt_31.transpose(),svd_U_31.transpose())
    sv_t_12=sv_cent_2-np.dot(sv_r_12,sv_cent_1)
    sv_t_23=sv_cent_3-np.dot(sv_r_23,sv_cent_2)
    sv_t_31=sv_cent_1-np.dot(sv_r_31,sv_cent_3)
    sv_r=block_diag(sv_r_12,sv_r_23,sv_r_31)
    sv_t=block_diag(sv_t_12,sv_t_23,sv_t_31)
    return [sv_r,sv_t]

def centers_determination(sv_r,sv_t):
    sv_r_12=sv_r[0:3,0:3]
    sv_r_23=sv_r[3:6,3:6]
    sv_r_31=sv_r[6:9,6:9]
    sv_t_12=sv_t[0,0:3]
    sv_t_23=sv_t[1,3:6]
    sv_t_31=sv_t[2,6:9]
    c1=np.zeros(3)
    c2=c1+sv_t_12
    c3=c2+np.dot(sv_r_12,sv_t_23)
    return c1,c2,c3

def intersection(liste_p,liste_azim):
    if len(liste_p)==len(liste_azim):
        longueur=len(liste_p)
    sum_v=np.zeros((3,3))
    sum_vp=np.zeros((3,1))
    for i in range(longueur):
        azim=np.matrix(liste_azim[i])
        p=np.matrix(liste_p[i])
        v=np.identity(3)-np.dot(azim.transpose(),azim)
        vp=np.dot(v,p.transpose())
        sum_v+=v
        sum_vp+=vp
    inter=np.dot(np.linalg.inv(sum_v),sum_vp)
    inter=np.squeeze(np.asarray(inter))
    return inter

def estimation_rayons(p3d_1,p3d_2,p3d_3,sv_u,sv_v,sv_w,sv_r,sv_t):
    if len(p3d_1)==len(p3d_2)==len(p3d_3):
        longueur=len(p3d_1)
    sv_r_12=sv_r[0:3,0:3]
    sv_r_23=sv_r[3:6,3:6]
    sv_r_31=sv_r[6:9,6:9]
    c1,c2,c3=centers_determination(sv_r,sv_t)
    sv_u=[]
    sv_v=[]
    sv_w=[]
    for i in range(longueur):
        azim1=np.array(p3d_1[i],copy=True)
        azim2=np.array(p3d_2[i],copy=True)
        azim3=np.array(p3d_3[i],copy=True)
        azim2=np.dot(azim2,sv_r_23)
        azim2=np.dot(azim2,sv_r_31)
        azim3=np.dot(azim3,sv_r_31)
        inter=intersection([c1,c2,c3],[azim1,azim2,azim3])
        inter1=c1+(np.dot(inter-c1,azim1)/np.dot(azim1,azim1))*azim1
        inter2=c2+(np.dot(inter-c2,azim2)/np.dot(azim2,azim2))*azim2
        inter3=c3+(np.dot(inter-c3,azim3)/np.dot(azim3,azim3))*azim3
        sv_u.append(np.linalg.norm(inter1-c1))
        sv_v.append(np.linalg.norm(inter2-c2))
        sv_w.append(np.linalg.norm(inter3-c3))
    return [sv_u,sv_v,sv_w]

def pose_scene(p3d_1,p3d_2,p3d_3,sv_u,sv_v,sv_w,sv_r,sv_t):
    if len(p3d_1)==len(p3d_2)==len(p3d_3):
        longueur=len(p3d_1)
    sv_r_12=sv_r[0:3,0:3]
    sv_r_23=sv_r[3:6,3:6]
    sv_r_31=sv_r[6:9,6:9]
    c1,c2,c3=centers_determination(sv_r,sv_t)
    sv_scene=[]
    for i in range(longueur):
        azim1=np.array(p3d_1[i],copy=True)
        azim2=np.array(p3d_2[i],copy=True)
        azim3=np.array(p3d_3[i],copy=True)
        azim2=np.dot(azim2,sv_r_23)
        azim2=np.dot(azim2,sv_r_31)
        azim3=np.dot(azim3,sv_r_31)
        inter=intersection([c1,c2,c3],[azim1,azim2,azim3])
        sv_scene.append(inter)
    positions=[c1,c2,c3]
    return [sv_scene,positions]

def fusion(model_1,model_2):
    scene_1=model_1[0]
    pos_1=model_1[1]
    scene_2=model_2[0]
    pos_2=model_2[1]
    pos_a_1=np.asarray(pos_1[-2])
    pos_b_1=np.asarray(pos_1[-1])
    pos_a_2=np.asarray(pos_2[0])
    pos_b_2=np.asarray(pos_2[1])
    pos_c=np.asarray(pos_2[2])
    scale_factor=np.linalg.norm(pos_b_1-pos_a_1)/np.linalg.norm(pos_b_2-pos_a_2)
    t1=(pos_b_1-pos_a_1)/np.linalg.norm(pos_b_1-pos_a_1)
    t2=(pos_b_2-pos_a_2)/np.linalg.norm(pos_b_2-pos_a_2)
    v=np.cross(t1,t2)
    c=np.dot(t1,t2)
    w=np.matrix([[0.0,-v[2],v[1]],[v[2],0.0,-v[0]],[-v[1],v[0],0.0]])
    rotation=np.identity(3)+w+np.dot(w,w)/(1+c)
    translation=pos_a_1-pos_a_2
    scene=[]
    for i in range(len(scene_1)):
        point=np.squeeze(np.asarray(scene_1[i]))
        scene.append(point)
    for i in range(len(scene_2)):
        point=np.squeeze(np.asarray(translation+scale_factor*np.dot(scene_2[i],rotation)))
        scene.append(point)
    positions=pos_1+[np.squeeze(np.asarray(translation+scale_factor*np.dot(pos_c,rotation)))]
    model=[scene,positions]
    return model

def fusion_totale(x):
    model=x[0]
    for i in range(1,len(x)):
        model=fusion(model,x[i])
    return model
