import math as m
import numpy as np
import tf as tf
from scipy.spatial.transform import Rotation
import time




class FK:
    start_time = time.time()
    
    def rotx(self,alpha):
        '''
        Parameters
        ----------
        alpha : Angle of Rotaion around X-Axis (Radian).

        Returns
        -------
        rotx : Rotaion Matrix(3x3).

        '''
        rotx = np.matrix([[1, 0, 0, 0], [0, m.cos(alpha), -m.sin(alpha), 0], [0, m.sin(alpha), m.cos(alpha), 0],[0, 0, 0, 1]]);
       # print('Rotation about x:\n',rotx)
        return rotx
    def roty(self,beta):
        '''
        Parameters
        ----------
        alpha : Angle of Rotaion around Y-Axis (Radian).

        Returns
        -------
        rotx : Rotaion Matrix(3x3).

        '''
        roty = np.matrix([[m.cos(beta), 0,m.sin(beta), 0],[0, 1, 0, 0], [-m.sin(beta), 0, m.cos(beta), 0],[0, 0, 0, 1]] )
       # print('Rotation about y:\n',roty)
        return roty
    def rotz(self,gamma):
        '''
        Parameters
        ----------
        alpha : Angle of Rotaion around Z-Axis (Radian).

        Returns
        -------
        rotx : Rotaion Matrix(3x3).

        '''
        rotz = np.matrix([[m.cos(gamma), -m.sin(gamma), 0, 0], [m.sin(gamma), m.cos(gamma), 0, 0], [0, 0, 1, 0],[0, 0, 0, 1]])
       # print('Rotation about z:\n',rotz)
        return rotz
    
    def transx(self,Lx):
        
        transx = np.matrix([[1, 0, 0, Lx],[0, 1, 0, 0],[0, 0, 1, 0],[0, 0, 0, 1]])
      #  print('Translation about x:\n',transx)
        return transx
    
    def transz(self,Lz):
        
        transz = np.matrix([[1, 0, 0, 0],[0, 1, 0, 0],[0, 0, 1, Lz],[0, 0, 0, 1]])
        #print('Translation about z:\n',transz)
        return transz
    def fwdKin(self,q,q1,q3,q5,Lx,Lw,Lz):
        '''
        Parameters
        ----------
        q : Fixed Angle (Radian).
        q1 : Joint 1 Angle (Radian).
        q3 : Joint 3 Angle (Radian).
        q5 : Joint 5 Angle.
        
        Lx : Length of the Link1,2,3,4 (Meters).
        Lz : Length of the Link5 (Meters).
        Lw : Offset length of Link4 (Meters).
        

        Returns
        -------
        final_tr : Transformation Matrix (3x4).

        '''
        tr_0_1 = np.matmul(np.matmul(self.rotz(q1), self.rotx(m.pi/2)), self.transx(Lx))
        #print('Transformation matrix 1 w.r.t 0:\n',tr_0_1)
        tr_1_2 = np.matmul(np.matmul(self.rotz(q), self.rotx(-m.pi/2)), self.transx(Lx))
        #print('Transformation matrix 2 w.r.t 1:\n',tr_1_2)
        tr_2_3 = np.matmul(np.matmul(self.rotz(q3), self.rotx(m.pi/2)), self.transx(Lx))
        #print('Transformation matrix 3 w.r.t 2:\n',tr_2_3)
        tr_3_4 = np.matmul(np.matmul(self.rotz(q), self.rotx(m.pi/2)), self.transx(Lx+Lw))
        #print('Transformation matrix 4 w.r.t 3:\n',tr_3_4)
        tr_4_5 = np.matmul(np.matmul(self.rotz(q5), self.rotx(0)), self.transz(Lz))
        #print('Transformation matrix 5 w.r.t 4:\n',tr_4_5)
        final_tr = np.matmul(np.matmul(np.matmul(np.matmul(tr_0_1, tr_1_2), tr_2_3), tr_3_4), tr_4_5)
        
        
        #print("The Final Forward transformation matrix matrix")
        #print('The Final Transformation matrix:\n',final_tr)
        return final_tr
      
    
    def ypr2rot(self,yaw,pitch,roll):
        '''
        Parameters
        ----------
        yaw : Angle rotated about Z-Axis (Radian)(Viz. Gamma).
        pitch : Angle rotated about Y-Axis (Radian)(viz. Beta) .
        roll : Angle rotated about X-axis (Radian)(viz. Alpha).

        Returns
        -------
        result : Set of possible solution after filteration and conversion of range to [-pi, pi].
        
        * This function uses rotation functions to convert roll, pitch and yaw 
        values to rotation matrix which is formed by sequential rotation(which means Rot about Z> Rot about Y> Rot about X) around 
        ZYX axis
        * The values required to find joint angles are passed to IK() function.
        * The output converted to a matrix with different possible solution sets.
        * All the possible solutions are placed in a loop wherein, solutions are 
        applied to forward kinematics functions, then filtered by comparing first 
        value of transformation matrix with position matrix developed by RPY values
        and saved filtered sets to a separate matrix.
        * convertion of range of values from [-2pi, 2pi] to [-pi to pi] and
        its resulting matrix is returned from the function
        '''
        global pose 
        pose = np.matmul(np.matmul(self.rotz(yaw), self.roty(pitch)), self.rotx(roll))
        #print('Final rotation matrix from the input Gamma,Beta,Alpha:,\n',pose)

        #passing value to make_matrix function
        az = pose[2, 2]
        ax = pose[0, 2]
        ay = pose[1, 2]
        nz = pose[2, 0]

        soln = self.IK(az, ax, ay, nz)
        if soln ==None:
            return None
        else: 
            #print(type(soln))
            solnMatrix = np.array([soln[0], soln[2], soln[4],
                                   soln[0], soln[2], soln[5], 
                                   soln[1], soln[3], soln[6],
                                   soln[1], soln[3], soln[7]]).reshape(4, 3)
            #print(solnMatrix)
           # print('Length of the solMatrix',len(solnMatrix))
            # filteration of solution sets using forward kinematics
            result = np.empty((0, 3))
            for i in range(len(solnMatrix)):
                rfk = self.fwdKin(-5 * m.pi / 18, solnMatrix[i, 0], solnMatrix[i, 1], solnMatrix[i, 2],0.15,0.005,0.32)
                if abs(pose[0, 0] - rfk[0, 0]) < 0.0001:
                    result = np.vstack((result, solnMatrix[i, :]))
                    #print(result)
                    #print(len(result))
    
            # convertion of range of values from [-2pi, 2pi] to [ to 2pi]
            for i in range(len(result)):
                for j in range(len(result[i])):
    
                    if result  [i, j] < -m.pi:
                        
                        result[i, j] = round(float(result[i, j] + 2 * m.pi), 3)
                        
                
                    elif result[i, j] > m.pi :
                        result[i, j] = round(float(result[i, j] - 2*m.pi), 3)
                        
                    if result[i,1] <0:
                        result[i, 1] = round(float(result[i, 1] + 2 * m.pi), 3)
                    
    
                    result[i,j] = round(m.degrees(result[i,j]),3)
            
            return result
    def quaternion_rotation_matrix(Q):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = Q[0]
        q1 = Q[1]
        q2 = Q[2]
        q3 = Q[3]
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]])
                                
        return rot_matrix

    
    def IK(self,az,ax,ay,nz):
       '''       
       Parameters
       ----------
       az : Rotation Matrix Element (3rd row,3rd column)r33.
       ax : Rotation Matrix Element (1st row,3rd column)r13.
       ay : Rotation Matrix Element (2nd row,3rd column)r23.
       nz : Rotation Matrix Element (3rd row,1st column)r31.
       Returns
       -------
       q11 : q1 solution with +ve square root
       q12 : q1 solution with -ve square root
       q31 : q3 solution with +ve square root
       q32 : q3 solution with -ve square root
       q51 : q5 solution with +ve square root and q31
       q52 : q5 solution with -ve square root and q31
       q53 : q5 solution with +ve square root and q32
       q54 : q5 solution with -ve square root and q32
   
       * All the equations are derived from forward kinematics to get more 
       details of the equations refer to 'Local System-Probe Manipulator Documentation.docx'
       * values inside the square root is rounded since very small values 
       less than zero and approximately zero values creates math domain error.
       '''
       q = -5*m.pi/18;
       b3 = (az + m.cos(q)**(2))/(m.sin(q))**(2);
       if b3**(2) < 1:
           sqr31 =m.sqrt(round(1-b3**(2),15))
           q31 = m.atan2(sqr31,b3);
           q32 = m.atan2(-sqr31,b3);
           
           a1 = ax*m.sin(q); b1 = ay*m.sin(q); c1 = m.cos(q)*(1+az);
           sqr11 = m.sqrt(round(a1**(2)+b1**(2)-c1**(2),15))
           q11 = 2*m.atan2((b1-sqr11),(a1+c1));
           q12 = 2*m.atan2((b1+sqr11),(a1+c1));
           
           a51= m.cos(q)*m.sin(q)+(m.cos(q)*m.sin(q)*m.cos(q31));
           b51= m.sin(q)*m.sin(q31);
           c51= nz;
           a52= m.cos(q)*m.sin(q)+(m.cos(q)*m.sin(q)*m.cos(q32));
           b52= m.sin(q)*m.sin(q32);
           c52= nz;
           sqr51 = m.sqrt(round(a51**(2)+b51**(2)-c51**(2),15));
           sqr52 = m.sqrt(round(a52**(2)+b52**(2)-c52**(2),15))
           q51 = 2*m.atan2((b51+sqr51),(a51+c51));
           q52 = 2*m.atan2((b51-sqr51),(a51+c51));
           q53 = 2*m.atan2((b52+sqr52),(a52+c52));
           q54 = 2*m.atan2((b52-sqr52),(a52+c52));
           #print('Joint angles obtained from the input: GBA\n',q11, q12, q31, q32, q51, q52, q53, q54);
           return q11, q12, q31, q32, q51, q52, q53, q54
       else:
           return None
       

class PlebcEngine:
    # Example coordinates
    x = 0.9965
    y = 0.05835471249

    # This code is for theta 3
    #az = -0.9646
    
    # Calculate atan2
    #result = m.atan2(y, x)

    

    #print("fk from joint angles",lambi.ypr2rot(m.radians(0),m.radians(0),m.radians(0)))


    def quaternion_to_rotation_matrix(self,q):
        q = q/np.linalg.norm(q)
        w, x, y, z = q
        rotation_matrix = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])
        return rotation_matrix

    def JointAnglesFromquat(self,quat,prevq1,prevq3,prevq5):
        # Example quaternion [w, x, y, z]
        #quat = np.array([0.4663,0.5939,0.2460,-0.6077])

        # Convert quaternion to rotation matrix
        rotation_matrix = self.quaternion_to_rotation_matrix(quat)

        #print("rotation_matrix  \n",rotation_matrix)

        # This method of converting quat to rot is giving wrong values
        """
        def quaternion_rotation_matrix(Q):
        
            Covert a quaternion into a full three-dimensional rotation matrix.
        
            Input
            :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
        
            Output
            :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                    This rotation matrix converts a point in the local reference 
                    frame to a point in the global reference frame.
        
            # Extract the values from Q
            q0 = Q[0]
            q1 = Q[1]
            q2 = Q[2]
            q3 = Q[3]
            
            # First row of the rotation matrix
            r00 = 2 * (q0 * q0 + q1 * q1) - 1
            r01 = 2 * (q1 * q2 - q0 * q3)
            r02 = 2 * (q1 * q3 + q0 * q2)
            
            # Second row of the rotation matrix
            r10 = 2 * (q1 * q2 + q0 * q3)
            r11 = 2 * (q0 * q0 + q2 * q2) - 1
            r12 = 2 * (q2 * q3 - q0 * q1)
            
            # Third row of the rotation matrix
            r20 = 2 * (q1 * q3 - q0 * q2)
            r21 = 2 * (q2 * q3 + q0 * q1)
            r22 = 2 * (q0 * q0 + q3 * q3) - 1
            
            # 3x3 rotation matrix
            rot_matrix = np.array([[r00, r01, r02],
                                [r10, r11, r12],
                                [r20, r21, r22]])
                                    
            return rot_matrix



        quaternion = np.array([0,-0.64,0,0.77])  # Example quaternion
        """
        #rotation_matrix1 = quaternion_rotation_matrix(quaternion)
        #print("rotation_matrix 1 \n",rotation_matrix1)
        lambi = FK()
        q = -5*m.pi/18
        az = rotation_matrix[2,2];
        ax = rotation_matrix[0, 2];
        ay = rotation_matrix[1,2];
        nz = rotation_matrix[2,0];

        if round(az,3)<-1 or az>0.1736:
            return "no solution"
        
        b3 = (az + m.cos(q)**(2))/(m.sin(q))**(2);

        if b3**(2)>1:
            if az<0:
                z=m.radians(180)
            elif az>0 and az<0.18:
                z=m.radians(0)
            else:
                return "no solution"
        else:
            z = m.atan2(m.sqrt(1-b3**(2)),b3)

        # Convert the result to degrees if desired
        q3inDegrees = m.degrees(z)
        q3indegrees = m.degrees(-z)
        #print(b3)
        #print("atan2 =", z)
        #print("+Q3 in degrees =", q3inDegrees)  
        #print("-Q3 in degrees =", q3indegrees)
       
        if(abs(q3inDegrees)==180):
            SingularitySolns = self.SolveSingularity(quat,prevq1,prevq5,q3inDegrees)
            return self.checkikWithAngles(SingularitySolns,rotation_matrix)
        # This code is for theta1
        #ax = -0.0297
        #ay = 0.2620
        a1 = ax*m.sin(q) 
        b1 = ay*m.sin(q)
        c1 = m.cos(q)*(1+az)

        # Find out scenarios where a1**(2)+b1**(2)-c1**(2) is less than 0
        # for time being bypassing the sqr11 

        if a1**(2)+b1**(2)-c1**(2)<-0.001:
            print("unable to compute q1")
            q11=0
            q12=0
        else:
            if(a1**(2)+b1**(2)-c1**(2)<0):
                sqr11=0
            else:
                sqr11 = m.sqrt((a1**(2)+b1**(2)-c1**(2)))
            q11 = 2*m.atan2((b1-sqr11),(a1+c1))
            q12 = 2*m.atan2((b1+sqr11),(a1+c1))

        #print('possibel angles for theta1 ',m.degrees(q11)," second value ",m.degrees(q12))

        #This code is for theta5
        #nz =  -0.0297    
        a51= m.cos(q)*m.sin(q)+(m.cos(q)*m.sin(q)*m.cos(z));
        b51= m.sin(q)*m.sin(z);
        c51= nz;
        a52= m.cos(q)*m.sin(q)+(m.cos(q)*m.sin(q)*m.cos(-z));
        b52= m.sin(q)*m.sin(-z);
        c52= nz;
        sqr51 = m.sqrt(round(a51**(2)+b51**(2)-c51**(2),15));
        sqr52 = m.sqrt(round(a52**(2)+b52**(2)-c52**(2),15))
        q51 = 2*m.atan2((b51+sqr51),(a51+c51));
        q52 = 2*m.atan2((b51-sqr51),(a51+c51));
        q53 = 2*m.atan2((b52+sqr52),(a52+c52));
        q54 = 2*m.atan2((b52-sqr52),(a52+c52));

        #print('q51 of theta 5',m.degrees(q51))#," second value ",m.degrees(q12))
        #print('q52 of theta 5',m.degrees(q52))
        #print('q53 of theta 5',m.degrees(q53))
        #print('q54 of theta 5',m.degrees(q54))
        


        # filtering values based -pi to pi 
        q1List  = [*set([m.degrees(q11),m.degrees(q12)])]
        q3List  = [*set([m.degrees(z),m.degrees(-z)])]
        q5listBeforeFilter  = [*set([q51,q52,q53,q54])]


        q5listFiltered =[]
        for i in q5listBeforeFilter:
            a=m.degrees(i)
            if a<-180:
                if a+360<=180:
                    q5listFiltered.append(a+360)
            elif a>180: 
                if a-360>=-180:
                    q5listFiltered.append(a-360)
            else:
                q5listFiltered.append(a)
        #print("q5listFiltered",q5listFiltered)
        possiblesolution = []

        for i in q1List:
            for j in q3List:
                for k in q5listFiltered:
                    possiblesolution.append([i,j,k])

        return self.checkikWithAngles(possiblesolution,rotation_matrix)
       
    def quaternion_to_euler(self,quaternion):
    # Normalize the quaternion
        quaternion = quaternion / np.linalg.norm(quaternion)

        # Create a rotation object
        rotation = Rotation.from_quat(quaternion)

        # Convert to Euler angles
        euler = rotation.as_euler('xyz')

        return euler


    def checkikWithAngles(self,possiblesolution,rotation_matrix):
        #print(possiblesolution)

        #for i in possiblesolution:
            #print("FwfKin with soln index = ",i,"\n",lambi.fwdKin(q,m.radians(i[0]),m.radians(i[1]),m.radians(i[2]),0.15,0.00,0.32)) 

        #Now comparing matrix with fwd kinematic
        numOfSolutions = len(possiblesolution)
        #print(numOfSolutions)
        indexFkDict ={}
        lambi = FK()
        q=m.radians(-50);
        for i in range(0,numOfSolutions):
            indexFkDict[i]=lambi.fwdKin(q,m.radians(possiblesolution[i][0]),m.radians(possiblesolution[i][1]),m.radians(possiblesolution[i][2]),0.15,0.00,0.32)

        ListofIndexs = []
        for i in range(0,numOfSolutions):
            ListofIndexs.append(i)
            
        if len(ListofIndexs)>1:# checking (0,0)
            for i in ListofIndexs[:]:
                if abs(indexFkDict[i][0,0]-rotation_matrix[0,0])<0.001:
                    continue
                else:
                    ListofIndexs.remove(i)

        if len(ListofIndexs)>1:# checking (1,0)
            for i in ListofIndexs[:]:
                if abs(indexFkDict[i][1,0]-rotation_matrix[1,0])<0.001:
                    continue
                else:
                    ListofIndexs.remove(i)

        if len(ListofIndexs)>1: # checking (2,0)
            for i in ListofIndexs[:]:
                if abs(indexFkDict[i][2,0]-rotation_matrix[2,0])<0.001:
                    continue
                else:
                    ListofIndexs.remove(i)
        # Comparing 2nd column
        if len(ListofIndexs)>1: # checking (0,1)
            for i in ListofIndexs[:]:
                if abs(indexFkDict[i][0,1]-rotation_matrix[0,1])<0.001:
                    continue
                else:
                    ListofIndexs.remove(i)

        if len(ListofIndexs)>1: # checking (1,1)
            for i in ListofIndexs[:]:
                if abs(indexFkDict[i][1,1]-rotation_matrix[1,1])<0.001:
                    continue
                else:
                    ListofIndexs.remove(i)
                    
        if len(ListofIndexs)>1: # checking (2,1)
            for i in ListofIndexs[:]:
                if abs(indexFkDict[i][2,1]-rotation_matrix[2,1])<0.001:
                    continue
                else:
                    ListofIndexs.remove(i)
        # comparing 3rd colum   
        if len(ListofIndexs)>1: # checking (0,2)
            for i in ListofIndexs[:]:
                if abs(indexFkDict[i][0,2]-rotation_matrix[0,2])<0.001:
                    continue
                else:
                    ListofIndexs.remove(i)

        if len(ListofIndexs)>1: # checking (1,2)
            for i in ListofIndexs[:]:
                if abs(indexFkDict[i][1,2]-rotation_matrix[1,2])<0.001:
                    continue
                else:
                    ListofIndexs.remove(i)
                
        if len(ListofIndexs)>1: # checking (2,2)
            for i in ListofIndexs[:]:
                if abs(indexFkDict[i][2,2]-rotation_matrix[2,2])<0.001:
                    continue
                else:
                    ListofIndexs.remove(i)

        #print("Final no.of sets of joint angles obtains",len(ListofIndexs))

        #for i in ListofIndexs:
            #print("pos soln after r11 filter ",possiblesolution[i])
            #print("fk of above pos soln \n",indexFkDict[i])
        if len(ListofIndexs)<1:
            return "no solution"
        
        FinalJointAngles=[]
        for i in range(0,2):
            if len(ListofIndexs)>i:
                FinalJointAngles.append(possiblesolution[ListofIndexs[i]])
        
        for i in FinalJointAngles:
            if i[0]<0:
                i[0]=i[0]+360
            if i[1]<0:
                i[1]=i[1]+360
            if i[2]<0:
                i[2]=i[2]+360
    
        return FinalJointAngles

        #print("Fwd kin  with 0,0,0 = \n",lambi.fwdKin(q,0,0,0,0.15,0.00,0.32)) 

        # now try to compare with forward kinematic with these joint angles
        """"
        print(" q,q11,z,q51 \n",lambi.fwdKin(q,q11,z,q51,0.15,0.00,0.32)) #no match 
        print("q,q11,z,q52,\n",lambi.fwdKin(q,q11,z,q52,0.15,0.00,0.32)) # r11 match r21 no match
        print("q,q11,z,q53,\n",lambi.fwdKin(q,q11,z,q53,0.15,0.00,0.32)) # r11 slight match r31 no match
        print("q,q11,z,q54,\n",lambi.fwdKin(q,q11,z,q54,0.15,0.00,0.32)) # r11 no match
        print("q,q12,z,q51,\n",lambi.fwdKin(q,q12,z,q51,0.15,0.00,0.32))#ANSWER # full match
        print("q,q12,z,q52,\n",lambi.fwdKin(q,q12,z,q52,0.15,0.00,0.32)) # r11 no match 
        print("q,q12,z,q53,\n",lambi.fwdKin(q,q12,z,q53,0.15,0.00,0.32)) # r11 no match
        print("q,q12,z,q54,\n",lambi.fwdKin(q,q12,z,q54,0.15,0.00,0.32)) # All match # answer

print(" q,q11,-z,q51 \n",lambi.fwdKin(q,q11,-z,q51,0.15,0.00,0.32)) #r11 no match 
print("q,q11,-z,q52,\n",lambi.fwdKin(q,q11,-z,q52,0.15,0.00,0.32))  # r11 no match
print("q,q11,-z,q53,\n",lambi.fwdKin(q,q11,-z,q53,0.15,0.00,0.32))  #r11 slight match , r21 no match
print("q,q11,-z,q54,\n",lambi.fwdKin(q,q11,-z,q54,0.15,0.00,0.32))  # r11 no match 
print("q,q12,-z,q51,\n",lambi.fwdKin(q,q12,-z,q51,0.15,0.00,0.32))# r11 match r12 no match
print("q,q12,-z,q52,\n",lambi.fwdKin(q,q12,-z,q52,0.15,0.00,0.32)) # r11 no match
print("q,q12,-z,q53,\n",lambi.fwdKin(q,q12,-z,q53,0.15,0.00,0.32)) # r11 no match
print("q,q12,-z,q54,\n",lambi.fwdKin(q,q12,-z,q54,0.15,0.00,0.32)) # r11 match r21 no match
"""

    def SolveSingularity(self,quat,prevq1,prevq5,q3):
        
        # Example usage
        #quaternion = np.array([0.707, 0, 0.707, 0])  # Quaternion (w, x, y, z)
        euler_angles = self.quaternion_to_euler(quat)
        #print("Euler angles (in radians):", euler_angles)
        #print("Euler angles (in degrees):",m.degrees(euler_angles[0]),m.degrees(euler_angles[1]),m.degrees(euler_angles[2]))
        eulerZ = m.degrees(euler_angles[0])
        q1=prevq5-eulerZ
        q5=(eulerZ+prevq1)
        possibleSolns = [[q1,q3,prevq5],[prevq1,q3,q5]]
        return possibleSolns
        
#torus = PlebcEngine()
#testquat = np.array([0.02, 0.85,-0.52,-0.01]) 
#print("Computed joint angles from ik",torus.JointAnglesFromquat(testquat))


#print(lambi.ypr2rot(m.radians(-180),m.radians(80),m.radians(0)))




#(-120, 80, 0)	0.38 < 0.56, 0.32, -0.66 >
# Example usage
