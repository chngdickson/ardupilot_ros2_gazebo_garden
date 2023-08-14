import numpy as np
from typing import List

class Quin_segment(object):
    def __init__(self, CX: list,CY:list,CZ:list,T:float):
        if len(CX) != 6 or len(CY) != 6 or len(CZ) != 6:
            raise ValueError("The length of the coefficient list must be 6")
        if T <= 0:
            raise ValueError("Time must be greater than 0")
        self.CX = CX
        self.CY = CY
        self.CZ = CZ
        self.T = T
        self.all_var = [CX, CY, CZ, T]
    
    def __repr__(self):
        return repr(self.all_var)
    
    def __getitem__(self, index):
        return self.all_var[index]
    
class PolynomialTraj():
    def __init__(self):
        self.traj_passed : List[Quin_segment] = []
        pass
    
    def a_Quintic_segment_generation(self, 
        start_pt:np.ndarray, start_vel:np.ndarray, start_acc:np.ndarray,
        end_pt:np.ndarray,   end_vel:np.ndarray,   end_acc:np.ndarray,
         t:float)-> List[Quin_segment]:
        """
        This function generates coefficients of a Quintic Polynomial n^5
        in direction
            - x
            - y
            - z 
        
        The coefficients are generated using the following parameters:
        :param. initial 
            - distance
            - velocity
            - acceleration
        final 
            - distance
            - velocity
            - acceleration
        time
        
        :return: coefficients of the polynomial
        
        What does the coefficients mean? Idk yet
        but the idea is, with these values, we can get the position, velocity, and acceleration
        using any given time as a variable.
        
        
        For "position" at time t = 2.5, we can just plug in 2.5 into the
        polynomial function and get the position at that time.
        
        p(t) = cx[0]*t^5 + cx[1]*t^4 + cx[2]*t^3 + cx[3]*t^2 + cx[4]*t + cx[5]
        p(2) = cx[0]*2^5 + cx[1]*2^4 + cx[2]*2^3 + cx[3]*2^2 + cx[4]*2 + cx[5]
        
        
        For velocity, at any given time, 
        equation 0 : derivative Equation
        equation 1 : velocity simplified 
        
        eq_v(0) : v(t) = d/dt [p(t)] = d/dt [cx[0]*t^5 + cx[1]*t^4 + cx[2]*t^3 + cx[3]*t^2 + cx[4]*t + cx[5]]
        eq_v(1) : v(t) = 5*cx[0]*t^4 + 4*cx[1]*t^3 + 3*cx[2]*t^2 + 2*cx[3]*t + cx[4]
        
        
        For Acceleration, at any given time,
        equation 0 : derivative Equation
        equation 1 : acceleration simplified
        
        eq_a(0) : a(t) = d/dt [v(t)] = d/dt [5*cx[0]*t^4 + 4*cx[1]*t^3 + 3*cx[2]*t^2 + 2*cx[3]*t + cx[4]]
        eq_a(1) : a(t) = 20*cx[0]*t^3 + 12*cx[1]*t^2 + 6*cx[2]*t + 2*cx[3]

        """
        C, Crow = np.zeros((6,6)),np.zeros((6,1))
        Bx,By,Bz = np.zeros(6), np.zeros(6), np.zeros(6)
        C[0, 5] = 1
        C[1, 4] = 1
        C[2, 3] = 2
        Crow = np.array([[t**5, t**4, t**3, t**2, t, 1]])
        C[3, :] = Crow

        Crow = np.array([[5 * t**4, 4 * t**3, 3 * t**2, 2 * t, 1, 0]])
        C[4, :] = Crow

        Crow = np.array([[20 * t**3, 12 * t**2, 6 * t, 2, 0, 0]])
        C[5, :] = Crow
        
        Bx = np.array([start_pt[0], start_vel[0], start_acc[0], end_pt[0], end_vel[0], end_acc[0]])
        By = np.array([start_pt[1], start_vel[1], start_acc[1], end_pt[1], end_vel[1], end_acc[1]])
        Bz = np.array([start_pt[2], start_vel[2], start_acc[2], end_pt[2], end_vel[2], end_acc[2]])

        # Solve for Cofx, Cofy, and Cofz
        Cofx = np.linalg.solve(C, Bx)
        Cofy = np.linalg.solve(C, By)
        Cofz = np.linalg.solve(C, Bz)
        
        # Convert Cofx, Cofy, and Cofz to lists
        cx = Cofx.tolist()
        cy = Cofy.tolist()
        cz = Cofz.tolist()
        
        # # print cx cy cz on separate lines
        # print(cx)
        # print(cy)
        # print(cz)
        
        self.traj_passed.append(Quin_segment(CX=cx, CY=cy, CZ=cz, T=t))
        return self.traj_passed
    
    def b_evaluate_pos(self, t: float) -> np.ndarray:
        """
        Calculate Position at time t
        For polynomial types, Cubic, Quintic, and Sextic
        
        Using Quintic example
        p(t) = cx[0]*t^5 + cx[1]*t^4 + cx[2]*t^3 + cx[3]*t^2 + cx[4]*t + cx[5]
        
        Attributes :
            cx : [cx[0], cx[1], cx[2], cx[3], cx[4], cx[5]]
            tv : [t^5, t^4, t^3, t^2, t^1, t^0]
        : param t: time
        : return [x,y,z]: position at time t
        """
        # print("evaluate_pos")
        # Determine which segment based on the time.
        idx = 0
        while self.traj_passed[idx].T + 1e-4 < t:
            t -= self.traj_passed[idx].T
            idx += 1

        # evaluation
        traj_segment = self.traj_passed[idx]
        order = len(traj_segment.CX)
        cx = np.array(traj_segment.CX)
        cy = np.array(traj_segment.CY)
        cz = np.array(traj_segment.CZ)
        tv = np.array([pow(t, i) for i in range(order)][::-1])

        pt = np.array([tv.dot(cx), tv.dot(cy), tv.dot(cz)])
        return pt
    
    def b_evaluate_vel(self, t: float) -> np.ndarray:
        """
        Calculate Velocity at time t
        For polynomial types, Cubic, Quintic, and Sextic
        
        Using Quintic example
        v(t) = 5*cx[0]*t^4 + 4*cx[1]*t^3 + 3*cx[2]*t^2 + 2*cx[3]*t + cx[4]
        
        Attributes :
            vx : [5*cx[0], 4*cx[1], 3*cx[2], 2*cx[3], cx[4]]
            tv : [t^4, t^3, t^2, t, 1]
        : param t: time
        : return [vx,vy,vz]: velocity at time t
        """
        # Determine which segment based on the time.
        # print("evaluate_vel")
        idx = 0
        while self.traj_passed[idx].T + 1e-4 < t:
            t -= self.traj_passed[idx].T
            idx += 1
        traj_segment = self.traj_passed[idx]
        
        # Initialize Coefficients
        cx = np.array(traj_segment.CX)
        cy = np.array(traj_segment.CY)
        cz = np.array(traj_segment.CZ)
        order = len(cx)
        vx, vy, vz = np.zeros(order - 1), np.zeros(order - 1), np.zeros(order - 1)
        
        # Calculate Velocity
        for i in range(order-1):
            vx[i] = float(i + 1) * cx[order - 2 - i]
            vy[i] = float(i + 1) * cy[order - 2 - i]
            vz[i] = float(i + 1) * cz[order - 2 - i]

        ts = t
        tv = np.array([pow(ts, i) for i in range(order - 1)])
        vel = np.array([tv.dot(vx), tv.dot(vy), tv.dot(vz)])
        return vel
    
    def b_evaluate_acc(self, t: float) -> np.ndarray:
        """
        Calculate Acceleration at time t
        For polynomial types, Cubic, Quintic, and Sextic
        
        Using Quintic example
        a(t) = 20*cx[0]*t^3 + 12*cx[1]*t^2 + 6*cx[2]*t + 2*cx[3]
        
        Attributes :
            ax : [20*cx[0], 12*cx[1], 6*cx[2], 2*cx[3]]
            tv : [t^3, t^2, t, 1]
        : param t: time
        : return [ax,ay,az]: acceleration at time t
        """
        # Determine which segment based on the time.
        # print("evaluate_acc")
        idx = 0
        while self.traj_passed[idx].T + 1e-4 < t:
            t -= self.traj_passed[idx].T
            idx += 1
        traj_segment = self.traj_passed[idx]
        
        # Initialize Coefficients
        cx = np.array(traj_segment.CX)
        cy = np.array(traj_segment.CY)
        cz = np.array(traj_segment.CZ)
        order = len(cx)
        ax, ay, az = np.zeros(order - 2), np.zeros(order - 2), np.zeros(order - 2)
        
        # Calculate Acceleration
        for i in range(order-2):
            ax[i] = float(i + 2) * (i + 1) * cx[order - 3 - i]
            ay[i] = float(i + 2) * (i + 1) * cy[order - 3 - i]
            az[i] = float(i + 2) * (i + 1) * cz[order - 3 - i]
            
        tv = np.array([pow(t, i) for i in range(order - 2)])
        acc = np.array([tv.dot(ax), tv.dot(ay), tv.dot(az)])
        return acc
    
    def b_PlanMinSnapTraj(self, Pos, start_vel, end_vel, start_acc, end_acc, Time)->List[Quin_segment]:
        """
        Calculate Minimum Snap Trajectory
        Generate a Smooth Trajectory that passes through all the waypoints using prior information of segments.
        
        Using Quintic example
        
        Attributes :
        : param pos         : [x,y,z] 
        : param start_vel   : [vx,vy,vz]
        : param end_vel     : [vx,vy,vz]
        : param start_acc   : [ax,ay,az]
        : param end_acc     : [ax,ay,az]
        : param Times       : list of[Ts]
         
        : return List of Quintic Segment
        """
        seg_num = Time.size
        poly_coeff = np.zeros((seg_num, 3 * 6))
        Px = np.zeros(6 * seg_num)
        Py = np.zeros(6 * seg_num)
        Pz = np.zeros(6 * seg_num)

        def factorial(x):
            fac = 1
            for i in range(x, 0, -1):
                fac *= i
            return fac

        ###-----------Calculate end point Derivative-----------------###
        Dx = np.zeros(seg_num * 6)
        Dy = np.zeros(seg_num * 6)
        Dz = np.zeros(seg_num * 6)

        for k in range(seg_num):
            Dx[k * 6] = Pos[0, k]
            Dx[k * 6 + 1] = Pos[0, k + 1]
            
            Dy[k * 6] = Pos[1, k]
            Dy[k * 6 + 1] = Pos[1, k + 1]
            
            Dz[k * 6] = Pos[2, k]
            Dz[k * 6 + 1] = Pos[2, k + 1]

            if k == 0:
                Dx[k * 6 + 2] = start_vel[0]
                Dy[k * 6 + 2] = start_vel[1]
                Dz[k * 6 + 2] = start_vel[2]

                Dx[k * 6 + 4] = start_acc[0]
                Dy[k * 6 + 4] = start_acc[1]
                Dz[k * 6 + 4] = start_acc[2]
            elif k == seg_num - 1:
                Dx[k * 6 + 3] = end_vel[0]
                Dy[k * 6 + 3] = end_vel[1]
                Dz[k * 6 + 3] = end_vel[2]

                Dx[k * 6 + 5] = end_acc[0]
                Dy[k * 6 + 5] = end_acc[1]
                Dz[k * 6 + 5] = end_acc[2]        
        
        ###-----------Mapping Matrix A-----------------###
        Ab = np.zeros((6, 6))
        A = np.zeros((seg_num * 6, seg_num * 6))

        for k in range(seg_num):
            Ab = np.zeros((6, 6))
            for i in range(3):
                Ab[2 * i, i] = factorial(i)
                for j in range(i, 6):
                    Ab[2 * i + 1, j] = factorial(j) / factorial(j - i) * pow(Time[k], j - i)
            A[k * 6:k * 6 + 6, k * 6:k * 6 + 6] = Ab
        
        # print(f"A: {A.T} \nA.shape: {A.shape}")

        ###-----------Selection Matrix C-----------------###
        num_f = 2 * seg_num + 4 # 3 + 3 + (seg_num - 1) * 2 = 2m + 4
        num_p = 2 * seg_num - 2 # (seg_num - 1) * 2 = 2m - 2
        num_d = 6 * seg_num
        Ct = np.zeros((num_d, num_f + num_p))
        Ct[0, 0] = 1
        Ct[2, 1] = 1
        Ct[4, 2] = 1
        Ct[1, 3] = 1
        Ct[3, 2 * seg_num + 4] = 1
        Ct[5, 2 * seg_num + 5] = 1

        Ct[6 * (seg_num - 1) + 0, 2 * seg_num + 0] = 1
        Ct[6 * (seg_num - 1) + 1, 2 * seg_num + 1] = 1
        Ct[6 * (seg_num - 1) + 2, 4 * seg_num + 0] = 1
        Ct[6 * (seg_num - 1) + 3, 2 * seg_num + 2] = 1
        Ct[6 * (seg_num - 1) + 4, 4 * seg_num + 1] = 1
        Ct[6 * (seg_num - 1) + 5, 2 * seg_num + 3] = 1
        
        for j in range(2,seg_num):
            Ct[6 * (j - 1) + 0,         2       + 2 * (j-1) + 0] = 1
            Ct[6 * (j - 1) + 1,         2       + 2 * (j-1) + 1] = 1
            Ct[6 * (j - 1) + 2, (2 * seg_num +4)+ 2 * (j-2) + 0] = 1
            Ct[6 * (j - 1) + 3, (2 * seg_num +4)+ 2 * (j-1) + 0] = 1
            Ct[6 * (j - 1) + 4, (2 * seg_num +4)+ 2 * (j-2) + 1] = 1
            Ct[6 * (j - 1) + 5, (2 * seg_num +4)+ 2 * (j-1) + 1] = 1

        C = Ct.T

        Dx1, Dy1, Dz1 = np.dot(C, Dx), np.dot(C, Dy), np.dot(C, Dz)
        ###---------minimum snap matrix -----------------###
        Q = np.zeros((num_d, num_d))
        for k in range(0, seg_num):
            for i in range(3,6):
                for j in range(3,6):
                    Q[k*6+i, k*6+j] = i*(i-1)*(i-2)*j*(j-1)*(j-2)/(i+j-5)*pow(Time[k],(i+j-5))
        # print(f"Q: {Q}")

        ###-----------------R Matrix--------------------###
        Ct_transpose = np.transpose(Ct)
        R = np.dot(C,np.dot(np.dot(np.dot(np.linalg.inv(A.T),Q),np.linalg.inv(A)),Ct))
        
        Dxf, Dyf, Dzf = Dx1[0:2*seg_num+4], Dy1[0:2*seg_num+4], Dz1[0:2*seg_num+4]
        Rff = R[          0:2*seg_num+4                   ,           0:2*seg_num+4                  ]
        Rfp = R[          0:2*seg_num+4                   , 2*seg_num+4:((2*seg_num+4)+(2*seg_num-2))]
        Rpf = R[2*seg_num+4:((2*seg_num+4)+(2*seg_num-2)) ,           0:2*seg_num+4                  ]
        Rpp = R[2*seg_num+4:((2*seg_num+4)+(2*seg_num-2)) , 2*seg_num+4:((2*seg_num+4)+(2*seg_num-2))]
        
        ###----------closed form solution -------------###
        Dup = -np.dot(np.linalg.inv(Rpp), Rpf)
        Dxp = np.dot(Dup, Dxf)
        Dyp = np.dot(Dup, Dyf)
        Dzp = np.dot(Dup, Dzf)
        
        Dx1[2*seg_num+4:((2*seg_num+4)+(2*seg_num-2))] = Dxp
        Dy1[2*seg_num+4:((2*seg_num+4)+(2*seg_num-2))] = Dyp
        Dz1[2*seg_num+4:((2*seg_num+4)+(2*seg_num-2))] = Dzp
        
        Pup = np.dot(np.linalg.inv(A),Ct)
        Px = np.dot(Pup, Dx1)
        Py = np.dot(Pup, Dy1)
        Pz = np.dot(Pup, Dz1)
        
        for i in range(seg_num):
            poly_coeff[i, 0:6] = Px[i * 6:i * 6 + 6]
            poly_coeff[i, 6:12] = Py[i * 6:i * 6 + 6]
            poly_coeff[i, 12:18] = Pz[i * 6:i * 6 + 6]
        
        ###-----------use polynomials--------------###
        for i in range(poly_coeff.shape[0]):
            cx, cy, cz = np.zeros(6), np.zeros(6), np.zeros(6)
            for j in range(0,6):
                cx[j], cy[j], cz[j]= poly_coeff[i][j], poly_coeff[i][j+6], poly_coeff[i][j+12]
            cx, cy, cz = cx[::-1], cy[::-1], cz[::-1]
            ts = Time[i]

            self.traj_passed.append(Quin_segment(CX=cx, CY=cy, CZ=cz, T=ts))
        return self.traj_passed    
    
    def c_calc_Total_Acc_cost(self) -> float:
        cost = 0.0
        for i in range(len(self.traj_passed)):
            traj_segment = self.traj_passed[i]
            order = len(traj_segment.CX)
            um = np.zeros(3)
            um[0], um[1], um[2] = 2 * traj_segment.CX[order - 3], 2 * traj_segment.CY[order - 3], 2 * traj_segment.CZ[order - 3]
            cost += pow(np.linalg.norm(um),2) * traj_segment.T
        return cost
    
    def c_getJerk(self) -> float:
        jerk = 0.0
        
        for seg in range(len(self.traj_passed)):
            traj_segment = self.traj_passed[seg]
            order = len(traj_segment.CX)
            cxv, cyv, czv = np.zeros(order), np.zeros(order), np.zeros(order)
            for j in range(order):
                cxv[j] , cyv[j], czv[j] = traj_segment.CX[order - 1 -j], traj_segment.CY[order - 1 -j], traj_segment.CZ[order - 1 -j]
            
            # jerk matrix
            jerk_mat = np.zeros((order,order))
            for i in range(order):
                for j in range(order):
                    if (i+j-5) != 0:
                        jerk_mat[i][j] = float(i*(i-1)*(i-2)*j*(j-1)*(j-2)*pow(traj_segment.T, i+j-5)) / float((i+j-5))
            jerk += (cxv.T* jerk_mat*cxv)[0][0]
            jerk += (cyv.T* jerk_mat*cyv)[0][0]
            jerk += (czv.T* jerk_mat*czv)[0][0]
            
        return jerk
        
if __name__ == '__main__':
    test = PolynomialTraj()
    start_pt = -18,0,0
    start_vel = 0,0,0
    start_acc = 0,0,0
    end_pt = -10.5,0.706,0.565
    end_vel = 2.01,0.189,0.151
    end_acc = 0,0,0
    time = 4.44
    cx = [ 0.0106, -0.129, 0.449, -6.66e-16, 3.33e-15,-18]
    cy = [0.000996, -0.0121, 0.0423,-3.21e-17,1.34e-16,-5.75e-16]
    cz = [0.000798,-0.00972,0.0339,-1.26e-17,9.55e-17,-4.81e-16]
    cool = test.a_Quintic_segment_generation(start_pt, start_vel, start_acc, end_pt, end_vel, end_acc, time)
    # print(test.b_evaluate_pos(4.4))
    # print(test.b_evaluate_vel(0.3))
    # print(test.b_evaluate_acc(0.6))
    # print(test.c_calc_Total_Acc_cost())
    # print(test.c_getJerk())
    POS = np.array([[-18,0,0],[-14.7, 0.908, 0.333],[-11.4,1.82,0.667],[-8.09 , 2.72 , 1]])
    POS = np.transpose(POS)
    start_vel = np.array([0,0,0])
    start_acc = np.array([0,0,0])
    end_vel = np.array([0,0,0])
    end_acc = np.array([0,0,0])
    time_stuff = np.array([3.44,1.72,3.44])
    damn = test.b_PlanMinSnapTraj(POS, start_vel, end_vel, start_acc, end_acc, time_stuff)
    print(damn)