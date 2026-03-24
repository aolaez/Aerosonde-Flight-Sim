import math
from ..Containers import States
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

class VehicleDynamicsModel:
    def __init__(self, dT=VPC.dT):
        # store timestep, state, state derivative
        self.dT = dT
        self.state = States.vehicleState()
        self.stateDot = States.vehicleState()
        self.forces = None


    def reset(self):
        # Resets state and derivative to zero
        self.state = States.vehicleState()
        self.stateDot = States.vehicleState()


    def getVehicleState(self):
        return self.state

    def setVehicleState(self, newState):
        # Replace state with passed in state
        self.state = newState

    def getVehicleDerivative(self):
        return self.stateDot

    def setVehicleDerivative(self, newDt):
        # Replace derivateive with passed in state
        self.stateDot = newDt

    def Update(self, forces):
        self.derivative(self.state, forces)
        self.state = self.IntegrateState(self.dT, self.state, self.stateDot)
    
    def derivative(self, state, forces):
        X = state
        Fx = forces.Fx
        Fy = forces.Fy
        Fz = forces.Fz
        Mx = forces.Mx
        My = forces.My
        Mz = forces.Mz
        m  = VPC.mass


        # Pos. derivative from page 30 of Beard
        #Pned_dot = R^T[u,v,w]
        inertial_velocity = MatrixMath.multiply(MatrixMath.transpose(X.R), [[X.u], [X.v], [X.w]])
        Pn_dot = inertial_velocity[0][0]
        Pe_dot = inertial_velocity[1][0]
        Pd_dot = inertial_velocity[2][0]

        # body frame velocity derivative, eq from page 33
        # uvw_dot = [rv-qw, pw-ru, qu-pv] + 1/m([fx, fy, fz])
        u_dot = (X.r*X.v - X.q*X.w) + (Fx/m)
        v_dot = (X.p*X.w - X.r*X.u) + (Fy/m)
        w_dot = (X.q*X.u - X.p*X.v) + (Fz/m)

        # Rotation matrix derivative from lecture
        om = [
            [0, -X.r, X.q],
            [X.r, 0, -X.p],
            [-X.q, X.p, 0]
        ]
        not_R_dot = MatrixMath.multiply(om, X.R)
        R_dot = MatrixMath.scalarMultiply(-1, not_R_dot)

        # angluar rate dt's, formula from lecture
        omega = [[X.p], [X.q], [X.r]]
        M  = [[Mx], [My], [Mz]]
        Jomega = MatrixMath.multiply(VPC.Jbody, omega)

        omega_Jomega = [
            [X.q * Jomega[2][0] - X.r * Jomega[1][0]],
            [X.r * Jomega[0][0] - X.p * Jomega[2][0]],
            [X.p * Jomega[1][0] - X.q * Jomega[0][0]]
        ]

        paren = MatrixMath.subtract(M, omega_Jomega)
        omega_dot = MatrixMath.multiply(VPC.JinvBody, paren)

        p_dot = omega_dot[0][0]
        q_dot = omega_dot[1][0]
        r_dot = omega_dot[2][0]

        # Euler angle derivatives
        psi, theta, phi = Rotations.dcm2Euler(X.R)

        phi_theta_matrix = [
            [1, math.sin(phi)*math.tan(theta), math.cos(phi)*math.tan(theta)],
            [0, math.cos(phi), -1*math.sin(phi)],
            [0, math.sin(phi)/(math.cos(theta)), math.cos(phi)/(math.cos(theta))]
        ]

        euler_dot = MatrixMath.multiply(phi_theta_matrix, omega)
        
        roll_dot  = euler_dot[0][0]
        pitch_dot = euler_dot[1][0]
        yaw_dot   = euler_dot[2][0]

        self.stateDot = States.vehicleState(
            pn = Pn_dot,
            pe = Pe_dot,
            pd = Pd_dot,
            u  = u_dot,
            v  = v_dot,
            w  = w_dot,
            p  = p_dot,
            q  = q_dot,
            r  = r_dot,
            yaw = yaw_dot,
            pitch = pitch_dot,
            roll = roll_dot
        )

        self.stateDot.R = R_dot

        return self.stateDot
    
    def ForwardEuler(self, dT, state, dot):
        #Formula from class
        #X_t1 = X_t0 + dot * dT
        newState = States.vehicleState(
            pn = state.pn + dot.pn * dT,
            pe = state.pe + dot.pe * dT,
            pd = state.pd + dot.pd * dT,
            u = state.u + dot.u * dT,
            v = state.v + dot.v * dT,
            w = state.w + dot.w * dT,
            p = state.p + dot.p * dT,
            q = state.q + dot.q * dT,
            r = state.r + dot.r * dT,
            yaw = state.yaw + dot.yaw * dT,
            pitch = state.pitch + dot.pitch * dT,
            roll = state.roll + dot.roll * dT
        )
        
        return newState
    
    def Rexp(self, dT, state, dot):
            # eq 42 from attitude cheat sheet
            p = state.p + dot.p * (dT/2.0)
            q = state.q + dot.q * (dT/2.0)
            r = state.r + dot.r * (dT/2.0)
            
            # ||omega||
            magnitude_omega = math.sqrt(p**2 + q**2 + r**2)
            omega = [
                [0, -r, q],
                [r, 0, -p],
                [-q, p, 0]
            ]
            
            # check nominal
            if magnitude_omega < 0.1: # eqs 31 and 32
                norm_sq = magnitude_omega**2
                dT_sq = dT**2
                
                transcend1 = dT - (dT**3 * norm_sq)/6.0 + (dT**5 * norm_sq**2)/120.0
                transcend2 = dT_sq/2.0 - (dT**4 * norm_sq)/24.0 + (dT**6 * norm_sq**2)/720.0
            else:
                # eq 29 from cheat sheet
                transcend1 = math.sin(magnitude_omega * dT) / magnitude_omega
                transcend2 = (1.0 - math.cos(magnitude_omega * dT)) / (magnitude_omega**2)
            
            omega_sq = MatrixMath.multiply(omega, omega)
            identity = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
            
            term1 = MatrixMath.scalarMultiply(-transcend1, omega)
            term2 = MatrixMath.scalarMultiply(transcend2, omega_sq)
            
            sum = MatrixMath.add(identity, term1)
            Rexp_matrix = MatrixMath.add(sum, term2) # result of eq 29
            
            return Rexp_matrix
    
    def IntegrateState(self, dT, state, dot):
        # go forward a dt
        # ForwardEuler for position, velocity, and rotation rates
        newState = self.ForwardEuler(dT, state, dot)
        
        # Find rot matrx R and euler angles
        Rexp_matrix = self.Rexp(dT, state, dot)
        newState.R = MatrixMath.multiply(Rexp_matrix, state.R)
        newState.yaw, newState.pitch, newState.roll = Rotations.dcm2Euler(newState.R)
        
        # keep airspeed from last timestep ang calculate chi course angle
        newState.alpha = state.alpha
        newState.beta = state.beta
        newState.Va = state.Va
        newState.chi = math.atan2(dot.pe, dot.pn)
        
        return newState
