import math
import random
from ..Containers import States
from ..Utilities import MatrixMath
from ..Constants import VehiclePhysicalConstants as VPC

class WindModel:
    def __init__(self, dT=VPC.dT, Va=VPC.InitialSpeed, drydenParameters=VPC.DrydenNoWind):
        self.dT = dT
        self.Va = Va
        self.drydenParameters = drydenParameters

        self.Wind = States.windState()

        # states for turbulence at each axis
        self.x_u_vector = [[0.0]]
        self.x_v_vector = [[0.0], [0.0]]
        self.x_w_vector = [[0.0], [0.0]]

        # Tfer function matrices for each axis
        self.Phi_u = [[0.0]]
        self.Gamma_u = [[0.0]]
        self.H_u = [[0.0]]
        
        self.Phi_v = [[0.0, 0.0], [0.0, 0.0]]
        self.Gamma_v = [[0.0], [0.0]]
        self.H_v = [[0.0, 0.0]]
        
        self.Phi_w = [[0.0, 0.0], [0.0, 0.0]]
        self.Gamma_w = [[0.0], [0.0]]
        self.H_w = [[0.0, 0.0]]

        self.CreateDrydenTransferFns(dT, Va, drydenParameters)
        
        return
    

    def reset(self):
        self.Wind = States.windState()

        self.x_u_vector = [[0.0]]
        self.x_v_vector = [[0.0], [0.0]]
        self.x_w_vector = [[0.0], [0.0]]

        self.CreateDrydenTransferFns(self.dT, self.Va, self.drydenParameters)
        return
    

    def getWind(self):
        return self.Wind

    def setWind(self, wind):
        self.Wind = wind
        return

    def setWindModelParameters(self, Wn = 0.0, We = 0.0, Wd = 0.0, drydenParameters=VPC.DrydenNoWind):
        # set parameters then recreate Tfer functions
        self.Wind.Wn = Wn
        self.Wind.We = We
        self.Wind.Wd = Wd
        self.CreateDrydenTransferFns(self.dT, self.Va, drydenParameters)

    def getDrydenTransferFns(self):
        return (self.Phi_u, self.Gamma_u, self.H_u,
                self.Phi_v, self.Gamma_v, self.H_v,
                self.Phi_w, self.Gamma_w, self.H_w)
    

    def CreateDrydenTransferFns(self, dT, Va, drydenParameters):
        L_u = drydenParameters.Lu
        L_v = drydenParameters.Lv
        L_w = drydenParameters.Lw
        sigma_u = drydenParameters.sigmau
        sigma_v = drydenParameters.sigmav
        sigma_w = drydenParameters.sigmaw
        
        if drydenParameters == VPC.DrydenNoWind:
            self.Phi_u = [[1]]
            self.Gamma_u = [[0]]
            self.H_u = [[1]]
            
            self.Phi_v = [[1.0, 0.0], [0.0, 1.0]]
            self.Gamma_v = [[0.0], [0.0]]
            self.H_v = [[1.0, 1.0]]
            
            self.Phi_w = [[1.0, 0.0], [0.0, 1.0]]
            self.Gamma_w = [[0.0], [0.0]]
            self.H_w = [[1.0, 1.0]]
            return
        
        if Va < 1e-6:
            Va = 0.1
        
        # u axis turbulence from pg 5        
        self.Phi_u = [[math.exp(-(Va * dT) / L_u)]]
        
        self.Gamma_u = [[(L_u / Va) * (1.0 - math.exp(-(Va * dT) / L_u))]]
        
        self.H_u = [[sigma_u * math.sqrt(2.0 * Va / (math.pi * L_u))]]
        
        # --- V-axis turbulence ---
        exp_v = math.exp(-(Va * dT) / L_v)
        
        # Phi_v
        phi_v_mat = [
            [(1.0 + -(Va * dT) / L_v), -((Va / L_v)**2) * dT],
            [dT, (1.0 - -(Va * dT) / L_v)] 
        ]
        self.Phi_v = MatrixMath.scalarMultiply(exp_v, phi_v_mat)
        
        # Gamma_v 
        gamma_v_mat = [
            [dT],
            [((L_v / Va)**2) * (math.exp((Va / L_v) * dT) - 1.0) - (L_v / Va) * dT]
        ]
        self.Gamma_v = MatrixMath.scalarMultiply(exp_v, gamma_v_mat)
        
        # H_v
        h_v_coeff = sigma_v * math.sqrt(3.0 * Va / (math.pi * L_v))
        self.H_v = MatrixMath.scalarMultiply(sigma_v * math.sqrt(3.0 * Va / (math.pi * L_v)), [[1, Va / (math.sqrt(3.0) * L_v)]])
        
        # w axis turbulence        
        # Phi_w
        phi_w_mat = [
            [(1.0 + -(Va * dT) / L_w), -((Va / L_w)**2) * dT],
            [dT, (1.0 - -(Va * dT) / L_w)] 
        ]
        self.Phi_w = MatrixMath.scalarMultiply(math.exp(-(Va * dT) / L_w), phi_w_mat)
        
        # Gamma_w 
        gamma_w_mat = [
            [dT],
            [((L_w / Va)**2) * (math.exp((Va / L_w) * dT) - 1.0) - (L_w / Va) * dT]
        ]
        self.Gamma_w = MatrixMath.scalarMultiply(math.exp(-(Va * dT) / L_w), gamma_w_mat)
        
        # H_w
        h_w_coeff = sigma_w * math.sqrt(3.0 * Va / (math.pi * L_w))
        self.H_w = [[h_w_coeff, h_w_coeff * Va / (math.sqrt(3.0) * L_w)]]
        
        return
    

    def Update(self, uu=None, uv=None, uw=None):
        if uu == None:
            uu = random.gauss(0 , 1)
        if uv == None:
            uv = random.gauss(0 , 1)
        if uw == None:
            uw = random.gauss(0 , 1)
        
        # step 2, update state using random input
        # u-axis
        phi_x_u = MatrixMath.multiply(self.Phi_u, self.x_u_vector)
        gamma_u_term = MatrixMath.scalarMultiply(uu, self.Gamma_u)
        x_u_plus = MatrixMath.add(phi_x_u, gamma_u_term)
        
        # v-axis
        phi_x_v = MatrixMath.multiply(self.Phi_v, self.x_v_vector)
        gamma_v_term = MatrixMath.scalarMultiply(uv, self.Gamma_v)
        x_v_plus = MatrixMath.add(phi_x_v, gamma_v_term)
        
        # w-axis
        phi_x_w = MatrixMath.multiply(self.Phi_w, self.x_w_vector)
        gamma_w_term = MatrixMath.scalarMultiply(uw, self.Gamma_w)
        x_w_plus = MatrixMath.add(phi_x_w, gamma_w_term)
        
        # generate gusts from state
        self.Wind.Wu = MatrixMath.multiply(self.H_u, x_u_plus)[0][0]
        self.Wind.Wv = MatrixMath.multiply(self.H_v, x_v_plus)[0][0]
        self.Wind.Ww = MatrixMath.multiply(self.H_w, x_w_plus)[0][0]
        
        # update previous state
        self.x_u_vector = x_u_plus
        self.x_v_vector = x_v_plus
        self.x_w_vector = x_w_plus

        return self.Wind