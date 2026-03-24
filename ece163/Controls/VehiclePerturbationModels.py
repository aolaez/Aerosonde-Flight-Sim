import math
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Constants import VehiclePhysicalConstants as VPC
from ece163.Containers import States
from ece163.Containers import Inputs
from ece163.Containers import Linearized
from ece163.Utilities import MatrixMath
from ece163.Controls import VehicleTrim

def CreateTransferFunction(trimState, trimInputs):
    rho = VPC.rho
    m = VPC.mass
    S = VPC.S
    b = VPC.b
    c = VPC.c
    Va = trimState.Va

    trans_func = Linearized.transferFunctions()
    trans_func.Va_trim = math.sqrt(trimState.u**2 + trimState.v**2 + trimState.w**2)
    trans_func.alpha_trim = trimState.alpha
    trans_func.theta_trim = trimState.pitch

    if math.isclose(Va, 0.0):
        trans_func.beta_trim = 0
    else:
        trans_func.beta_trim = trimState.beta
    
    trans_func.gamma_trim = trans_func.theta_trim - trans_func.alpha_trim
    trans_func.phi_trim = trimState.roll

    trans_func.a_V1 = (((rho*Va* S)/m)*(VPC.CD0 + (VPC.CDalpha*trimState.alpha) + (VPC.CDdeltaE*trimInputs.Elevator))) - ((1.0/m) * dThrust_dVa(Va, trimInputs.Throttle))
    trans_func.a_V2 = (1.0/m) * dThrust_dThrottle(Va, trimInputs.Throttle)
    trans_func.a_V3 = VPC.g0 * math.cos(trimState.pitch - trimState.alpha)

    trans_func.a_phi1 = -0.5 * rho * (Va ** 2.0) * S * b * VPC.Cpp * (b/(2.0*Va))
    trans_func.a_phi2 = 0.5 * rho * (Va ** 2.0) * S * b * VPC.CpdeltaA

    trans_func.a_beta1 = ((-rho * Va * S)/(2.0*m)) * VPC.CYbeta
    trans_func.a_beta2 = ((rho * Va * S)/(2.0*m)) * VPC.CYdeltaR

    trans_func.a_theta1 = -((rho * (trimState.Va **2.0) * c * S)/(2.0* VPC.Jyy)) * VPC.CMq * (c/(2.0*Va))
    trans_func.a_theta2 = -((rho * (trimState.Va **2.0) * c * S)/(2.0* VPC.Jyy)) * VPC.CMalpha
    trans_func.a_theta3 = ((rho * (trimState.Va **2.0) * c * S)/(2.0* VPC.Jyy)) * VPC.CMdeltaE

    return trans_func

def dThrust_dThrottle(Va, Throttle, epsilon=0.01):
    # partial derivative of thrust wrt throttle 
    aeroModel = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    
    # thrust at curr throttle
    fx = aeroModel.CalculatePropForces(Va, Throttle)
    
    # w epsilon
    fxplus = aeroModel.CalculatePropForces(Va, Throttle + epsilon)
    
    dt_dDeltaT = (fxplus[0] - fx[0])/epsilon # derivative
    
    return dt_dDeltaT


def dThrust_dVa(Va, Throttle, epsilon=0.5):
    # partial derivative of thrust wrt airspeed
    aeroModel = VehicleAerodynamicsModel.VehicleAerodynamicsModel()
    
    # thrust at curr Va
    fx = aeroModel.CalculatePropForces(Va, Throttle)
    
    # thrust + epsilon
    fxplus = aeroModel.CalculatePropForces(Va + epsilon, Throttle)
    
    # derivative
    dt_dVa = (fxplus[0] - fx[0]) / epsilon
    
    return dt_dVa
