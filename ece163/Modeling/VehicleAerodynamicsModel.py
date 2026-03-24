"""
Docstring for ece163.Modeling.VehicleAerodynamicsModel
Author: Aydan Olaez (aolaez@ucsc.edu)
File containing functions for wrapping dynamics model,
calculating gravity forces based on body angles
"""
import math
from ..Containers import States
from ..Containers import Inputs
from ..Modeling import VehicleDynamicsModel
from ..Modeling import WindModel
from ..Utilities import MatrixMath
from ..Utilities import Rotations
from ..Constants import VehiclePhysicalConstants as VPC

class VehicleAerodynamicsModel:
    # 3.1 - init and rset
    def __init__(self, initialSpeed=VPC.InitialSpeed, initialHeight=VPC.InitialDownPosition, dT=VPC.dT):
        # save I.C's and make dynamics model
        self.initialSpeed = initialSpeed
        self.initialHeight = initialHeight
        self.dT = dT
        self.vehicleDynamics = VehicleDynamicsModel.VehicleDynamicsModel(dT=dT)
        
        # save initial state
        init_State = States.vehicleState()
        
        # save initial pos
        init_State.pn = VPC.InitialNorthPosition
        init_State.pe = VPC.InitialEastPosition
        init_State.pd = initialHeight
        
        # set init velocity
        init_State.u = initialSpeed # x velo
        init_State.v = 0.0 # y velocity
        init_State.w = 0.0 # z velocity
        
        # initial eulers and derive R_B->I from them
        init_State.yaw = VPC.InitialYawAngle # heading
        init_State.pitch = 0.0 # nose up/down
        init_State.roll = 0.0 # wings
        init_State.R = Rotations.euler2DCM(init_State.yaw, init_State.pitch, init_State.roll)
        
        # set airspeed w no wind
        init_State.Va = initialSpeed  # airspeed magnitude
        init_State.alpha = 0.0 # angle of attack, no vert velocity
        init_State.beta = 0.0  # sideslip angle, no lateral velocity
        self.vehicleDynamics.setVehicleState(init_State)

        self.windModel = WindModel.WindModel(dT=dT)
        return


    def reset(self):
        # reset inner dynamics model
        self.vehicleDynamics.reset()
        self.windModel.reset()
        
        # set initial state, speed, coords, R, Va, attack and sideslip angles
        initialState = States.vehicleState()
        initialState.pn = VPC.InitialNorthPosition
        initialState.pe = VPC.InitialEastPosition
        initialState.pd = self.initialHeight  
        initialState.u = self.initialSpeed 
        initialState.v = 0.0
        initialState.w = 0.0
        initialState.yaw = VPC.InitialYawAngle
        initialState.pitch = 0.0
        initialState.roll = 0.0
        initialState.R = Rotations.euler2DCM(initialState.yaw, initialState.pitch, initialState.roll)
        initialState.Va = self.initialSpeed
        initialState.alpha = 0.0
        initialState.beta = 0.0
        
        self.vehicleDynamics.setVehicleState(initialState)
        
        return

    # 3.2 - getteres and setters
    def getVehicleState(self):
        # j get dynamics object
        return self.vehicleDynamics.getVehicleState()
    
    def setVehicleState(self, state):
        # set internal state
        self.vehicleDynamics.setVehicleState(state)
        return
    
    def getVehicleDerivative(self):
        # get inner derivative
        return self.vehicleDynamics.getVehicleDerivative()
    
    def getVehicleDynamicsModel(self):
        # return full object
        return self.vehicleDynamics
    
    ###################
    # CHANGE NEXT LAB #
    ###################
    def setWindModel(self, windModel):
        self.windModel = windModel
        return
    ###################
    # CHANGE NEXT LAB #
    ###################
    def getWindModel(self):
        return self.windModel
    
    # 3.3 - update()
    def Update(self, controls):
        # get curr state, calculate total forces and pass them to update

        state = self.vehicleDynamics.getVehicleState()
        wind = self.windModel.Update()
        forces = self.updateForces(state, controls, wind)
        self.vehicleDynamics.Update(forces)
        
        return
    
    # 4.1 - gravityForces()
    def gravityForces(self, state):
        # NED/inertial frame gravity vector
        gravity_inertial = [[0.0],
                        [0.0],
                        [VPC.mass * VPC.g0]]
        
        # rotate into body frame
        gravity_body = MatrixMath.multiply(state.R, gravity_inertial)
        
        # create force object
        forces = Inputs.forcesMoments()
        forces.Fx = gravity_body[0][0]
        forces.Fy = gravity_body[1][0]
        forces.Fz = gravity_body[2][0]
        
        # set moments to 0 bc gravity creates none
        forces.Mx = 0.0
        forces.My = 0.0
        forces.Mz = 0.0
        
        return forces
    
    # 4.2 - calculateCoeffAlpha()
    def CalculateCoeff_alpha(self, alpha):
        # sigma calc from book
        exponential = math.exp(-VPC.M * (alpha - VPC.alpha0))
        neg_exponential = math.exp(VPC.M * (alpha + VPC.alpha0))
        sigma = (1.0 + neg_exponential + exponential) / ((1.0 + neg_exponential) * (1.0 + exponential))

        # all equations from lab doc 
        C_L_attached = VPC.CL0 + (VPC.CLalpha * alpha)
        C_D_attached = VPC.CDp + ((C_L_attached ** 2)/(math.pi * VPC.AR * VPC.e))
        C_L_separated = 2.0*math.sin(alpha)*math.cos(alpha)
        C_D_separated = 2.0*(math.sin(alpha) ** 2)
        C_L = ((1.0 - sigma) * C_L_attached) + (sigma * C_L_separated)
        C_D = ((1.0 - sigma) * C_D_attached) + (sigma * C_D_separated)
        C_m = VPC.CM0 + (VPC.CMalpha * alpha)

        return (C_L, C_D, C_m)
    
    # 4.3 - aeroForces()
    def aeroForces(self, state):
        Va = state.Va
        alpha = state.alpha
        nonnormalized_q = state.q
        beta = state.beta
        p = state.p
        r = state.r
        S = VPC.S
        b = VPC.b
        rho = VPC.rho

        if abs(Va) < 1e-6:
            return Inputs.forcesMoments()

        C_L, C_D, C_m = self.CalculateCoeff_alpha(alpha)

        # from beard ch 4
        F_lift = (0.5*rho * (Va ** 2) * S) * (C_L + ((VPC.CLq*VPC.c * nonnormalized_q)/(2*Va))) # 4.6
        F_drag = (0.5*rho * (Va ** 2) * S) * (C_D + ((VPC.CDq*VPC.c * nonnormalized_q)/(2*Va))) # 4.7
        m_partial = (0.5*rho * (Va ** 2) * S * VPC.c) * (C_m + ((VPC.CMq * VPC.c * nonnormalized_q)/(2*Va))) # 4.5
        # 4.14 - lateral force
        f_y = (0.5*rho * (Va ** 2) * S) * (VPC.CY0 + (VPC.CYbeta*beta) + (VPC.CYp*b*p)/(2*Va) + (VPC.CYr*b*r)/(2*Va))
        # 4.15 - l
        roll_moment = (0.5*rho * (Va ** 2) * S * b) * (VPC.Cl0 + (VPC.Clbeta*beta) + (VPC.Clp*b*p)/(2*Va) + (VPC.Clr*b*r)/(2*Va))
        # 4.16 - n
        yaw_moment = (0.5*rho * (Va ** 2) * S * b) * (VPC.Cn0 + (VPC.Cnbeta*beta) + (VPC.Cnp*b*p)/(2*Va) + (VPC.Cnr*b*r)/(2*Va))

        # convert lift/drag to body frame (Lab 1 simplification)
        F_x = -F_drag * math.cos(alpha) + F_lift * math.sin(alpha)
        F_y = f_y
        F_z = -F_drag * math.sin(alpha) - F_lift * math.cos(alpha)

        # package into forcesMoments object
        forces = Inputs.forcesMoments()
        forces.Fx = F_x
        forces.Fy = F_y
        forces.Fz = F_z
        forces.Mx = roll_moment
        forces.My = m_partial
        forces.Mz = yaw_moment

        return forces
    
    # calc prop thrust and torque
    # throttle is a duty cycle and therefore either 1 or 0
    def CalculatePropForces(self, Va, Throttle): # output thrust force and torque moment both aboud x-body        
        Vin = Throttle * VPC.V_max
        # constants
        rho = VPC.rho
        D = VPC.D_prop
        
        # motor constants
        KV = VPC.KV 
        KT = 60.0 / (2.0 * math.pi * KV)
        KE = KT 
        R = VPC.R_motor
        i0 = VPC.i0
        
        # propeller coeffs
        C_Q0 = VPC.C_Q0
        C_Q1 = VPC.C_Q1
        C_Q2 = VPC.C_Q2
        C_T0 = VPC.C_T0
        C_T1 = VPC.C_T1
        C_T2 = VPC.C_T2
        
        a = (rho * (D**5) * C_Q0) / (4.0 * (math.pi**2))
        b = (rho * (D**4) * Va * C_Q1) / (2.0 * math.pi) + ((KT * KE) / R)
        c = rho * (D**3) * (Va**2) * C_Q2 - ((KT * Vin)/R) + (KT * i0)
        
        discriminant = b**2 - 4.0 * a * c
        try: # computetrim error
            omega = (-b + math.sqrt(discriminant)) / (2.0 * a)
        except:
            omega = 100.0 
        if abs(omega) < 1e-6:
            return (0.0, 0.0)

        # advance ratio
        J = (2.0 * math.pi * Va)/(omega * D)
        
        # thrust and torque coeffs
        C_T = C_T0 + (C_T1*J) + (C_T2 * J**2)
        C_Q = C_Q0 + (C_Q1*J) + (C_Q2 * J**2)
        
        # EQ from cheat sheet
        F_prop = (rho * omega**2 * D**4 * C_T)/(4.0 * (math.pi**2))
        
        # EQ from cheat sheet, negative bc propeller spinning CW
        M_prop = -(rho * omega**2 * D**5 * C_Q)/(4.0 * (math.pi**2))
        
        return (F_prop, M_prop)

    # aero forces from control surfaces and prop
    # takes vehicle state and throttle, aileron, elevator, rudder
    def controlForces(self, state, controls):
        # control inputs and deflection vals in rads
        throttle = controls.Throttle
        delta_a = controls.Aileron 
        delta_e = controls.Elevator
        delta_r = controls.Rudder
        
        # state variables
        Va = state.Va
        alpha = state.alpha
        S = VPC.S
        
        # zero airspeed case, do i need this?
        
        mult_factor = 0.5 * VPC.rho * (Va**2) * S

        # missing terms from 4.5-4.7
        f_lift_elevator = mult_factor *  VPC.CLdeltaE * delta_e
        f_drag_elevator = mult_factor * VPC.CDdeltaE * delta_e
        # elevator pitching moment
        m_elevator = mult_factor * VPC.c * VPC.CMdeltaE * delta_e
        
        # convert frames
        fx_elevator = -f_drag_elevator * math.cos(alpha) + f_lift_elevator * math.sin(alpha)
        fz_elevator = -f_drag_elevator * math.sin(alpha) - f_lift_elevator * math.cos(alpha)
        
        # aileron and rudder, lateral conrtol surfqce forces
        # 4.14-4.16
        # aileron and rudder side force
        Fy_control = mult_factor * ((VPC.CYdeltaA * delta_a) + (VPC.CYdeltaR * delta_r))
        # roll moment
        m_x_control = mult_factor * VPC.b * ((VPC.CldeltaA * delta_a) + (VPC.CldeltaR * delta_r))
        
        # yawing moment
        m_z_control = mult_factor * VPC.b * ((VPC.CndeltaA * delta_a) + (VPC.CndeltaR * delta_r))
        
        # prop forces both about x body axis
        Fprop, Mprop = self.CalculatePropForces(Va, throttle)

        forces = Inputs.forcesMoments()
        forces.Fx = fx_elevator + Fprop
        forces.Fy = Fy_control
        forces.Fz = fz_elevator
        forces.Mx = m_x_control + Mprop
        forces.My = m_elevator
        forces.Mz = m_z_control
        
        return forces

    # sum gravity aero and control forces, update Va alpha and beta
    def updateForces(self, state, controls, wind=None):
        if wind != None:
            airspeed = self.CalculateAirspeed(state, wind)
            state.Va = airspeed[0]
            state.alpha = airspeed[1]
            state.beta = airspeed[2]
        else:
            state.Va = math.sqrt(state.u**2 + state.v**2 + state.w**2)
            state.alpha = math.atan2(state.w, state.u)
            if(math.isclose(state.Va, 0.0)):
                state.beta = 0.0
            else:
                state.beta = math.asin(state.v/math.hypot(state.u, state.v, state.w))
        
        # calc force components
        gravity = self.gravityForces(state)
        aero = self.aeroForces(state)
        control = self.controlForces(state, controls)
        
        total_forces = gravity + aero + control
        return total_forces
    

    # calculate Va, alpha and beta
    def CalculateAirspeed(self, state, wind):
        # body frame velocities
        u = state.u
        v = state.v
        w = state.w

        # NED frame steady wind
        Wn = wind.Wn
        We = wind.We
        Wd = wind.Wd

        # steady wind vector, must be rotated from intertial into body frame
        Wind_s_magnitude = math.sqrt((Wn ** 2) + (We ** 2) + (Wd ** 2))
        # azimuth and elevation
        chi_omega = math.atan2(We, Wn)
        if math.isclose(Wind_s_magnitude, 0.0): # for divide by zero error
            gamma_omega = 0.0
        else:
            gamma_omega = -math.asin(Wd / Wind_s_magnitude)

        # azimuth elevation matrix
        R_azimuth_elevation = [
            [math.cos(chi_omega)*math.cos(gamma_omega), math.sin(chi_omega)*math.cos(gamma_omega), -math.sin(gamma_omega)],
            [-math.sin(chi_omega), math.cos(chi_omega), 0], 
            [math.cos(chi_omega)*math.sin(gamma_omega), math.sin(chi_omega)*math.sin(gamma_omega), math.cos(gamma_omega)]
        ]

        Wind_g = [[wind.Wu], [wind.Wv], [wind.Ww]]
        Wind_g_inertial = MatrixMath.multiply(MatrixMath.transpose(R_azimuth_elevation), Wind_g)

        Wind_s_inertial = [[Wn], [We], [Wd]]
        total_inertial = MatrixMath.add(Wind_s_inertial, Wind_g_inertial)
        Wind_body = MatrixMath.multiply(state.R, total_inertial)

        # velocities minus wind
        u_r = u - Wind_body[0][0]
        v_r = v - Wind_body[1][0]
        w_r = w - Wind_body[2][0]
        
        Va = math.sqrt(u_r**2 + v_r**2 + w_r**2)
        
        # angle of attack
        alpha = math.atan2(w_r, u_r)
        
        # sideslip angle
        if abs(Va) < 1e-6:
            beta = 0.0
        else:
            beta = math.asin(v_r / Va)

        state.Va = Va
        state.alpha = alpha
        state.beta = beta
        
        return Va, alpha, beta






    
