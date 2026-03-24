import math
from ..Containers import Controls
from ..Containers import Sensors
from ..Containers import States
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleDynamicsModel as VDM
from ..Sensors import SensorsModel
from ..Utilities import MatrixMath as MM
from ..Utilities import Rotations

class LowPassFilter:
    def __init__(self, dT=VPC.dT, cutoff=1.0):
        self.dT = dT
        self.cutoff = cutoff
        
        a = 2.0 * math.pi * cutoff
        self.alpha = math.exp(-a * dT)
        
        self.output = 0.0
    
    def reset(self):
        self.output = 0.0
    
    def update(self, input):
        self.output = self.alpha * self.output + (1.0 - self.alpha) * input
        return self.output


class VehicleEstimator:
    def __init__(self, dT=VPC.dT, gains=None, sensorsModel=None):
        self.dT = dT
        self.sensorsModel = sensorsModel
        
        # init estimated state
        self.estimatedState = States.vehicleState()
        self.estimatedState.pd = VPC.InitialDownPosition
        self.estimatedState.Va = VPC.InitialSpeed
        
        self.estimatedState.R = [[1.0, 0.0, 0.0],
                                [0.0, 1.0, 0.0],
                                [0.0, 0.0, 1.0]]
        
        if gains is None:
            self.gains = type('obj', (object,), {
                'Kp_acc': 0.0,
                'Ki_acc': 0.0,
                'Kp_mag': 0.0,
                'Ki_mag': 0.0,
                'Kp_Va': 0.0,
                'Ki_Va': 0.0,
                'Kp_h': 0.0,
                'Ki_h': 0.0,
                'Kp_h_gps': 0.0,
                'Ki_h_gps': 0.0,
                'Kp_chi': 0.0,
                'Ki_chi': 0.0,
                'lowPassCutoff_h': 0.1
            })()
        else:
            self.gains = gains
        
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        self.airspeed_bias = 0.0
        self.altitude_gps_bias = 0.0
        self.course_bias = 0.0

        # Low-pass filter for baro
        if hasattr(self.gains, 'lowPassCutoff_h'):
            cutoff_freq = self.gains.lowPassCutoff_h if self.gains.lowPassCutoff_h > 0 else 0.1
        else:
            cutoff_freq = 0.1
        
        self.baroLPF = LowPassFilter(dT=dT, cutoff=cutoff_freq)
        
        self.hdot = 0.0
    
    def reset(self):
        # reset estimated state
        self.estimatedState = States.vehicleState()
        self.estimatedState.pd = VPC.InitialDownPosition
        self.estimatedState.Va = VPC.InitialSpeed
        
        # reset DCM to identity
        self.estimatedState.R = [[1.0, 0.0, 0.0],
                                   [0.0, 1.0, 0.0],
                                   [0.0, 0.0, 1.0]]
        
        # reset all biases to zero
        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0
        self.gyro_bias_z = 0.0
        self.airspeed_bias = 0.0
        self.altitude_gps_bias = 0.0
        self.course_bias = 0.0
        
        # reset low-pass filter
        self.baroLPF.reset()
        
        # reset climb rate
        self.hdot = 0.0
    
    
    def getEstimatedState(self):
        return self.estimatedState
    
    def setEstimatedState(self, state):
        self.estimatedState = state
    
    def getEstimatorGains(self):
        return self.gains
    
    def setEstimatorGains(self, gains):
        self.gains = gains

    def setEstimatorBiases(self, estimatedGyroBias=None, estimatedPitotBias=0, 
                          estimatedChiBias=0, estimatedAscentRate=0, estimatedAltitudeGPSBias=0):
        if estimatedGyroBias is None:
            estimatedGyroBias = [[0], [0], [0]]
        
        self.gyro_bias_x = estimatedGyroBias[0][0]
        self.gyro_bias_y = estimatedGyroBias[1][0]
        self.gyro_bias_z = estimatedGyroBias[2][0]
        
        self.airspeed_bias = estimatedPitotBias
        self.course_bias = estimatedChiBias
        self.hdot = estimatedAscentRate
        self.altitude_gps_bias = estimatedAltitudeGPSBias
    
    
    def estimateAttitude(self, sensorData=Sensors.vehicleSensors(), estimatedState=States.vehicleState()):
        # Extract gyro measurements
        p_meas = sensorData.gyro_x
        q_meas = sensorData.gyro_y
        r_meas = sensorData.gyro_z
        
        # Extract accelerometer and magnetometer
        accel_body = [[sensorData.accel_x],
                     [sensorData.accel_y],
                     [sensorData.accel_z]]
        
        mag_body = [[sensorData.mag_x],
                   [sensorData.mag_y],
                   [sensorData.mag_z]]
        
        i_grav = [[0.0], [0.0], [VPC.g0]]
        i_grav_norm = MM.scalarMultiply(1.0 / VPC.g0, i_grav)
        
        # Magnetic field from constants
        i_mag = VSC.magfield
        mag_mag = math.sqrt(i_mag[0][0]**2 + i_mag[1][0]**2 + i_mag[2][0]**2)
        i_mag_norm = MM.scalarMultiply(1.0 / mag_mag, i_mag)
        
        omega_eps_mag = [[0.0], [0.0], [0.0]]
        omega_eps_acc = [[0.0], [0.0], [0.0]]
        bias_dot = [[0.0], [0.0], [0.0]]
        
        mag_meas_mag = math.sqrt(mag_body[0][0]**2 + mag_body[1][0]**2 + mag_body[2][0]**2)
        
        if mag_meas_mag > 1e-6:
            b_mag_norm = MM.scalarMultiply(1.0 / mag_meas_mag, mag_body)
            R_i_mag = MM.multiply(estimatedState.R, i_mag_norm)
            
            omega_eps_mag = self.crossProduct(b_mag_norm, R_i_mag)
            
            # Bias rate from magnetometer
            bias_dot = MM.add(bias_dot, MM.scalarMultiply(-self.gains.Ki_mag, omega_eps_mag))
        
        accel_mag = math.sqrt(accel_body[0][0]**2 + accel_body[1][0]**2 + accel_body[2][0]**2)
        
        if 0.9 * VPC.g0 <= accel_mag <= 1.1 * VPC.g0:
            b_accel_norm = MM.scalarMultiply(1.0 / accel_mag, accel_body)
            R_i_grav = MM.multiply(estimatedState.R, i_grav_norm)
            
            omega_eps_acc = self.crossProduct(b_accel_norm, R_i_grav)
            
            # Bias rate from accelerometer
            bias_dot = MM.add(bias_dot, MM.scalarMultiply(-self.gains.Ki_acc, omega_eps_acc))
        
        bias_vec = [[self.gyro_bias_x], [self.gyro_bias_y], [self.gyro_bias_z]]
        bias_vec = MM.add(bias_vec, MM.scalarMultiply(self.dT, bias_dot))
        
        self.gyro_bias_x = bias_vec[0][0]
        self.gyro_bias_y = bias_vec[1][0]
        self.gyro_bias_z = bias_vec[2][0]
        
        correction = MM.add(
            MM.scalarMultiply(self.gains.Kp_mag, omega_eps_mag),
            MM.scalarMultiply(self.gains.Kp_acc, omega_eps_acc)
        )
        
        p_corr = p_meas - self.gyro_bias_x + correction[0][0]
        q_corr = q_meas - self.gyro_bias_y + correction[1][0]
        r_corr = r_meas - self.gyro_bias_z + correction[2][0]
        
        dummyState = States.vehicleState()
        dummyState.p = p_corr
        dummyState.q = q_corr
        dummyState.r = r_corr
        dummyState.R = estimatedState.R
        
        dummyDot = States.vehicleState()  # all zeros
        
        vdm = VDM.VehicleDynamicsModel(dT=self.dT)
        Rexp_matrix = vdm.Rexp(self.dT, dummyState, dummyDot)
        
        estimatedState.R = MM.multiply(Rexp_matrix, estimatedState.R)
        
        # Update body rates and Euler angles
        estimatedState.p = p_corr
        estimatedState.q = q_corr
        estimatedState.r = r_corr
        
        estimatedState.yaw, estimatedState.pitch, estimatedState.roll = Rotations.dcm2Euler(estimatedState.R)
        
        # Return values as per documentation
        omega_corr_vec = [[p_corr], [q_corr], [r_corr]]
        return bias_vec, omega_corr_vec, estimatedState.R
    
    def crossProduct(self, a, b):
        result = [[a[1][0] * b[2][0] - a[2][0] * b[1][0]],
                  [a[2][0] * b[0][0] - a[0][0] * b[2][0]],
                  [a[0][0] * b[1][0] - a[1][0] * b[0][0]]]
        return result
    
    
    def estimateAirspeed(self, sensorData=Sensors.vehicleSensors(), estimatedState=States.vehicleState()):
        # Get pitot measurement and convert to airspeed
        pitot_pressure = sensorData.pitot
        if pitot_pressure < 0:
            pitot_pressure = 0.0
        
        Va_pitot = math.sqrt(2.0 * pitot_pressure / VPC.rho)
        
        gravity_inertial = [[0.0], [0.0], [VPC.g0]]
        gravity_body = MM.multiply(estimatedState.R, gravity_inertial)
        
        accel_body = [[sensorData.accel_x],
                     [sensorData.accel_y],
                     [sensorData.accel_z]]
        
        accel_inertial_body = MM.add(accel_body, gravity_body)
        ax = accel_inertial_body[0][0]
        
        # Complementary filter
        error = Va_pitot - estimatedState.Va
        
        # Update bias
        bias_dot = -self.gains.Ki_Va * error
        self.airspeed_bias += bias_dot * self.dT
        
        # Update airspeed
        Va_dot = ax - self.airspeed_bias + self.gains.Kp_Va * error
        estimatedState.Va += Va_dot * self.dT
        
        # Clamp to positive
        if estimatedState.Va < 0.1:
            estimatedState.Va = 0.1
        
        return self.airspeed_bias, estimatedState.Va
    
    
    def estimateAltitude(self, sensorData=Sensors.vehicleSensors(), estimatedState=States.vehicleState()):
        # Convert baro pressure to altitude
        baro_pressure = sensorData.baro
        h_baro = (VSC.Pground - baro_pressure) / (VPC.rho * VPC.g0)
        
        # Low-pass filter baro
        h_baro_filt = self.baroLPF.update(h_baro)
        
        accel_body = [[sensorData.accel_x],
                    [sensorData.accel_y],
                    [sensorData.accel_z]]
        
        R_T = MM.transpose(estimatedState.R)
        accel_inertial = MM.multiply(R_T, accel_body)
        i_aup = accel_inertial[2][0] + VPC.g0
        
        h_est = -estimatedState.pd
        
        error_baro = h_baro_filt - h_est
        
        self.hdot += (i_aup + self.gains.Ki_h * error_baro) * self.dT
        
        h_est += (self.gains.Kp_h * error_baro + self.hdot) * self.dT
        
        gps_updated = False
        if self.sensorsModel is not None:
            gps_updated = (self.sensorsModel.updateTicks % self.sensorsModel.gpsTicksUpdate == 1)
        
        if gps_updated:
            gps_alt = sensorData.gps_alt
            gps_error = gps_alt - h_est
            
            # update GPS bias
            bias_dot_gps = -self.gains.Ki_h_gps * gps_error
            self.altitude_gps_bias += bias_dot_gps * self.dT
            
            h_dot_GPS = h_est - self.altitude_gps_bias + self.gains.Kp_h_gps * gps_error
            h_est += h_dot_GPS * self.dT
        else:
            # Remove bias between GPS updates
            h_est -= self.altitude_gps_bias
        
        # update state
        estimatedState.pd = -h_est
        
        return h_est, self.hdot, self.altitude_gps_bias


    def estimateCourse(self, sensorData=Sensors.vehicleSensors(), estimatedState=States.vehicleState()):
        # Calculate yaw rate from body rates
        phi = estimatedState.roll
        theta = estimatedState.pitch
        q = estimatedState.q
        r = estimatedState.r
        
        cos_theta = math.cos(theta)
        if abs(cos_theta) < 0.01:
            cos_theta = 0.01 if cos_theta >= 0 else -0.01
        
        chi_dot = (q * math.sin(phi) + r * math.cos(phi)) / cos_theta
        
        # Check for GPS update
        gps_updated = False
        if self.sensorsModel is not None:
            gps_updated = (self.sensorsModel.updateTicks % self.sensorsModel.gpsTicksUpdate == 1)
        
        if gps_updated:
            gps_cog = sensorData.gps_cog
            
            # Calculate error
            chi_error = gps_cog - estimatedState.chi
            chi_error = self.wrapAngle(chi_error)
            
            # Update bias
            bias_dot = -self.gains.Ki_chi * chi_error
            self.course_bias += bias_dot * self.dT
            
            # Update course 
            estimatedState.chi += (self.gains.Kp_chi * chi_error - self.course_bias) * self.dT
        else:
            estimatedState.chi += (chi_dot - self.course_bias) * self.dT
        
        estimatedState.chi = self.wrapAngle(estimatedState.chi)
        
        return self.course_bias, estimatedState.chi
    
    def wrapAngle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle
    
    
    def Update(self):
        sensorData = self.sensorsModel.getSensorsNoisy()
        
        # Update in order: Attitude FIRST, then others
        self.estimateAttitude(sensorData, self.estimatedState)
        self.estimateAltitude(sensorData, self.estimatedState)
        self.estimateAirspeed(sensorData, self.estimatedState)
        self.estimateCourse(sensorData, self.estimatedState)
        
        # Set alpha and beta (not estimated in this lab)
        self.estimatedState.alpha = 0.0
        self.estimatedState.beta = 0.0