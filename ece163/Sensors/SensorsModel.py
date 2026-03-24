import math
import random
from ece163.Modeling import VehicleAerodynamicsModel
from ece163.Utilities import MatrixMath
from ..Containers import Sensors
from ..Constants import VehiclePhysicalConstants as VPC
from ..Constants import VehicleSensorConstants as VSC
from ..Modeling import VehicleAerodynamicsModel

class GaussMarkov():
    def __init__(self, dT=VPC.dT, tau=1000000, eta=0):
        self.dT = dT
        self.tau = tau
        self.eta = eta
        self.v = 0

    def reset(self):
        self.v = 0

    def update(self, vnoise = None):
        if vnoise is None:
            vnoise = random.gauss(0, self.eta) 
            
        self.v = math.exp(-self.dT / self.tau) * self.v + vnoise
        return self.v
    
class GaussMarkovXYZ():
    def __init__(self, dT = VPC.dT, tauX =1000000, etaX=0.0, tauY =None, etaY=None, tauZ=None, etaZ=None):
        self.dT = dT
        self.vX = 0
        self.vY = 0
        self.vZ = 0
        
        if tauY is None:
            tauY = tauX
        if etaY is None:
            etaY = etaX
        
        if tauZ is None:
            tauZ = tauY
        if etaZ is None:
            etaZ = etaY
        
        self.tauX = tauX
        self.etaX = etaX
        self.tauY = tauY
        self.etaY = etaY
        self.tauZ = tauZ
        self.etaZ = etaZ

    def reset(self):
        self.vX = 0
        self.vY = 0
        self.vZ = 0

    def update(self, vXnoise=None, vYnoise=None, vZnoise=None):
        if vXnoise is None:
            vXnoise = random.gauss(0, self.etaX)
        self.vX = math.exp(-self.dT/self.tauX) * self.vX + vXnoise
        
        if vYnoise is None:
            vYnoise = random.gauss(0, self.etaY)
        self.vY = math.exp(-self.dT/self.tauY) * self.vY + vYnoise
        
        if vZnoise is None:
            vZnoise = random.gauss(0, self.etaZ)
        self.vZ = math.exp(-self.dT/self.tauZ) * self.vZ + vZnoise
        
        return self.vX, self.vY, self.vZ

class SensorsModel():
    def __init__(self,aeroModel=VehicleAerodynamicsModel.VehicleAerodynamicsModel(),taugyro=400.0,etagyro=0.0012740903539558606, tauGPS = 1100.0, etaGPSHorizontal=0.21, etaGPSVertical=0.4, gpsUpdateHz=1.0):
        if aeroModel is None:
            aeroModel = VehicleAerodynamicsModel.VehicleAerodynamicsModel()

        self.aeroModel = aeroModel
        self.dynamicsModel = aeroModel.getVehicleDynamicsModel()
        self.dT = self.dynamicsModel.dT

        # sensor stuffs
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsNoisy = Sensors.vehicleSensors()
        self.sensorBiases = self.initializeBiases()
        self.sensorSigmas = self.initializeSigmas()

        self.gpsGM = GaussMarkovXYZ(dT=1/gpsUpdateHz, tauX=tauGPS, etaX=etaGPSHorizontal, tauZ=tauGPS, etaZ=etaGPSVertical)
        self.gyroGM = GaussMarkovXYZ(dT=self.dT, tauX=taugyro, etaX=etagyro)
        self.updateTicks = 0
        self.gpsTicksUpdate = int(1.0 / (gpsUpdateHz * self.dT))

    # random static biases for each sensor
    def initializeBiases(self, gyroBias=None, accelBias=None, magBias=None, 
                        baroBias=None, pitotBias=None):
        # default if needed
        if gyroBias is None:
            gyroBias = VSC.gyro_bias
        if accelBias is None:
            accelBias = VSC.accel_bias
        if magBias is None:
            magBias = VSC.mag_bias
        if baroBias is None:
            baroBias = VSC.baro_bias
        if pitotBias is None:
            pitotBias = VSC.pitot_bias
        
        sensorBiases = Sensors.vehicleSensors()
        
        # gyro biases 
        sensorBiases.gyro_x = random.uniform(-gyroBias, gyroBias)
        sensorBiases.gyro_y = random.uniform(-gyroBias, gyroBias)
        sensorBiases.gyro_z = random.uniform(-gyroBias, gyroBias)
        
        # accelerometer biases
        sensorBiases.accel_x = random.uniform(-accelBias, accelBias)
        sensorBiases.accel_y = random.uniform(-accelBias, accelBias)
        sensorBiases.accel_z = random.uniform(-accelBias, accelBias)
        
        # magnetometer biases
        sensorBiases.mag_x = random.uniform(-magBias, magBias)
        sensorBiases.mag_y = random.uniform(-magBias, magBias)
        sensorBiases.mag_z = random.uniform(-magBias, magBias)
        
        # barometer bias 
        sensorBiases.baro = random.uniform(-baroBias, baroBias)
        
        # pitot tube bias
        sensorBiases.pitot = random.uniform(-pitotBias, pitotBias)
        
        # gps no static bias
        sensorBiases.gps_n = 0.0
        sensorBiases.gps_e = 0.0
        sensorBiases.gps_alt = 0.0
        sensorBiases.gps_sog = 0.0
        sensorBiases.gps_cog = 0.0
        
        return sensorBiases

    def initializeSigmas(self, gyroSigma=None, accelSigma=None, magSigma=None,
                        baroSigma=None, pitotSigma=None, 
                        gpsSigmaHorizontal=None, gpsSigmaVertical=None,
                        gpsSigmaSOG=None, gpsSigmaCOG=None):
        if gyroSigma is None:
            gyroSigma = VSC.gyro_sigma
        if accelSigma is None:
            accelSigma = VSC.accel_sigma
        if magSigma is None:
            magSigma = VSC.mag_sigma
        if baroSigma is None:
            baroSigma = VSC.baro_sigma
        if pitotSigma is None:
            pitotSigma = VSC.pitot_sigma
        if gpsSigmaHorizontal is None:
            gpsSigmaHorizontal = VSC.GPS_sigmaHorizontal
        if gpsSigmaVertical is None:
            gpsSigmaVertical = VSC.GPS_sigmaVertical
        if gpsSigmaSOG is None:
            gpsSigmaSOG = VSC.GPS_sigmaSOG
        if gpsSigmaCOG is None:
            gpsSigmaCOG = VSC.GPS_sigmaCOG
        
        sensorSigmas = Sensors.vehicleSensors()
        
        # IMU sigmas
        sensorSigmas.gyro_x = gyroSigma
        sensorSigmas.gyro_y = gyroSigma
        sensorSigmas.gyro_z = gyroSigma
        
        sensorSigmas.accel_x = accelSigma
        sensorSigmas.accel_y = accelSigma
        sensorSigmas.accel_z = accelSigma
        
        sensorSigmas.mag_x = magSigma
        sensorSigmas.mag_y = magSigma
        sensorSigmas.mag_z = magSigma
        
        # pressure sensors
        sensorSigmas.baro = baroSigma
        sensorSigmas.pitot = pitotSigma
        
        # GPS sigmas
        sensorSigmas.gps_n = gpsSigmaHorizontal
        sensorSigmas.gps_e = gpsSigmaHorizontal
        sensorSigmas.gps_alt = gpsSigmaVertical
        sensorSigmas.gps_sog = gpsSigmaSOG
        sensorSigmas.gps_cog = gpsSigmaCOG
        
        return sensorSigmas

    # ideal gyro measurements
    def updateGyrosTrue(self, state):
        return [state.p, state.q, state.r]

    # ideal accelerometer measurements
    def updateAccelsTrue(self, state, dot):
        # body frame v derivates
        vDot = [[dot.u], [dot.v], [dot.w]]
        
        # angular velocity vector
        omega = [[state.p], [state.q], [state.r]]
        v = [[state.u], [state.v], [state.w]]
        
        # accel_centripetal = omega x v
        omegaSkew = [[0, -state.r, state.q],
                     [state.r, 0, -state.p],
                     [-state.q, state.p, 0]]
        omegaCrossV = MatrixMath.multiply(omegaSkew, v)
        
        # gravity in body frame
        gravityInertial = [[0], [0], [VPC.g0]]
        gravityBody = MatrixMath.multiply(state.R, gravityInertial)
        
        accel = MatrixMath.add(vDot, omegaCrossV)
        accel = MatrixMath.subtract(accel, gravityBody)
        
        return [accel[0][0], accel[1][0], accel[2][0]]

    # ideal megnetometer measurements
    def updateMagsTrue(self, state):
        # earth magnetic field in inertial frame
        magInertial = VSC.magfield
        
        # rotate to body frame
        magBody = MatrixMath.multiply(state.R, magInertial)
        
        return [magBody[0][0], magBody[1][0], magBody[2][0]]

    # ideal barometer readings
    def updatePressureSensorsTrue(self, state):
        # barometer measures absolute pressure
        baro = VSC.Pground + VPC.rho * VPC.g0 * state.pd
        
        pitot = 0.5 * VPC.rho * (state.Va ** 2)
        
        return [baro, pitot]

    # ideal gps readings
    def updateGPSTrue(self, state, dot):
        gps_n = state.pn
        gps_e = state.pe
        gps_alt = -state.pd 
        
        # speed over ground
        gps_sog = math.hypot(dot.pn, dot.pe)
        
        # course over ground
        gps_cog = math.atan2(dot.pe, dot.pn)
        
        return [gps_n, gps_e, gps_alt, gps_sog, gps_cog]

    def updateSensorsTrue(self, prevTrueSensors, state, dot):
        trueSensors = Sensors.vehicleSensors()
        
        gyro = self.updateGyrosTrue(state)
        trueSensors.gyro_x = gyro[0]
        trueSensors.gyro_y = gyro[1]
        trueSensors.gyro_z = gyro[2]
        
        accel = self.updateAccelsTrue(state, dot)
        trueSensors.accel_x = accel[0]
        trueSensors.accel_y = accel[1]
        trueSensors.accel_z = accel[2]
        
        mag = self.updateMagsTrue(state)
        trueSensors.mag_x = mag[0]
        trueSensors.mag_y = mag[1]
        trueSensors.mag_z = mag[2]
        
        pressure = self.updatePressureSensorsTrue(state)
        trueSensors.baro = pressure[0]
        trueSensors.pitot = pressure[1]
        
        if self.updateTicks % self.gpsTicksUpdate == 0:
            gps = self.updateGPSTrue(state, dot)
            trueSensors.gps_n = gps[0]
            trueSensors.gps_e = gps[1]
            trueSensors.gps_alt = gps[2]
            trueSensors.gps_sog = gps[3]
            trueSensors.gps_cog = gps[4]
        else:
            trueSensors.gps_n = prevTrueSensors.gps_n
            trueSensors.gps_e = prevTrueSensors.gps_e
            trueSensors.gps_alt = prevTrueSensors.gps_alt
            trueSensors.gps_sog = prevTrueSensors.gps_sog
            trueSensors.gps_cog = prevTrueSensors.gps_cog
        
        return trueSensors

    def updateSensorsNoisy(self, trueSensors=None, sensorBiases=None, sensorSigmas=None, noisySensors=None):
        if trueSensors is None:
            trueSensors = self.sensorsTrue
        if sensorBiases is None:
            sensorBiases = self.sensorBiases
        if sensorSigmas is None:
            sensorSigmas = self.sensorSigmas
        if noisySensors is None:
            noisySensors = Sensors.vehicleSensors()
                
        gyroDrift = self.gyroGM.update()
        
        noisySensors.gyro_x = (trueSensors.gyro_x + 
                               sensorBiases.gyro_x + 
                               gyroDrift[0] +
                               random.gauss(0, sensorSigmas.gyro_x))
        noisySensors.gyro_y = (trueSensors.gyro_y + 
                               sensorBiases.gyro_y + 
                               gyroDrift[1] +
                               random.gauss(0, sensorSigmas.gyro_y))
        noisySensors.gyro_z = (trueSensors.gyro_z + 
                               sensorBiases.gyro_z + 
                               gyroDrift[2] +
                               random.gauss(0, sensorSigmas.gyro_z))
        
        noisySensors.accel_x = (trueSensors.accel_x + 
                                sensorBiases.accel_x +
                                random.gauss(0, sensorSigmas.accel_x))
        noisySensors.accel_y = (trueSensors.accel_y + 
                                sensorBiases.accel_y +
                                random.gauss(0, sensorSigmas.accel_y))
        noisySensors.accel_z = (trueSensors.accel_z + 
                                sensorBiases.accel_z +
                                random.gauss(0, sensorSigmas.accel_z))
        
        noisySensors.mag_x = (trueSensors.mag_x + 
                              sensorBiases.mag_x +
                              random.gauss(0, sensorSigmas.mag_x))
        noisySensors.mag_y = (trueSensors.mag_y + 
                              sensorBiases.mag_y +
                              random.gauss(0, sensorSigmas.mag_y))
        noisySensors.mag_z = (trueSensors.mag_z + 
                              sensorBiases.mag_z +
                              random.gauss(0, sensorSigmas.mag_z))
        
        noisySensors.baro = (trueSensors.baro + 
                            sensorBiases.baro +
                            random.gauss(0, sensorSigmas.baro))
        
        noisySensors.pitot = (trueSensors.pitot + 
                             sensorBiases.pitot +
                             random.gauss(0, sensorSigmas.pitot))
        
        if self.updateTicks % self.gpsTicksUpdate == 0:
            gpsDrift = self.gpsGM.update()
            
            noisySensors.gps_n = (trueSensors.gps_n + 
                                 gpsDrift[0] +
                                 random.gauss(0, sensorSigmas.gps_n))
            noisySensors.gps_e = (trueSensors.gps_e + 
                                 gpsDrift[1] +
                                 random.gauss(0, sensorSigmas.gps_e))
            noisySensors.gps_alt = (trueSensors.gps_alt + 
                                   gpsDrift[2] +
                                   random.gauss(0, sensorSigmas.gps_alt))
            noisySensors.gps_sog = (trueSensors.gps_sog + 
                                   random.gauss(0, sensorSigmas.gps_sog))
            noisySensors.gps_cog = (trueSensors.gps_cog + 
                                   random.gauss(0, sensorSigmas.gps_cog))
        else:
            noisySensors.gps_n = self.sensorsNoisy.gps_n
            noisySensors.gps_e = self.sensorsNoisy.gps_e
            noisySensors.gps_alt = self.sensorsNoisy.gps_alt
            noisySensors.gps_sog = self.sensorsNoisy.gps_sog
            noisySensors.gps_cog = self.sensorsNoisy.gps_cog
        
        return noisySensors

    def update(self):
        # get curr state and derivative from VAM
        state = self.aeroModel.getVehicleState()
        dot = self.aeroModel.getVehicleDerivative()
        
        # update true sensors
        self.sensorsTrue = self.updateSensorsTrue(self.sensorsTrue, state, dot)
        
        # update noisy sensors
        self.sensorsNoisy = self.updateSensorsNoisy(self.sensorsTrue)
        
        # increment tick counter
        self.updateTicks += 1

    def getSensorsTrue(self):
        return self.sensorsTrue

    def getSensorsNoisy(self):
        return self.sensorsNoisy

    def setSensorsTrue(self, sensors):
        self.sensorsTrue = sensors

    def setSensorsNoisy(self, sensors):
        self.sensorsNoisy = sensors

    def reset(self):
        # reset sensor containers
        self.sensorsTrue = Sensors.vehicleSensors()
        self.sensorsNoisy = Sensors.vehicleSensors()
        
        # recalculate random static biases
        self.sensorBiases = self.initializeBiases()
        
        
        # reset Gauss-Markov processes
        self.gyroGM.reset()
        self.gpsGM.reset()
        
        # reset tick counter
        self.updateTicks = 0
