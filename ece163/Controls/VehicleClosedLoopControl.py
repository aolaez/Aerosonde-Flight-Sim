import math
import sys
import ece163.Containers.Inputs as Inputs
import ece163.Containers.Controls as Controls
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Modeling.VehicleAerodynamicsModel as VehicleAerodynamicsModule
import ece163.Controls.VehicleEstimator as VehicleEstimator
import ece163.Sensors.SensorsModel as SensorsModel


class PDControl():
    # kp * error - kd * derivative + trim
    def __init__(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return

    # for in-flight tuning
    def setPDGains(self, kp=0.0, kd=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.kp = kp
        self.kd = kd
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        error = command - current
        
        # PD control form is 
        #u = kp * error - kd * derivative + trim
        u = (self.kp * error) - (self.kd * derivative) + self.trim
        
        # saturation limits
        if u > self.highLimit:
            u = self.highLimit
        elif u < self.lowLimit:
            u = self.lowLimit
        
        return u
    
    def update(self, command=0.0, current=0.0, derivative=0.0): # weird
        return self.Update(command, current, derivative)



class PIControl():
    # u = kp * error + ki * integral(error) + trim
    def __init__(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0  
        self.prevError = 0.0
        return

    def setPIGains(self, dT=VPC.dT, kp=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.kp = kp
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return

    def Update(self, command=0.0, current=0.0):
        error = command - current
        
        # trapezoidal integration for accumulator
        self.accumulator += 0.5 * self.dT * (error + self.prevError)
        
        u_unsat = (self.kp * error) + (self.ki * self.accumulator) + self.trim
        
        # saturation and undo previous step
        u = u_unsat
        if u > self.highLimit:
            u = self.highLimit
            self.accumulator -= 0.5 * self.dT * (error + self.prevError)
        elif u < self.lowLimit:
            u = self.lowLimit
            self.accumulator -= 0.5 * self.dT * (error + self.prevError)
        
        self.prevError = error
        
        return u
    
    def update(self, command=0.0, current=0.0): # weird
        return self.Update(command, current)


    def resetIntegrator(self):
        self.accumulator = 0.0
        self.prevError = 0.0
        return


class PIDControl():
    # u = kp * error + ki * integral(error) - kd * derivative + trim
    def __init__(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        self.accumulator = 0.0 
        self.prevError = 0.0 
        return

    def setPIDGains(self, dT=VPC.dT, kp=0.0, kd=0.0, ki=0.0, trim=0.0, lowLimit=0.0, highLimit=0.0):
        self.dT = dT
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.trim = trim
        self.lowLimit = lowLimit
        self.highLimit = highLimit
        return

    def Update(self, command=0.0, current=0.0, derivative=0.0):
        error = command - current
        
        # trapezoidal integration
        self.accumulator += 0.5 * self.dT * (error + self.prevError)

        u_unsat = (self.kp * error) + (self.ki * self.accumulator) - (self.kd * derivative) + self.trim
        
        # saturation and undo previous step
        u = u_unsat
        if u > self.highLimit:
            u = self.highLimit
            self.accumulator -= 0.5 * self.dT * (error + self.prevError)
        elif u < self.lowLimit:
            u = self.lowLimit
            self.accumulator -= 0.5 * self.dT * (error + self.prevError)
        
        self.prevError = error
        
        return u
    
    def update(self, command=0.0, current=0.0, derivative=0.0): # weird
        return self.Update(command, current, derivative)


    def resetIntegrator(self):
        self.accumulator = 0.0
        self.prevError = 0.0
        return

# autopilot controller w seven feedback controllers
class VehicleClosedLoopControl():
    def __init__(self, dT=VPC.dT, rudderControlSource='SIDESLIP', useSensors=False, useEstimator=False):
        self.dT = dT
        self.rudderControlSource = rudderControlSource
        self.useSensors = useSensors
        self.useEstimator = useEstimator
        
        # create VAM model to be our plant
        self.VAM = VehicleAerodynamicsModule.VehicleAerodynamicsModel()
        
        # Create sensors model if needed
        if self.useSensors or self.useEstimator:
            self.sensorsModel = SensorsModel.SensorsModel()
        else:
            self.sensorsModel = None
        
        # Create estimator if needed
        if self.useEstimator:
            self.vehicleEstimator = VehicleEstimator.VehicleEstimator(
                dT=self.dT,
                sensorsModel=self.sensorsModel
            )
        
        # Control gains and input containers
        self.GAINS = Controls.controlGains()
        self.trimInputs = Inputs.controlInputs()
        self.VamInputs = Inputs.controlInputs()
        
        # altitude control FSM
        self.altitudeState = Controls.AltitudeStates.HOLDING
        
        # create 7 controllers
        self.rollFromCourse = PIControl()
        self.rudderFromSideslip = PIControl()
        self.throttleFromAirspeed = PIControl()
        self.pitchFromAltitude = PIControl()
        self.pitchFromAirspeed = PIControl()
        self.elevatorFromPitch = PDControl()
        self.aileronFromRoll = PIDControl()
        
        return
    
    def reset(self):
        self.rollFromCourse.resetIntegrator()
        self.rudderFromSideslip.resetIntegrator()
        self.throttleFromAirspeed.resetIntegrator()
        self.pitchFromAltitude.resetIntegrator()
        self.pitchFromAirspeed.resetIntegrator()
        self.aileronFromRoll.resetIntegrator()
        self.VAM.reset()
        
        # Reset sensors if using them
        if self.sensorsModel is not None:
            self.sensorsModel.reset()
        
        # Reset estimator if using it
        if self.useEstimator:
            self.vehicleEstimator.reset()
        
        return
    
    def getControlGains(self):
        return self.GAINS

    def setControlGains(self, controlGains=Controls.controlGains()):
        self.GAINS = controlGains
        
        # outputs commanded roll angle
        self.rollFromCourse.setPIGains(
            dT=self.dT,
            kp=self.GAINS.kp_course,
            ki=self.GAINS.ki_course,
            trim=0.0,
            lowLimit=-math.radians(VPC.bankAngleLimit),
            highLimit=math.radians(VPC.bankAngleLimit)
        )
        
        # outputs rudder deflection
        self.rudderFromSideslip.setPIGains(
            dT=self.dT,
            kp=self.GAINS.kp_sideslip,
            ki=self.GAINS.ki_sideslip,
            trim=self.trimInputs.Rudder,
            lowLimit=VPC.minControls.Rudder,
            highLimit=VPC.maxControls.Rudder
        )
        
        # outputs throttle setting
        self.throttleFromAirspeed.setPIGains(
            dT=self.dT,
            kp=self.GAINS.kp_SpeedfromThrottle,
            ki=self.GAINS.ki_SpeedfromThrottle,
            trim=self.trimInputs.Throttle,
            lowLimit=VPC.minControls.Throttle,
            highLimit=VPC.maxControls.Throttle
        )
        
        # outputs commanded pitch angle from altitude
        self.pitchFromAltitude.setPIGains(
            dT=self.dT,
            kp=self.GAINS.kp_altitude,
            ki=self.GAINS.ki_altitude,
            trim=0.0,
            lowLimit=-math.radians(VPC.pitchAngleLimit),
            highLimit=math.radians(VPC.pitchAngleLimit)
        )
        
        # outputs commanded pitch angle from speed
        self.pitchFromAirspeed.setPIGains(
            dT=self.dT,
            kp=self.GAINS.kp_SpeedfromElevator,
            ki=self.GAINS.ki_SpeedfromElevator,
            trim=0.0,
            lowLimit=-math.radians(VPC.pitchAngleLimit),
            highLimit=math.radians(VPC.pitchAngleLimit)
        )
        
        # outputs elevator deflection
        self.elevatorFromPitch.setPDGains(
            kp=self.GAINS.kp_pitch,
            kd=self.GAINS.kd_pitch,
            trim=self.trimInputs.Elevator,
            lowLimit=VPC.minControls.Elevator,
            highLimit=VPC.maxControls.Elevator
        )
        
        # outputs aileron deflection
        self.aileronFromRoll.setPIDGains(
            dT=self.dT,
            kp=self.GAINS.kp_roll,
            kd=self.GAINS.kd_roll,
            ki=self.GAINS.ki_roll,
            trim=self.trimInputs.Aileron,
            lowLimit=VPC.minControls.Aileron,
            highLimit=VPC.maxControls.Aileron
        )
        
        return

    def getVehicleState(self):
        return self.VAM.getVehicleState()

    def setVehicleState(self, state):
        self.VAM.setVehicleState(state)
        return 

    def getTrimInputs(self):
        return self.trimInputs

    def setTrimInputs(self, trimInputs=Inputs.controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)):
        self.trimInputs = trimInputs
        return 
    
    def getVehicleAerodynamicsModel(self):
        return self.VAM
    
    def getVehicleControlSurfaces(self):
        return self.VamInputs
    
    def getVehicleEstimator(self):
        if self.useEstimator:
            return self.vehicleEstimator
        else:
            return None
    
    def UpdateControlCommands(self, referenceCommands, state):
        # altitude control FSM
        # output container
        inputs = Inputs.controlInputs()
        
        # course wrapping, if course error more than pi took long route
        courseError = referenceCommands.commandedCourse - state.chi
        if courseError >= math.pi:
            state.chi += 2.0 * math.pi
        elif courseError <= -math.pi:
            state.chi -= 2.0 * math.pi
        
        # define altitude thresholds
        lower_thresh = referenceCommands.commandedAltitude - VPC.altitudeHoldZone
        upper_thresh = referenceCommands.commandedAltitude + VPC.altitudeHoldZone
        altitude = -state.pd  # convert NE(D) to altitude
        
        pitchCommand = referenceCommands.commandedPitch
        
        # altitude FSM START
        if self.altitudeState == Controls.AltitudeStates.HOLDING:
            # HOLDING mode:
            # - Pitch controlled by altitude
            # - Throttle controlled by airspeed
            pitchCommand = self.pitchFromAltitude.update(
                referenceCommands.commandedAltitude,
                altitude
            )
            inputs.Throttle = self.throttleFromAirspeed.update(
                referenceCommands.commandedAirspeed,
                state.Va
            )
            
            # transitions
            if altitude > upper_thresh:
                # start DESCENDING bc too high
                self.altitudeState = Controls.AltitudeStates.DESCENDING
                inputs.Throttle = VPC.minControls.Throttle
                pitchCommand = self.pitchFromAirspeed.update(referenceCommands.commandedAirspeed, state.Va)
                self.pitchFromAirspeed.resetIntegrator()
                
            elif altitude < lower_thresh:
                # start CLIMBING bc too low
                self.altitudeState = Controls.AltitudeStates.CLIMBING
                inputs.Throttle = VPC.maxControls.Throttle
                pitchCommand = self.pitchFromAirspeed.update(referenceCommands.commandedAirspeed, state.Va)
                self.pitchFromAirspeed.resetIntegrator()
        
        elif self.altitudeState == Controls.AltitudeStates.DESCENDING:
            # DESCENDING mode:
            # - Pitch controlled by airspeed
            # - Throttle set to minimum
            pitchCommand = self.pitchFromAirspeed.update(referenceCommands.commandedAirspeed, state.Va)
            inputs.Throttle = VPC.minControls.Throttle
            
            # transition back to holding
            if lower_thresh < altitude < upper_thresh:
                self.altitudeState = Controls.AltitudeStates.HOLDING
                self.pitchFromAltitude.resetIntegrator()
        
        elif self.altitudeState == Controls.AltitudeStates.CLIMBING:
            # CLIMBING mode:
            # - Pitch controlled by airspeed
            # - Throttle set to maximum
            pitchCommand = self.pitchFromAirspeed.update(referenceCommands.commandedAirspeed, state.Va)
            inputs.Throttle = VPC.maxControls.Throttle
            
            # transition back to HOLDING
            if lower_thresh < altitude < upper_thresh:
                self.altitudeState = Controls.AltitudeStates.HOLDING
                self.pitchFromAltitude.resetIntegrator()
        
        # lateral control
        
        # roll from course error
        rollCommand = self.rollFromCourse.update(
            referenceCommands.commandedCourse,
            state.chi
        )
        
        # aileron command from roll error 
        inputs.Aileron = self.aileronFromRoll.update(
            rollCommand,
            state.roll,
            state.p 
        )
        
        # rudder command from sideslip (coordinated turn)
        inputs.Rudder = self.rudderFromSideslip.update(
            0.0,  
            state.beta
        )
        
        # longitudinal control
        
        # elevator command from pitch error 
        inputs.Elevator = self.elevatorFromPitch.update(
            pitchCommand,
            state.pitch,
            state.q  
        )
        
        referenceCommands.commandedPitch = pitchCommand
        referenceCommands.commandedRoll = rollCommand
        
        self.VamInputs = inputs
        
        return inputs

    # get curr state, compute control commands, apply them, update state
    def update(self, referenceCommands=Controls.referenceCommands()):
        # Determine which state to use for control
        if self.useEstimator:
            # Update sensors first
            if self.sensorsModel is not None:
                self.sensorsModel.update()
            
            # Update estimator
            self.vehicleEstimator.Update()
            
            # Use estimated state for control
            controlState = self.vehicleEstimator.getEstimatedState()
        else:
            # Use true state from aerodynamics model
            controlState = self.VAM.getVehicleState()
        
        # Compute control commands using appropriate state
        inputs = self.UpdateControlCommands(referenceCommands, controlState)
        
        # Apply controls to vehicle
        self.VAM.Update(inputs)
        
        return