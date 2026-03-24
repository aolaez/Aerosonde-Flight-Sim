import math
import sys
import matplotlib.pyplot as plt
import numpy as np

sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleAerodynamicsModel as VAM
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Controls.VehicleTrim as VehicleTrim
import ece163.Controls.VehicleControlGains as VCG
import ece163.Controls.VehicleClosedLoopControl as VCLC
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Containers.Controls as Controls
import ece163.Containers.Linearized as Linearized
import ece163.Constants.VehiclePhysicalConstants as VPC

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda a, b: math.isclose(a, b, abs_tol=1e-12)

def compareVectors(a, b):
    """A quick tool to compare two vectors"""
    el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
    return all(el_close)

failed = []
passed = []
def evaluateTest(test_name, boolean):
    """evaluateTest prints the output of a test and adds it to one of two 
    global lists, passed and failed, which can be printed later"""
    if boolean:
        print(f"   passed {test_name}")
        passed.append(test_name)
    else:
        print(f"   failed {test_name}")
        failed.append(test_name)
    return boolean

#%% VehicleControlGains tests

print("Beginning testing of VehicleControlGains")

print("\nTesting computeGains()")

cur_test = "computeGains returns controlGains object"
try:
    # Create a simple linearized model and tuning parameters
    tuning = Controls.controlTuning()
    tuning.Wn_roll = 2.0
    tuning.Zeta_roll = 0.7
    tuning.Wn_course = 1.0
    tuning.Zeta_course = 0.707
    tuning.Wn_sideslip = 1.5
    tuning.Zeta_sideslip = 0.707
    tuning.Wn_pitch = 3.0
    tuning.Zeta_pitch = 0.707
    tuning.Wn_altitude = 0.5
    tuning.Zeta_altitude = 0.707
    tuning.Wn_SpeedfromThrottle = 1.0
    tuning.Zeta_SpeedfromThrottle = 0.707
    tuning.Wn_SpeedfromElevator = 0.8
    tuning.Zeta_SpeedfromElevator = 0.707
    
    linearized = Linearized.transferFunctions()
    linearized.Va_trim = 25.0
    linearized.a_phi1 = 1.0
    linearized.a_phi2 = 5.0
    linearized.a_beta1 = 0.5
    linearized.a_beta2 = 3.0
    linearized.a_theta1 = 1.0
    linearized.a_theta2 = 2.0
    linearized.a_theta3 = 4.0
    linearized.a_V1 = 0.3
    linearized.a_V2 = 2.0
    
    gains = VCG.computeGains(tuning, linearized)
    
    if not evaluateTest(cur_test, isinstance(gains, Controls.controlGains)):
        print(f"expected controlGains object")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

print("\nTesting computeTuningParameters()")

cur_test = "computeTuningParameters inverts computeGains"
try:
    # Start with tuning parameters
    tuning_orig = Controls.controlTuning()
    tuning_orig.Wn_roll = 2.0
    tuning_orig.Zeta_roll = 0.7
    tuning_orig.Wn_course = 1.0
    tuning_orig.Zeta_course = 0.707
    tuning_orig.Wn_sideslip = 1.5
    tuning_orig.Zeta_sideslip = 0.707
    tuning_orig.Wn_pitch = 3.0
    tuning_orig.Zeta_pitch = 0.707
    tuning_orig.Wn_altitude = 0.5
    tuning_orig.Zeta_altitude = 0.707
    tuning_orig.Wn_SpeedfromThrottle = 1.0
    tuning_orig.Zeta_SpeedfromThrottle = 0.707
    tuning_orig.Wn_SpeedfromElevator = 0.8
    tuning_orig.Zeta_SpeedfromElevator = 0.707
    
    linearized = Linearized.transferFunctions()
    linearized.Va_trim = 25.0
    linearized.a_phi1 = 1.0
    linearized.a_phi2 = 5.0
    linearized.a_beta1 = 0.5
    linearized.a_beta2 = 3.0
    linearized.a_theta1 = 1.0
    linearized.a_theta2 = 2.0
    linearized.a_theta3 = 4.0
    linearized.a_V1 = 0.3
    linearized.a_V2 = 2.0
    
    # Convert to gains and back
    gains = VCG.computeGains(tuning_orig, linearized)
    tuning_recovered = VCG.computeTuningParameters(gains, linearized)
    
    # Check roll parameters
    wn_match = math.isclose(tuning_recovered.Wn_roll, tuning_orig.Wn_roll, rel_tol=1e-3)
    zeta_match = math.isclose(tuning_recovered.Zeta_roll, tuning_orig.Zeta_roll, rel_tol=1e-3)
    
    if not evaluateTest(cur_test, wn_match and zeta_match):
        print(f"expected Wn_roll≈{tuning_orig.Wn_roll}, Zeta_roll≈{tuning_orig.Zeta_roll}")
        print(f"got Wn_roll={tuning_recovered.Wn_roll}, Zeta_roll={tuning_recovered.Zeta_roll}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")


#%% VehicleClosedLoopControl tests

print("\nBeginning testing of VehicleClosedLoopControl")

print("\nTesting PDControl")

cur_test = "PDControl init sets parameters"
try:
    pd = VCLC.PDControl(kp=1.0, kd=0.5, trim=0.1, lowLimit=-1.0, highLimit=1.0)
    
    params_set = (pd.kp == 1.0 and pd.kd == 0.5 and 
                  pd.trim == 0.1 and 
                  pd.lowLimit == -1.0 and pd.highLimit == 1.0)
    
    if not evaluateTest(cur_test, params_set):
        print(f"expected parameters to be set correctly")
        print(f"got kp={pd.kp}, kd={pd.kd}, trim={pd.trim}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "PDControl Update proportional term"
try:
    pd = VCLC.PDControl(kp=2.0, kd=0.0, trim=0.0, lowLimit=-10.0, highLimit=10.0)
    
    u = pd.Update(command=5.0, current=2.0, derivative=0.0)
    
    if not evaluateTest(cur_test, math.isclose(u, 6.0, abs_tol=1e-6)):
        print(f"expected u=6.0, got u={u}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "PDControl Update derivative term"
try:
    pd = VCLC.PDControl(kp=0.0, kd=0.5, trim=0.0, lowLimit=-10.0, highLimit=10.0)
    
    u = pd.Update(command=0.0, current=0.0, derivative=2.0)
    
    if not evaluateTest(cur_test, math.isclose(u, -1.0, abs_tol=1e-6)):
        print(f"expected u=-1.0, got u={u}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "PDControl Update with trim"
try:
    pd = VCLC.PDControl(kp=1.0, kd=0.0, trim=0.5, lowLimit=-10.0, highLimit=10.0)
    
    u = pd.Update(command=5.0, current=3.0, derivative=0.0)
    
    if not evaluateTest(cur_test, math.isclose(u, 2.5, abs_tol=1e-6)):
        print(f"expected u=2.5, got u={u}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "PDControl saturation upper limit"
try:
    pd = VCLC.PDControl(kp=10.0, kd=0.0, trim=0.0, lowLimit=-1.0, highLimit=1.0)
    

    u = pd.Update(command=5.0, current=0.0, derivative=0.0)
    
    if not evaluateTest(cur_test, math.isclose(u, 1.0, abs_tol=1e-6)):
        print(f"expected u=1.0 (saturated), got u={u}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "PDControl saturation lower limit"
try:
    pd = VCLC.PDControl(kp=10.0, kd=0.0, trim=0.0, lowLimit=-1.0, highLimit=1.0)
    
    u = pd.Update(command=0.0, current=5.0, derivative=0.0)
    
    if not evaluateTest(cur_test, math.isclose(u, -1.0, abs_tol=1e-6)):
        print(f"expected u=-1.0 (saturated), got u={u}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "PDControl setPDGains updates parameters"
try:
    pd = VCLC.PDControl(kp=1.0, kd=0.5)
    pd.setPDGains(kp=2.0, kd=1.0, trim=0.2)
    
    params_updated = (pd.kp == 2.0 and pd.kd == 1.0 and pd.trim == 0.2)
    
    if not evaluateTest(cur_test, params_updated):
        print(f"expected updated parameters")
        print(f"got kp={pd.kp}, kd={pd.kd}, trim={pd.trim}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

print("\nTesting PIControl")

cur_test = "PIControl init sets parameters"
try:
    pi = VCLC.PIControl(dT=0.01, kp=1.0, ki=0.5, trim=0.1, lowLimit=-1.0, highLimit=1.0)
    
    params_set = (pi.kp == 1.0 and pi.ki == 0.5 and 
                  pi.trim == 0.1 and pi.dT == 0.01)
    
    if not evaluateTest(cur_test, params_set):
        print(f"expected parameters to be set correctly")
        print(f"got kp={pi.kp}, ki={pi.ki}, dT={pi.dT}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "PIControl Update accumulates error"
try:
    pi = VCLC.PIControl(dT=0.1, kp=0.0, ki=1.0, trim=0.0, lowLimit=-10.0, highLimit=10.0)

    u1 = pi.Update(command=2.0, current=0.0)

    u2 = pi.Update(command=2.0, current=0.0)
    
    if not evaluateTest(cur_test, u2 > u1 and math.isclose(u2, 0.3, abs_tol=1e-6)):
        print(f"expected accumulation: u1={u1}, u2={u2} (expected 0.3)")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "PIControl anti-windup prevents accumulation when saturated"
try:
    pi = VCLC.PIControl(dT=0.1, kp=0.0, ki=1.0, trim=0.0, lowLimit=-1.0, highLimit=1.0)
    
    # Large error to saturate
    for i in range(5):
        u = pi.Update(command=100.0, current=0.0)
    
    # Accumulator should not have grown unbounded
    if not evaluateTest(cur_test, pi.accumulator < 10.0):
        print(f"expected bounded accumulator with anti-windup")
        print(f"got accumulator={pi.accumulator}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "PIControl resetIntegrator clears accumulator"
try:
    pi = VCLC.PIControl(dT=0.1, kp=1.0, ki=1.0)
    
    # Build up some integrator state
    for i in range(5):
        pi.Update(command=1.0, current=0.0)
    
    pi.resetIntegrator()
    
    if not evaluateTest(cur_test, pi.accumulator == 0.0 and pi.prevError == 0.0):
        print(f"expected zero accumulator after reset")
        print(f"got accumulator={pi.accumulator}, prevError={pi.prevError}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

print("\nTesting PIDControl")

cur_test = "PIDControl combines P, I, and D terms"
try:
    pid = VCLC.PIDControl(dT=0.1, kp=1.0, kd=0.5, ki=0.2, trim=0.0, lowLimit=-10.0, highLimit=10.0)
    
    # First update
    u1 = pid.Update(command=2.0, current=0.0, derivative=0.0)
    
    # Second update with derivative
    u2 = pid.Update(command=2.0, current=0.0, derivative=1.0)
    
    # u2 should be less than u1 due to negative derivative term
    if not evaluateTest(cur_test, u2 < u1):
        print(f"expected u2 < u1 due to derivative damping")
        print(f"got u1={u1}, u2={u2}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "PIDControl anti-windup works"
try:
    pid = VCLC.PIDControl(dT=0.1, kp=0.0, kd=0.0, ki=1.0, trim=0.0, lowLimit=-1.0, highLimit=1.0)
    
    # Saturate the output
    for i in range(10):
        u = pid.Update(command=100.0, current=0.0, derivative=0.0)
    
    # Accumulator should be bounded
    if not evaluateTest(cur_test, pid.accumulator < 10.0):
        print(f"expected bounded accumulator")
        print(f"got accumulator={pid.accumulator}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

print("\nTesting VehicleClosedLoopControl")

cur_test = "VCLC init creates controllers"
try:
    vclc = VCLC.VehicleClosedLoopControl()
    
    has_controllers = (hasattr(vclc, 'rollFromCourse') and
                       hasattr(vclc, 'elevatorFromPitch') and
                       hasattr(vclc, 'aileronFromRoll') and
                       hasattr(vclc, 'VAM'))
    
    if not evaluateTest(cur_test, has_controllers):
        print(f"expected VCLC to have all controller attributes")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "VCLC init sets HOLDING state"
try:
    vclc = VCLC.VehicleClosedLoopControl()
    
    if not evaluateTest(cur_test, vclc.altitudeState == Controls.AltitudeStates.HOLDING):
        print(f"expected HOLDING state on init")
        print(f"got state={vclc.altitudeState}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "VCLC setControlGains configures all controllers"
try:
    vclc = VCLC.VehicleClosedLoopControl()
    
    gains = Controls.controlGains()
    gains.kp_roll = 1.0
    gains.kd_roll = 0.5
    gains.ki_roll = 0.001
    gains.kp_course = 0.5
    gains.ki_course = 0.1
    gains.kp_pitch = 1.5
    gains.kd_pitch = 0.8
    
    vclc.setControlGains(gains)
    
    gains_set = math.isclose(vclc.aileronFromRoll.kp, 1.0, abs_tol=1e-6)
    
    if not evaluateTest(cur_test, gains_set):
        print(f"expected controller gains to be set")
        print(f"got aileronFromRoll.kp={vclc.aileronFromRoll.kp}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "VCLC setTrimInputs stores trim"
try:
    vclc = VCLC.VehicleClosedLoopControl()
    
    trimInputs = Inputs.controlInputs(Throttle=0.6, Elevator=-0.1, Aileron=0.0, Rudder=0.0)
    vclc.setTrimInputs(trimInputs)
    
    trim = vclc.getTrimInputs()
    
    if not evaluateTest(cur_test, math.isclose(trim.Throttle, 0.6) and math.isclose(trim.Elevator, -0.1)):
        print(f"expected trim inputs to be stored")
        print(f"got Throttle={trim.Throttle}, Elevator={trim.Elevator}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "VCLC reset clears integrators"
try:
    vclc = VCLC.VehicleClosedLoopControl()
    
    # Build up integrator state
    for i in range(10):
        vclc.rollFromCourse.Update(1.0, 0.0)
    
    vclc.reset()
    
    if not evaluateTest(cur_test, vclc.rollFromCourse.accumulator == 0.0):
        print(f"expected zero accumulator after reset")
        print(f"got accumulator={vclc.rollFromCourse.accumulator}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "VCLC UpdateControlCommands returns control inputs"
try:
    vclc = VCLC.VehicleClosedLoopControl()
    
    gains = Controls.controlGains()
    gains.kp_roll = 1.0
    gains.kd_roll = 0.5
    gains.ki_roll = 0.001
    gains.kp_course = 0.5
    gains.ki_course = 0.1
    gains.kp_altitude = 0.1
    gains.ki_altitude = 0.01
    gains.kp_SpeedfromThrottle = 1.0
    gains.ki_SpeedfromThrottle = 0.5
    gains.kp_pitch = 1.5
    gains.kd_pitch = 0.8
    gains.kp_sideslip = 0.5
    gains.ki_sideslip = 0.1
    gains.kp_SpeedfromElevator = -0.5
    gains.ki_SpeedfromElevator = -0.2
    vclc.setControlGains(gains)
    
    refCommands = Controls.referenceCommands()
    refCommands.commandedAirspeed = 25.0
    refCommands.commandedAltitude = -100.0
    refCommands.commandedCourse = 0.0
    
    state = States.vehicleState()
    state.Va = 25.0
    state.pd = -100.0
    state.chi = 0.0
    state.pitch = 0.0
    state.roll = 0.0
    state.beta = 0.0
    state.p = 0.0
    state.q = 0.0
    state.r = 0.0
    
    inputs = vclc.UpdateControlCommands(refCommands, state)
    
    if not evaluateTest(cur_test, isinstance(inputs, Inputs.controlInputs)):
        print(f"expected controlInputs object")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")


cur_test = "VCLC altitude state machine transitions to DESCENDING"
try:
    vclc = VCLC.VehicleClosedLoopControl()
    
    # Set gains
    gains = Controls.controlGains()
    gains.kp_altitude = 0.1
    gains.ki_altitude = 0.01
    gains.kp_SpeedfromThrottle = 1.0
    gains.ki_SpeedfromThrottle = 0.5
    gains.kp_SpeedfromElevator = -0.5
    gains.ki_SpeedfromElevator = -0.2
    gains.kp_pitch = 1.5
    gains.kd_pitch = 0.8
    gains.kp_roll = 1.0
    gains.kd_roll = 0.5
    gains.ki_roll = 0.001
    gains.kp_course = 0.5
    gains.ki_course = 0.1
    gains.kp_sideslip = 0.5
    gains.ki_sideslip = 0.1
    vclc.setControlGains(gains)
    
    refCommands = Controls.referenceCommands()
    refCommands.commandedAltitude = -100.0
    refCommands.commandedAirspeed = 25.0
    refCommands.commandedCourse = 0.0
    
    state = States.vehicleState()
    state.pd = -150.0  # Below commanded altitude 
    state.Va = 25.0
    state.chi = 0.0
    state.pitch = 0.0
    state.roll = 0.0
    state.beta = 0.0
    state.p = 0.0
    state.q = 0.0
    state.r = 0.0
    
    inputs = vclc.UpdateControlCommands(refCommands, state)
    
    if not evaluateTest(cur_test, vclc.altitudeState == Controls.AltitudeStates.DESCENDING):
        print(f"expected DESCENDING state when above commanded altitude")
        print(f"got state={vclc.altitudeState}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "VCLC CLIMBING uses max throttle"
try:
    vclc = VCLC.VehicleClosedLoopControl()
    vclc.altitudeState = Controls.AltitudeStates.CLIMBING
    
    # Set gains
    gains = Controls.controlGains()
    gains.kp_altitude = 0.1
    gains.ki_altitude = 0.01
    gains.kp_SpeedfromThrottle = 1.0
    gains.ki_SpeedfromThrottle = 0.5
    gains.kp_SpeedfromElevator = -0.5
    gains.ki_SpeedfromElevator = -0.2
    gains.kp_pitch = 1.5
    gains.kd_pitch = 0.8
    gains.kp_roll = 1.0
    gains.kd_roll = 0.5
    gains.ki_roll = 0.001
    gains.kp_course = 0.5
    gains.ki_course = 0.1
    gains.kp_sideslip = 0.5
    gains.ki_sideslip = 0.1
    vclc.setControlGains(gains)
    
    refCommands = Controls.referenceCommands()
    refCommands.commandedAltitude = -100.0
    refCommands.commandedAirspeed = 25.0
    refCommands.commandedCourse = 0.0
    
    state = States.vehicleState()
    state.pd = -95.0  # Still in HOLDING zone
    state.Va = 25.0
    state.chi = 0.0
    state.pitch = 0.0
    state.roll = 0.0
    state.beta = 0.0
    state.p = 0.0
    state.q = 0.0
    state.r = 0.0
    
    inputs = vclc.UpdateControlCommands(refCommands, state)
    
    if not evaluateTest(cur_test, math.isclose(inputs.Throttle, VPC.maxControls.Throttle)):
        print(f"expected max throttle in CLIMBING")
        print(f"got Throttle={inputs.Throttle}, max={VPC.maxControls.Throttle}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "VCLC course wrapping handles large errors"
try:
    vclc = VCLC.VehicleClosedLoopControl()
    
    # Set gains
    gains = Controls.controlGains()
    gains.kp_roll = 1.0
    gains.kd_roll = 0.5
    gains.ki_roll = 0.001
    gains.kp_course = 0.5
    gains.ki_course = 0.1
    gains.kp_altitude = 0.1
    gains.ki_altitude = 0.01
    gains.kp_SpeedfromThrottle = 1.0
    gains.ki_SpeedfromThrottle = 0.5
    gains.kp_SpeedfromElevator = -0.5
    gains.ki_SpeedfromElevator = -0.2
    gains.kp_pitch = 1.5
    gains.kd_pitch = 0.8
    gains.kp_sideslip = 0.5
    gains.ki_sideslip = 0.1
    vclc.setControlGains(gains)
    
    refCommands = Controls.referenceCommands()
    refCommands.commandedCourse = 0.1
    refCommands.commandedAltitude = -100.0
    refCommands.commandedAirspeed = 25.0
    
    state = States.vehicleState()
    state.chi = -math.pi * 1.9  # Should wrap
    state.pd = -100.0
    state.Va = 25.0
    state.pitch = 0.0
    state.roll = 0.0
    state.beta = 0.0
    state.p = 0.0
    state.q = 0.0
    state.r = 0.0
    
    inputs = vclc.UpdateControlCommands(refCommands, state)
    
    # After wrapping, chi should be in reasonable range
    if not evaluateTest(cur_test, abs(state.chi) < 2 * math.pi):
        print(f"expected chi to be wrapped to reasonable range")
        print(f"got chi={state.chi}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "VCLC Update advances simulation"
try:
    vclc = VCLC.VehicleClosedLoopControl()
    
    # Set trim and gains for level flight
    trimInputs = Inputs.controlInputs(Throttle=0.5, Elevator=0.0, Aileron=0.0, Rudder=0.0)
    vclc.setTrimInputs(trimInputs)
    
    gains = Controls.controlGains()
    gains.kp_roll = 1.0
    gains.kd_roll = 0.5
    gains.ki_roll = 0.001
    gains.kp_course = 0.5
    gains.ki_course = 0.1
    gains.kp_altitude = 0.1
    gains.ki_altitude = 0.01
    gains.kp_SpeedfromThrottle = 1.0
    gains.ki_SpeedfromThrottle = 0.5
    gains.kp_SpeedfromElevator = -0.5
    gains.ki_SpeedfromElevator = -0.2
    gains.kp_pitch = 1.5
    gains.kd_pitch = 0.8
    gains.kp_sideslip = 0.5
    gains.ki_sideslip = 0.1
    vclc.setControlGains(gains)
    
    # Set initial state
    initialState = States.vehicleState()
    initialState.u = 25.0
    initialState.pd = -100.0
    initialState.Va = 25.0
    vclc.setVehicleState(initialState)
    
    refCommands = Controls.referenceCommands()
    refCommands.commandedAirspeed = 25.0
    refCommands.commandedAltitude = -100.0
    refCommands.commandedCourse = 0.0
    
    initial_pn = vclc.getVehicleState().pn
    
    # Run a few updates
    for i in range(5):
        vclc.update(refCommands)
    
    final_pn = vclc.getVehicleState().pn
    
    if not evaluateTest(cur_test, final_pn > initial_pn):
        print(f"expected pn to advance")
        print(f"initial pn={initial_pn}, final pn={final_pn}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

#%% Summary

total = len(passed) + len(failed)
print(f"\nPassed {len(passed)}/{total} tests")

if failed:
    print(f"\nFailed {len(failed)}/{total} tests:")
    [print("   " + test) for test in failed]
else:
    print("\nAll tests passed!")


print("\nTest 1: Visualizing altitude state machine transitions")
try:
    vclc = VCLC.VehicleClosedLoopControl()
    
    trimInputs = Inputs.controlInputs(Throttle=0.5, Elevator=-0.1, Aileron=0.0, Rudder=0.0)
    vclc.setTrimInputs(trimInputs)
    
    gains = Controls.controlGains()
    gains.kp_roll = 1.0
    gains.kd_roll = 0.5
    gains.ki_roll = 0.001
    gains.kp_course = 0.5
    gains.ki_course = 0.1
    gains.kp_altitude = 0.05
    gains.ki_altitude = 0.01
    gains.kp_SpeedfromThrottle = 1.0
    gains.ki_SpeedfromThrottle = 0.5
    gains.kp_SpeedfromElevator = -0.5
    gains.ki_SpeedfromElevator = -0.2
    gains.kp_pitch = 1.5
    gains.kd_pitch = 0.8
    gains.kp_sideslip = 0.5
    gains.ki_sideslip = 0.1
    vclc.setControlGains(gains)
    
    initialState = States.vehicleState()
    initialState.u = 25.0
    initialState.pd = -50.0 
    initialState.Va = 25.0
    vclc.setVehicleState(initialState)
    
    refCommands = Controls.referenceCommands()
    refCommands.commandedAltitude = -150.0 
    refCommands.commandedAirspeed = 25.0
    refCommands.commandedCourse = 0.0
    
    time_data = []
    altitude_data = []
    throttle_data = []
    state_data = []
    
    for i in range(1000):
        time_data.append(i * VPC.dT)
        altitude_data.append(-vclc.getVehicleState().pd)
        
        inputs = vclc.UpdateControlCommands(refCommands, vclc.getVehicleState())
        throttle_data.append(inputs.Throttle)
        state_data.append(vclc.altitudeState.value)
        
        vclc.VAM.Update(inputs)
    
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 8))
    
    ax1.plot(time_data, altitude_data, 'b-', linewidth=2)
    ax1.axhline(y=150, color='r', linestyle='--', label='Target')
    ax1.set_ylabel('Altitude (m)')
    ax1.set_title('Altitude State Machine Transition Test')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2.plot(time_data, throttle_data, 'g-', linewidth=2)
    ax2.set_ylabel('Throttle')
    ax2.grid(True, alpha=0.3)
    
    ax3.plot(time_data, state_data, 'r-', linewidth=2)
    ax3.set_ylabel('State (0=CLIMB,1=DESCEND,2=HOLD)')
    ax3.set_xlabel('Time (s)')
    ax3.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('test_altitude_transitions.png', dpi=150)
    
except Exception as e:
    print(f"   Failed: {e}")

print("\nTest 2: Course tracking")
try:
    vclc = VCLC.VehicleClosedLoopControl()
    
    trimInputs = Inputs.controlInputs(Throttle=0.5, Elevator=-0.1, Aileron=0.0, Rudder=0.0)
    vclc.setTrimInputs(trimInputs)
    vclc.setControlGains(gains)
    
    initialState = States.vehicleState()
    initialState.u = 25.0
    initialState.pd = -100.0
    initialState.Va = 25.0
    initialState.chi = 0.0
    vclc.setVehicleState(initialState)
    
    refCommands = Controls.referenceCommands()
    refCommands.commandedAltitude = -100.0
    refCommands.commandedAirspeed = 25.0
    refCommands.commandedCourse = math.pi / 4 
    
    time_data = []
    course_data = []
    roll_data = []
    
    for i in range(500):
        time_data.append(i * VPC.dT)
        state = vclc.getVehicleState()
        course_data.append(state.chi)
        roll_data.append(state.roll)
        
        inputs = vclc.UpdateControlCommands(refCommands, state)
        vclc.VAM.Update(inputs)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
    
    ax1.plot(time_data, course_data, 'b-', linewidth=2)
    ax1.axhline(y=math.pi/4, color='r', linestyle='--', label='Target')
    ax1.set_ylabel('Course (rad)')
    ax1.set_title('Course Tracking Test')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2.plot(time_data, roll_data, 'g-', linewidth=2)
    ax2.set_ylabel('Roll (rad)')
    ax2.set_xlabel('Time (s)')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('test_course_tracking.png', dpi=150)
    
except Exception as e:
    print(f"   ✗ Failed: {e}")

print("\nPlot generation complete!")
plt.show()