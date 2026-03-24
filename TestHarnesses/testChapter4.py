"""This file is a test harness for the module VehicleAerodynamicsModel. 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter4.py (from the root directory) -or-
python testChapter4.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehicleAerodynamicsModel module"""


import math
import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleAerodynamicsModel as VAM
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Constants.VehiclePhysicalConstants as VPC

isclose = lambda a, b: math.isclose(a, b, abs_tol=1e-12)

def compareVectors(a, b):
    """A quick tool to compare two vectors"""
    el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
    return all(el_close)

def compareForcesMoments(fm1, fm2, tol=1e-6):
    """Compare two forcesMoments objects"""
    return (math.isclose(fm1.Fx, fm2.Fx, abs_tol=tol) and
            math.isclose(fm1.Fy, fm2.Fy, abs_tol=tol) and
            math.isclose(fm1.Fz, fm2.Fz, abs_tol=tol) and
            math.isclose(fm1.Mx, fm2.Mx, abs_tol=tol) and
            math.isclose(fm1.My, fm2.My, abs_tol=tol) and
            math.isclose(fm1.Mz, fm2.Mz, abs_tol=tol))

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

print("Beginning testing of VAM.__init__() and reset()")

cur_test = "init default values"
testVAM = VAM.VehicleAerodynamicsModel()
state = testVAM.getVehicleState()
if not evaluateTest(cur_test, isclose(state.u, VPC.InitialSpeed) and 
                     isclose(state.pd, VPC.InitialDownPosition) and
                     isclose(state.Va, VPC.InitialSpeed)):
    print(f"expected u={VPC.InitialSpeed}, pd={VPC.InitialDownPosition}, Va={VPC.InitialSpeed}")
    print(f"got u={state.u}, pd={state.pd}, Va={state.Va}")

cur_test = "init custom speed and height"
testVAM = VAM.VehicleAerodynamicsModel(initialSpeed=30.0, initialHeight=-150.0)
state = testVAM.getVehicleState()
if not evaluateTest(cur_test, isclose(state.u, 30.0) and isclose(state.pd, -150.0)):
    print(f"expected u=30.0, pd=-150.0, got u={state.u}, pd={state.pd}")

cur_test = "init sets zero v and w"
testVAM = VAM.VehicleAerodynamicsModel()
state = testVAM.getVehicleState()
if not evaluateTest(cur_test, isclose(state.v, 0.0) and isclose(state.w, 0.0)):
    print(f"expected v=0.0, w=0.0, got v={state.v}, w={state.w}")

cur_test = "reset returns to initial values"
testVAM = VAM.VehicleAerodynamicsModel(initialSpeed=20.0, initialHeight=-80.0)
newState = testVAM.getVehicleState()
newState.u = 50.0
newState.pd = -200.0
newState.v = 10.0
testVAM.setVehicleState(newState)
testVAM.reset()
state = testVAM.getVehicleState()
if not evaluateTest(cur_test, isclose(state.u, 20.0) and 
                     isclose(state.pd, -80.0) and 
                     isclose(state.v, 0.0)):
    print(f"expected u=20.0, pd=-80.0, v=0.0 after reset")
    print(f"got u={state.u}, pd={state.pd}, v={state.v}")

cur_test = "reset resets rotation matrix"
testVAM = VAM.VehicleAerodynamicsModel()
newState = testVAM.getVehicleState()
newState.roll = 0.5
newState.R = Rotations.euler2DCM(0, 0, 0.5)
testVAM.setVehicleState(newState)
testVAM.reset()
state = testVAM.getVehicleState()
identity = [[1,0,0],[0,1,0],[0,0,1]]
if not evaluateTest(cur_test, compareVectors([[state.R[0][0]],[state.R[1][1]],[state.R[2][2]]], 
                                              [[1],[1],[1]])):
    print(f"expected R to be identity after reset, got R[0][0]={state.R[0][0]}, R[1][1]={state.R[1][1]}, R[2][2]={state.R[2][2]}")

print("\nBeginning testing of VAM.gravityForces()")

cur_test = "gravity level flight"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.R = Rotations.euler2DCM(0, 0, 0) 
gravity = testVAM.gravityForces(testState)
expected_Fz = VPC.mass * VPC.g0
if not evaluateTest(cur_test, isclose(gravity.Fx, 0.0) and 
                     isclose(gravity.Fy, 0.0) and 
                     isclose(gravity.Fz, expected_Fz)):
    print(f"expected Fx=0, Fy=0, Fz={expected_Fz}")
    print(f"got Fx={gravity.Fx}, Fy={gravity.Fy}, Fz={gravity.Fz}")

cur_test = "gravity no moments"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.pitch = 0.3
testState.R = Rotations.euler2DCM(0, testState.pitch, 0)
gravity = testVAM.gravityForces(testState)
if not evaluateTest(cur_test, isclose(gravity.Mx, 0.0) and 
                     isclose(gravity.My, 0.0) and 
                     isclose(gravity.Mz, 0.0)):
    print(f"expected Mx=My=Mz=0 (gravity creates no moments)")
    print(f"got Mx={gravity.Mx}, My={gravity.My}, Mz={gravity.Mz}")

cur_test = "gravity pitched up 30deg"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.pitch = math.pi/6  # 30 degrees
testState.R = Rotations.euler2DCM(0, testState.pitch, 0)
gravity = testVAM.gravityForces(testState)
expected_Fx = -VPC.mass * VPC.g0 * math.sin(testState.pitch)
expected_Fz = VPC.mass * VPC.g0 * math.cos(testState.pitch)
if not evaluateTest(cur_test, math.isclose(gravity.Fx, expected_Fx, abs_tol=1e-6) and 
                     math.isclose(gravity.Fz, expected_Fz, abs_tol=1e-6)):
    print(f"expected Fx={expected_Fx}, Fz={expected_Fz}")
    print(f"got Fx={gravity.Fx}, Fz={gravity.Fz}")

cur_test = "gravity rolled 45deg"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.roll = math.pi/4 
testState.R = Rotations.euler2DCM(0, 0, testState.roll)
gravity = testVAM.gravityForces(testState)
expected_Fy = VPC.mass * VPC.g0 * math.sin(testState.roll)
expected_Fz = VPC.mass * VPC.g0 * math.cos(testState.roll)
if not evaluateTest(cur_test, math.isclose(gravity.Fy, expected_Fy, abs_tol=1e-6) and 
                     math.isclose(gravity.Fz, expected_Fz, abs_tol=1e-6)):
    print(f"expected Fy={expected_Fy}, Fz={expected_Fz}")
    print(f"got Fy={gravity.Fy}, Fz={gravity.Fz}")

cur_test = "gravity complex attitude"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.yaw = 0.2
testState.pitch = 0.3
testState.roll = 0.1
testState.R = Rotations.euler2DCM(testState.yaw, testState.pitch, testState.roll)
gravity = testVAM.gravityForces(testState)
magnitude = math.sqrt(gravity.Fx**2 + gravity.Fy**2 + gravity.Fz**2)
expected_magnitude = VPC.mass * VPC.g0
if not evaluateTest(cur_test, math.isclose(magnitude, expected_magnitude, abs_tol=1e-6)):
    print(f"expected magnitude={expected_magnitude}, got magnitude={magnitude}")

print("\nBeginning testing of VAM.CalculateCoeff_alpha()")

cur_test = "coeffs at zero alpha"
testVAM = VAM.VehicleAerodynamicsModel()
C_L, C_D, C_m = testVAM.CalculateCoeff_alpha(0.0)
expected_CL = VPC.CL0
expected_CD = VPC.CDp + (VPC.CL0**2)/(math.pi * VPC.AR * VPC.e)
expected_Cm = VPC.CM0
if not evaluateTest(cur_test, math.isclose(C_L, expected_CL, abs_tol=1e-6) and
                     math.isclose(C_D, expected_CD, abs_tol=1e-6) and
                     math.isclose(C_m, expected_Cm, abs_tol=1e-6)):
    print(f"expected C_L={expected_CL}, C_D={expected_CD}, C_m={expected_Cm}")
    print(f"got C_L={C_L}, C_D={C_D}, C_m={C_m}")

cur_test = "coeffs small positive alpha"
testVAM = VAM.VehicleAerodynamicsModel()
alpha = 0.1  
C_L, C_D, C_m = testVAM.CalculateCoeff_alpha(alpha)
if not evaluateTest(cur_test, C_L > VPC.CL0 and C_m < VPC.CM0): 
    print(f"expected C_L > {VPC.CL0} and C_m < {VPC.CM0}")
    print(f"got C_L={C_L}, C_m={C_m}")

cur_test = "coeffs negative alpha"
testVAM = VAM.VehicleAerodynamicsModel()
alpha = -0.05
C_L, C_D, C_m = testVAM.CalculateCoeff_alpha(alpha)
if not evaluateTest(cur_test, C_L < VPC.CL0 and C_m > VPC.CM0):
    print(f"expected C_L < {VPC.CL0} and C_m > {VPC.CM0}")
    print(f"got C_L={C_L}, C_m={C_m}")

cur_test = "coeffs at stall angle"
testVAM = VAM.VehicleAerodynamicsModel()
alpha = VPC.alpha0 
C_L, C_D, C_m = testVAM.CalculateCoeff_alpha(alpha)
if not evaluateTest(cur_test, C_D > VPC.CDp):
    print(f"expected C_D > {VPC.CDp} at stall")
    print(f"got C_D={C_D}")

cur_test = "coeffs drag increases with alpha"
testVAM = VAM.VehicleAerodynamicsModel()
C_L1, C_D1, C_m1 = testVAM.CalculateCoeff_alpha(0.05)
C_L2, C_D2, C_m2 = testVAM.CalculateCoeff_alpha(0.15)
C_L3, C_D3, C_m3 = testVAM.CalculateCoeff_alpha(0.25)
if not evaluateTest(cur_test, C_D1 < C_D2 < C_D3):
    print(f"expected drag to increase with alpha")
    print(f"got C_D at alpha=0.05: {C_D1}, 0.15: {C_D2}, 0.25: {C_D3}")

print("\nBeginning testing of VAM.aeroForces()")

cur_test = "aero zero airspeed"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.Va = 0.0
testState.alpha = 0.0
testState.beta = 0.0
aero = testVAM.aeroForces(testState)
if not evaluateTest(cur_test, isclose(aero.Fx, 0.0) and isclose(aero.Fz, 0.0) and isclose(aero.My, 0.0)):
    print(f"expected zero forces at zero airspeed")
    print(f"got Fx={aero.Fx}, Fz={aero.Fz}, My={aero.My}")

cur_test = "aero level flight no alpha"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.Va = 25.0
testState.alpha = 0.0
testState.beta = 0.0
testState.p = 0.0
testState.q = 0.0
testState.r = 0.0
aero = testVAM.aeroForces(testState)
if not evaluateTest(cur_test, aero.Fx < 0.0 and aero.Fz < 0.0):  # Drag and lift
    print(f"expected negative Fx (drag) and negative Fz (lift)")
    print(f"got Fx={aero.Fx}, Fz={aero.Fz}")


cur_test = "aero sideslip creates side force"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.Va = 25.0
testState.alpha = 0.0
testState.beta = 0.1  
testState.p = 0.0
testState.q = 0.0
testState.r = 0.0
aero = testVAM.aeroForces(testState)
if not evaluateTest(cur_test, abs(aero.Fy) > 1e-6 and abs(aero.Mz) > 1e-6):
    print(f"expected non-zero side force and yaw moment with sideslip")
    print(f"got Fy={aero.Fy}, Mz={aero.Mz}")

cur_test = "aero pitch rate creates pitch moment"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.Va = 25.0
testState.alpha = 0.05
testState.beta = 0.0
testState.p = 0.0
testState.q = 0.5  
testState.r = 0.0
aero = testVAM.aeroForces(testState)
if not evaluateTest(cur_test, abs(aero.My) > 1e-3):
    print(f"expected significant pitch moment with pitch rate")
    print(f"got My={aero.My}")

print("\nBeginning testing of VAM.CalculatePropForces()")


cur_test = "prop thrust increases with throttle"
testVAM = VAM.VehicleAerodynamicsModel()
F1, M1 = testVAM.CalculatePropForces(25.0, 0.2)
F2, M2 = testVAM.CalculatePropForces(25.0, 0.5)
F3, M3 = testVAM.CalculatePropForces(25.0, 0.8)
if not evaluateTest(cur_test, F1 < F2 < F3):
    print(f"expected thrust to increase with throttle")
    print(f"got F at throttle 0.2: {F1}, 0.5: {F2}, 0.8: {F3}")

cur_test = "prop thrust decreases with airspeed"
testVAM = VAM.VehicleAerodynamicsModel()
F1, M1 = testVAM.CalculatePropForces(10.0, 0.5)
F2, M2 = testVAM.CalculatePropForces(25.0, 0.5)
F3, M3 = testVAM.CalculatePropForces(40.0, 0.5)
if not evaluateTest(cur_test, F1 > F2 > F3):
    print(f"expected thrust to decrease with airspeed at constant throttle")
    print(f"got F at Va=10: {F1}, Va=25: {F2}, Va=40: {F3}")

print("\nBeginning testing of VAM.controlForces()")



cur_test = "control elevator creates pitch moment"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.Va = 25.0
testState.alpha = 0.05
testControls = Inputs.controlInputs(Throttle=0.0, Aileron=0.0, Elevator=0.2, Rudder=0.0)
control = testVAM.controlForces(testState, testControls)
if not evaluateTest(cur_test, control.My < 0.0):
    print(f"expected negative pitch moment from positive elevator deflection")
    print(f"got My={control.My}")

cur_test = "control aileron creates roll moment"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.Va = 25.0
testState.alpha = 0.0
testControls = Inputs.controlInputs(Throttle=0.0, Aileron=0.3, Elevator=0.0, Rudder=0.0)
control = testVAM.controlForces(testState, testControls)
if not evaluateTest(cur_test, abs(control.Mx) > 1e-3):
    print(f"expected non-zero roll moment from aileron deflection")
    print(f"got Mx={control.Mx}")

cur_test = "control rudder creates yaw moment"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.Va = 25.0
testState.alpha = 0.0
testControls = Inputs.controlInputs(Throttle=0.0, Aileron=0.0, Elevator=0.0, Rudder=0.2)
control = testVAM.controlForces(testState, testControls)
if not evaluateTest(cur_test, abs(control.Mz) > 1e-3):
    print(f"expected non-zero yaw moment from rudder deflection")
    print(f"got Mz={control.Mz}")

cur_test = "control throttle adds thrust"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.Va = 25.0
testState.alpha = 0.0
testControls1 = Inputs.controlInputs(Throttle=0.0, Aileron=0.0, Elevator=0.0, Rudder=0.0)
testControls2 = Inputs.controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)
control1 = testVAM.controlForces(testState, testControls1)
control2 = testVAM.controlForces(testState, testControls2)
if not evaluateTest(cur_test, control2.Fx > control1.Fx + 10.0):
    print(f"expected significant thrust increase with throttle")
    print(f"got Fx at throttle=0: {control1.Fx}, throttle=0.5: {control2.Fx}")

print("\nBeginning testing of VAM.updateForces()")

cur_test = "updateForces calculates Va correctly"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.u = 20.0
testState.v = 0.0
testState.w = 0.0
testControls = Inputs.controlInputs()
forces = testVAM.updateForces(testState, testControls)
if not evaluateTest(cur_test, math.isclose(testState.Va, 20.0, abs_tol=1e-6)):
    print(f"expected Va=20.0, got Va={testState.Va}")

cur_test = "updateForces calculates alpha correctly"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.u = 20.0
testState.v = 0.0
testState.w = 5.0
testControls = Inputs.controlInputs()
forces = testVAM.updateForces(testState, testControls)
expected_alpha = math.atan2(5.0, 20.0)
if not evaluateTest(cur_test, math.isclose(testState.alpha, expected_alpha, abs_tol=1e-6)):
    print(f"expected alpha={expected_alpha}, got alpha={testState.alpha}")

cur_test = "updateForces calculates beta correctly"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.u = 20.0
testState.v = 3.0
testState.w = 0.0
testControls = Inputs.controlInputs()
forces = testVAM.updateForces(testState, testControls)
Va = math.sqrt(20.0**2 + 3.0**2)
expected_beta = math.asin(3.0/Va)
if not evaluateTest(cur_test, math.isclose(testState.beta, expected_beta, abs_tol=1e-6)):
    print(f"expected beta={expected_beta}, got beta={testState.beta}")

cur_test = "updateForces sums forces correctly"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.u = 25.0
testState.v = 0.0
testState.w = 0.0
testState.p = 0.0
testState.q = 0.0
testState.r = 0.0
testState.R = Rotations.euler2DCM(0, 0, 0)
testState.Va = 25.0
testState.alpha = 0.0
testState.beta = 0.0
testControls = Inputs.controlInputs(Throttle=0.5, Aileron=0.0, Elevator=0.0, Rudder=0.0)
total_forces = testVAM.updateForces(testState, testControls)
gravity = testVAM.gravityForces(testState)
aero = testVAM.aeroForces(testState)
control = testVAM.controlForces(testState, testControls)
expected_Fx = gravity.Fx + aero.Fx + control.Fx
expected_Fz = gravity.Fz + aero.Fz + control.Fz
if not evaluateTest(cur_test, math.isclose(total_forces.Fx, expected_Fx, abs_tol=1e-6) and
                     math.isclose(total_forces.Fz, expected_Fz, abs_tol=1e-6)):
    print(f"expected Fx={expected_Fx}, Fz={expected_Fz}")
    print(f"got Fx={total_forces.Fx}, Fz={total_forces.Fz}")

cur_test = "updateForces handles complex state"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.u = 22.0
testState.v = 2.0
testState.w = 3.0
testState.p = 0.1
testState.q = 0.05
testState.r = 0.02
testState.pitch = 0.1
testState.roll = 0.05
testState.R = Rotations.euler2DCM(0, testState.pitch, testState.roll)
testControls = Inputs.controlInputs(Throttle=0.6, Aileron=0.1, Elevator=-0.05, Rudder=0.03)
forces = testVAM.updateForces(testState, testControls)
if not evaluateTest(cur_test, abs(forces.Fx) > 1.0 and abs(forces.Fz) > 10.0):
    print(f"expected reasonable force magnitudes for complex state")
    print(f"got Fx={forces.Fx}, Fz={forces.Fz}")

print("\nBeginning testing of VAM.Update()")

cur_test = "Update advances state"
testVAM = VAM.VehicleAerodynamicsModel(initialSpeed=25.0)
initial_pn = testVAM.getVehicleState().pn
testControls = Inputs.controlInputs(Throttle=0.5)
testVAM.Update(testControls)
final_pn = testVAM.getVehicleState().pn
if not evaluateTest(cur_test, final_pn > initial_pn):
    print(f"expected pn to increase after update")
    print(f"initial pn={initial_pn}, final pn={final_pn}")

cur_test = "Update with gravity only"
testVAM = VAM.VehicleAerodynamicsModel(initialSpeed=0.0, initialHeight=-100.0)
testVAM.reset()
testControls = Inputs.controlInputs(Throttle=0.0)
initial_w = testVAM.getVehicleState().w
for i in range(10):
    testVAM.Update(testControls)
final_w = testVAM.getVehicleState().w
if not evaluateTest(cur_test, final_w > initial_w + 0.5):
    print(f"expected downward acceleration from gravity")
    print(f"initial w={initial_w}, final w={final_w}")

cur_test = "Update with thrust"
testVAM = VAM.VehicleAerodynamicsModel(initialSpeed=20.0)
testVAM.reset()
testControls = Inputs.controlInputs(Throttle=0.8)
initial_u = testVAM.getVehicleState().u
for i in range(10):
    testVAM.Update(testControls)
final_u = testVAM.getVehicleState().u
if not evaluateTest(cur_test, final_u > initial_u):
    print(f"expected forward acceleration from thrust")
    print(f"initial u={initial_u}, final u={final_u}")

cur_test = "Update with elevator"
testVAM = VAM.VehicleAerodynamicsModel(initialSpeed=25.0)
testVAM.reset()
testControls = Inputs.controlInputs(Throttle=0.5, Elevator=-0.2)  # Negative = pull up
initial_q = testVAM.getVehicleState().q
for i in range(10):
    testVAM.Update(testControls)
final_q = testVAM.getVehicleState().q
if not evaluateTest(cur_test, abs(final_q) > abs(initial_q)):
    print(f"expected pitch rate to develop from elevator input")
    print(f"initial q={initial_q}, final q={final_q}")

print("\nBeginning testing of getters and setters")

cur_test = "getVehicleState returns state"
testVAM = VAM.VehicleAerodynamicsModel()
state = testVAM.getVehicleState()
if not evaluateTest(cur_test, state is not None and hasattr(state, 'u')):
    print(f"expected valid state object")

cur_test = "setVehicleState modifies state"
testVAM = VAM.VehicleAerodynamicsModel()
newState = States.vehicleState()
newState.u = 50.0
newState.v = 5.0
testVAM.setVehicleState(newState)
state = testVAM.getVehicleState()
if not evaluateTest(cur_test, isclose(state.u, 50.0) and isclose(state.v, 5.0)):
    print(f"expected u=50.0, v=5.0 after setState")
    print(f"got u={state.u}, v={state.v}")

cur_test = "getVehicleDynamicsModel returns dynamics instance"
testVAM = VAM.VehicleAerodynamicsModel()
vdm = testVAM.getVehicleDynamicsModel()
if not evaluateTest(cur_test, vdm is not None and hasattr(vdm, 'Update')):
    print(f"expected valid VehicleDynamicsModel instance")

cur_test = "getVehicleDerivative returns derivative"
testVAM = VAM.VehicleAerodynamicsModel()
testControls = Inputs.controlInputs(Throttle=0.5)
testVAM.Update(testControls)
deriv = testVAM.getVehicleDerivative()
if not evaluateTest(cur_test, deriv is not None and hasattr(deriv, 'u')):
    print(f"expected valid derivative object")

cur_test = "setState and getState are consistent"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.pn = 100.0
testState.pe = 50.0
testState.pd = -200.0
testState.u = 30.0
testState.v = 2.0
testState.w = 1.0
testVAM.setVehicleState(testState)
retrieved = testVAM.getVehicleState()
if not evaluateTest(cur_test, isclose(retrieved.pn, 100.0) and 
                     isclose(retrieved.u, 30.0) and
                     isclose(retrieved.v, 2.0)):
    print(f"expected state to match what was set")
    print(f"got pn={retrieved.pn}, u={retrieved.u}, v={retrieved.v}")

print("\nBeginning testing of WindModel")

import ece163.Modeling.WindModel as WM

cur_test = "WindModel init creates zero wind"
testWM = WM.WindModel()
wind = testWM.getWind()
if not evaluateTest(cur_test, isclose(wind.Wn, 0.0) and isclose(wind.Wu, 0.0)):
    print(f"expected zero wind on init, got Wn={wind.Wn}, Wu={wind.Wu}")

cur_test = "WindModel setWind modifies wind state"
testWM = WM.WindModel()
newWind = States.windState(Wn=5.0, We=3.0, Wd=0.0)
testWM.setWind(newWind)
wind = testWM.getWind()
if not evaluateTest(cur_test, isclose(wind.Wn, 5.0) and isclose(wind.We, 3.0)):
    print(f"expected Wn=5.0, We=3.0, got Wn={wind.Wn}, We={wind.We}")

cur_test = "WindModel Update generates gusts"
testWM = WM.WindModel(drydenParameters=VPC.DrydenLowAltitudeLight)
initial_Wu = testWM.getWind().Wu
testWM.Update()
final_Wu = testWM.getWind().Wu
if not evaluateTest(cur_test, True): 
    print(f"WindModel Update failed to execute")

cur_test = "WindModel reset zeros gusts"
testWM = WM.WindModel(drydenParameters=VPC.DrydenLowAltitudeLight)
for i in range(10):
    testWM.Update() 
testWM.reset()
wind = testWM.getWind()
if not evaluateTest(cur_test, isclose(wind.Wu, 0.0) and isclose(wind.Wv, 0.0) and isclose(wind.Ww, 0.0)):
    print(f"expected zero gusts after reset, got Wu={wind.Wu}, Wv={wind.Wv}, Ww={wind.Ww}")

print("\nBeginning testing of VAM.CalculateAirspeed()")

cur_test = "CalculateAirspeed with no wind"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.u = 25.0
testState.v = 0.0
testState.w = 0.0
testState.R = Rotations.euler2DCM(0, 0, 0)
testWind = States.windState() 
Va, alpha, beta = testVAM.CalculateAirspeed(testState, testWind)
if not evaluateTest(cur_test, isclose(Va, 25.0) and isclose(alpha, 0.0) and isclose(beta, 0.0)):
    print(f"expected Va=25.0, alpha=0.0, beta=0.0 with no wind")
    print(f"got Va={Va}, alpha={alpha}, beta={beta}")

cur_test = "CalculateAirspeed with headwind"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.u = 25.0
testState.v = 0.0
testState.w = 0.0
testState.R = Rotations.euler2DCM(0, 0, 0)
testWind = States.windState(Wn=5.0, We=0.0, Wd=0.0)  
Va, alpha, beta = testVAM.CalculateAirspeed(testState, testWind)
if not evaluateTest(cur_test, isclose(Va, 20.0)): 
    print(f"expected Va=20.0 with 5 m/s headwind")
    print(f"got Va={Va}")

cur_test = "CalculateAirspeed with crosswind"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.u = 25.0
testState.v = 0.0
testState.w = 0.0
testState.yaw = 0.0 
testState.R = Rotations.euler2DCM(0, 0, 0)
testWind = States.windState(Wn=0.0, We=5.0, Wd=0.0)  
Va, alpha, beta = testVAM.CalculateAirspeed(testState, testWind)
if not evaluateTest(cur_test, abs(beta) > 0.01):
    print(f"expected non-zero sideslip with crosswind, got beta={beta}")

cur_test = "CalculateAirspeed with gust"
testVAM = VAM.VehicleAerodynamicsModel()
testState = States.vehicleState()
testState.u = 25.0
testState.v = 0.0
testState.w = 0.0
testState.R = Rotations.euler2DCM(0, 0, 0)
testWind = States.windState(Wn=0.0, We=0.0, Wd=0.0, Wu=2.0, Wv=0.0, Ww=0.0)
Va, alpha, beta = testVAM.CalculateAirspeed(testState, testWind)
if not evaluateTest(cur_test, Va < 25.0):
    print(f"expected Va < 25.0 with positive u-gust, got Va={Va}")

print("\nBeginning testing of VAM with WindModel integration")

cur_test = "VAM getWindModel returns WindModel"
testVAM = VAM.VehicleAerodynamicsModel()
wm = testVAM.getWindModel()
if not evaluateTest(cur_test, wm is not None and hasattr(wm, 'Update')):
    print(f"expected valid WindModel instance")

cur_test = "VAM setWindModel changes wind model"
testVAM = VAM.VehicleAerodynamicsModel()
newWM = WM.WindModel(drydenParameters=VPC.DrydenLowAltitudeModerate)
testVAM.setWindModel(newWM)
wm = testVAM.getWindModel()
if not evaluateTest(cur_test, wm is newWM):
    print(f"expected wind model to be the one we set")

cur_test = "VAM Update with wind affects forces"
testVAM = VAM.VehicleAerodynamicsModel()
steadyWind = States.windState(Wn=5.0, We=0.0, Wd=0.0)
testVAM.getWindModel().setWind(steadyWind)
testControls = Inputs.controlInputs(Throttle=0.5)
testVAM.Update(testControls)
state = testVAM.getVehicleState()
if not evaluateTest(cur_test, state.Va < state.u):
    print(f"expected Va < u with headwind")
    print(f"got Va={state.Va}, u={state.u}")

total = len(passed) + len(failed)
print(f"Passed {len(passed)}/{total} tests")

if failed:
    print(f"\nFailed {len(failed)}/{total} tests:")
    [print("   " + test) for test in failed]
