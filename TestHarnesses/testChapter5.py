"""This file is a test harness for the module VehiclePerturbationModels. 

It is meant to be run from the Testharnesses directory of the repo with:

python ./TestHarnesses/testChapter5.py (from the root directory) -or-
python testChapter5.py (from inside the TestHarnesses directory)

at which point it will execute various tests on the VehiclePerturbationModels module"""

#%% Initialization of test harness and helpers:

import math

import sys
sys.path.append("..") #python is horrible, no?

import ece163.Utilities.MatrixMath as mm
import ece163.Utilities.Rotations as Rotations
import ece163.Modeling.VehicleDynamicsModel as VDM
import ece163.Controls.VehiclePerturbationModels as VPM
import ece163.Modeling.WindModel as WM
import ece163.Controls.VehicleTrim as VehicleTrim
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States

"""math.isclose doesn't work well for comparing things near 0 unless we 
use an absolute tolerance, so we make our own isclose:"""
isclose = lambda  a,b : math.isclose(a, b, abs_tol= 1e-12)

def compareVectors(a, b):
	"""A quick tool to compare two vectors"""
	el_close = [isclose(a[i][0], b[i][0]) for i in range(3)]
	return all(el_close)

#of course, you should test your testing tools too:
assert(compareVectors([[0], [0], [-1]],[[1e-13], [0], [-1+1e-9]]))
assert(not compareVectors([[0], [0], [-1]],[[1e-11], [0], [-1]]))
assert(not compareVectors([[1e8], [0], [-1]],[[1e8+1], [0], [-1]]))



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


print("Beginning testing of VehiclePerturbationModels")

print("\nTesting dThrust_dVa()")

cur_test = "dThrust_dVa is negative (thrust decreases with Va)"
try:
    result = VPM.dThrust_dVa(25.0, 0.5)
    if not evaluateTest(cur_test, result < 0.0):
        print(f"expected negative dThrust_dVa, got {result}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "dThrust_dVa changes with throttle"
try:
    result1 = VPM.dThrust_dVa(25.0, 0.3)
    result2 = VPM.dThrust_dVa(25.0, 0.7)
    if not evaluateTest(cur_test, abs(result1 - result2) > 0.01):
        print(f"expected different values at different throttles")
        print(f"got {result1} and {result2}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")


cur_test = "dThrust_dThrottle is positive (thrust increases with throttle)"
try:
    result = VPM.dThrust_dThrottle(25.0, 0.5)
    if not evaluateTest(cur_test, result > 0.0):
        print(f"expected positive dThrust_dThrottle, got {result}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "dThrust_dThrottle changes with airspeed"
try:
    result1 = VPM.dThrust_dThrottle(15.0, 0.5)
    result2 = VPM.dThrust_dThrottle(35.0, 0.5)
    if not evaluateTest(cur_test, abs(result1 - result2) > 1.0):
        print(f"expected different values at different airspeeds")
        print(f"got {result1} and {result2}")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

print("\nTesting VehicleTrim.computeTrim()")

cur_test = "computeTrim level flight"
try:
    vTrim = VehicleTrim.VehicleTrim()
    Vastar = 25.0
    Gammastar = 0.0  # Level flight
    Kappastar = 0.0  # Straight line
    check = vTrim.computeTrim(Vastar, Kappastar, Gammastar)
    if not evaluateTest(cur_test, check):
        print(f"computeTrim failed to converge for level flight")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "computeTrim climbing flight"
try:
    vTrim = VehicleTrim.VehicleTrim()
    Vastar = 25.0
    Gammastar = math.radians(6.0)  # 6 degree climb
    Kappastar = 0.0
    check = vTrim.computeTrim(Vastar, Kappastar, Gammastar)
    if not evaluateTest(cur_test, check):
        print(f"computeTrim failed to converge for climbing flight")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "computeTrim turning flight"
try:
    vTrim = VehicleTrim.VehicleTrim()
    Vastar = 25.0
    Gammastar = 0.0
    Kappastar = -1.0 / 150.0  # Turn with 150m radius
    check = vTrim.computeTrim(Vastar, Kappastar, Gammastar)
    if not evaluateTest(cur_test, check):
        print(f"computeTrim failed to converge for turning flight")
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "computeTrim sets reasonable trim state"
try:
    vTrim = VehicleTrim.VehicleTrim()
    Vastar = 25.0
    Gammastar = 0.0
    Kappastar = 0.0
    check = vTrim.computeTrim(Vastar, Kappastar, Gammastar)
    if check:
        trimState = vTrim.getTrimState()
        # Check that Va is close to desired
        if not evaluateTest(cur_test, math.isclose(trimState.Va, Vastar, abs_tol=0.5)):
            print(f"expected Va≈{Vastar}, got Va={trimState.Va}")
    else:
        evaluateTest(cur_test, False)
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "computeTrim sets reasonable trim controls"
try:
    vTrim = VehicleTrim.VehicleTrim()
    Vastar = 25.0
    Gammastar = 0.0
    Kappastar = 0.0
    check = vTrim.computeTrim(Vastar, Kappastar, Gammastar)
    if check:
        trimControls = vTrim.getTrimControls()
        if not evaluateTest(cur_test, 0.0 <= trimControls.Throttle <= 1.0):
            print(f"expected throttle in [0,1], got {trimControls.Throttle}")
    else:
        evaluateTest(cur_test, False)
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

print("\nTesting CreateTransferFunction()")


cur_test = "CreateTransferFunction sets trim values correctly"
try:
    vTrim = VehicleTrim.VehicleTrim()
    Vastar = 25.0
    Gammastar = math.radians(5.0)
    Kappastar = 0.0
    check = vTrim.computeTrim(Vastar, Kappastar, Gammastar)
    if check:
        tF = VPM.CreateTransferFunction(vTrim.getTrimState(), vTrim.getTrimControls())
        # Va_trim should be close to Vastar
        # gamma_trim should be close to Gammastar
        va_close = math.isclose(tF.Va_trim, Vastar, abs_tol=0.5)
        gamma_close = math.isclose(tF.gamma_trim, Gammastar, abs_tol=math.radians(1.0))
        if not evaluateTest(cur_test, va_close and gamma_close):
            print(f"expected Va_trim≈{Vastar}, gamma_trim≈{Gammastar}")
            print(f"got Va_trim={tF.Va_trim}, gamma_trim={tF.gamma_trim}")
    else:
        evaluateTest(cur_test, False)
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")


cur_test = "CreateTransferFunction computes V coefficients"
try:
    vTrim = VehicleTrim.VehicleTrim()
    check = vTrim.computeTrim(25.0, 0.0, 0.0)
    if check:
        tF = VPM.CreateTransferFunction(vTrim.getTrimState(), vTrim.getTrimControls())
        # a_V1 should be non-zero (drag effects)
        # a_V2 should be positive (thrust control)
        if not evaluateTest(cur_test, abs(tF.a_V1) > 1e-6 and tF.a_V2 > 0.0):
            print(f"expected a_V1 != 0 and a_V2 > 0")
            print(f"got a_V1={tF.a_V1}, a_V2={tF.a_V2}")
    else:
        evaluateTest(cur_test, False)
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception: {e}")

cur_test = "Full trim and transfer function test"
try:
    vTrim = VehicleTrim.VehicleTrim()
    
    Vastar = 25.0
    Gammastar = math.radians(6.0)
    Kappastar = -1.0 / 150.0
    
    check = vTrim.computeTrim(Vastar, Kappastar, Gammastar)
    
    if check:
        trimState = vTrim.getTrimState()
        trimControls = vTrim.getTrimControls()
        
        tF = VPM.CreateTransferFunction(trimState, trimControls)
        
        workflow_success = (
            tF is not None and
            math.isclose(tF.Va_trim, Vastar, abs_tol=1.0) and
            hasattr(tF, 'a_phi1') and
            hasattr(tF, 'a_V1')
        )
        
        if not evaluateTest(cur_test, workflow_success):
            print(f"Full workflow failed validation")
            print(f"Va_trim={tF.Va_trim}, expected≈{Vastar}")
    else:
        evaluateTest(cur_test, False)
        print(f"Trim computation failed in full workflow test")
        
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"Exception in full workflow: {e}")

total = len(passed) + len(failed)
print(f"\n---\nPassed {len(passed)}/{total} tests")
[print("   " + test) for test in passed]

if failed:
	print(f"Failed {len(failed)}/{total} tests:")
	[print("   " + test) for test in failed]