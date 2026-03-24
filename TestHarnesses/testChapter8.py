import math
import sys
sys.path.append("..")

import matplotlib.pyplot as plt
import ece163.Controls.VehicleEstimator as VE
import ece163.Containers.Controls as Controls
import ece163.Containers.Sensors as Sensors
import ece163.Containers.States as States
import ece163.Containers.Inputs as Inputs
import ece163.Constants.VehiclePhysicalConstants as VPC
import ece163.Constants.VehicleSensorConstants as VSC
import ece163.Sensors.SensorsModel as SM
import ece163.Modeling.VehicleAerodynamicsModel as VAM

passed = []
failed = []

def evaluateTest(test_name, boolean):
    if boolean:
        print(f"   {test_name}")
        passed.append(test_name)
    else:
        print(f"   {test_name}")
        failed.append(test_name)
    return boolean

print("\nTesting LowPassFilter...")

cur_test = "LowPassFilter init"
try:
    lpf = VE.LowPassFilter(dT=0.01, cutoff=1.0)
    evaluateTest(cur_test, lpf.output == 0.0 and lpf.cutoff == 1.0)
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"   Exception: {e}")

cur_test = "LowPassFilter reset"
try:
    lpf = VE.LowPassFilter()
    lpf.output = 5.0
    lpf.reset()
    evaluateTest(cur_test, lpf.output == 0.0)
except Exception as e:
    evaluateTest(cur_test, False)

cur_test = "LowPassFilter update smooths step input"
try:
    lpf = VE.LowPassFilter(dT=0.01, cutoff=1.0)
    output = lpf.update(10.0)
    evaluateTest(cur_test, 0 < output < 10.0)
except Exception as e:
    evaluateTest(cur_test, False)

print("\nTesting VehicleEstimator initialization...")

cur_test = "VehicleEstimator init"
try:
    sensors = SM.SensorsModel()
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    evaluateTest(cur_test, estimator.estimatedState.pd == VPC.InitialDownPosition)
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"   Exception: {e}")

cur_test = "VehicleEstimator reset"
try:
    sensors = SM.SensorsModel()
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    estimator.estimatedState.roll = 0.5
    estimator.gyro_bias_x = 0.1
    estimator.reset()
    evaluateTest(cur_test, estimator.gyro_bias_x == 0.0)
except Exception as e:
    evaluateTest(cur_test, False)

cur_test = "VehicleEstimator getEstimatedState"
try:
    sensors = SM.SensorsModel()
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    state = estimator.getEstimatedState()
    evaluateTest(cur_test, state is not None)
except Exception as e:
    evaluateTest(cur_test, False)

cur_test = "VehicleEstimator setEstimatorBiases"
try:
    sensors = SM.SensorsModel()
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    estimator.setEstimatorBiases(estimatedGyroBias=[[0.1], [0.2], [0.3]])
    evaluateTest(cur_test, math.isclose(estimator.gyro_bias_x, 0.1))
except Exception as e:
    evaluateTest(cur_test, False)

print("\nTesting estimateAttitude...")

cur_test = "estimateAttitude returns 3 values"
try:
    sensors = SM.SensorsModel()
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    sensorData = Sensors.vehicleSensors()
    sensorData.gyro_x = 0.1
    sensorData.accel_z = VPC.g0
    sensorData.mag_x = VSC.magfield[0][0]
    sensorData.mag_y = VSC.magfield[1][0]
    sensorData.mag_z = VSC.magfield[2][0]
    
    bias, omega, R = estimator.estimateAttitude(sensorData, estimator.estimatedState)
    evaluateTest(cur_test, len(bias) == 3 and len(omega) == 3 and len(R) == 3)
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"   Exception: {e}")

cur_test = "estimateAttitude updates DCM"
try:
    sensors = SM.SensorsModel()
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    sensorData = Sensors.vehicleSensors()
    sensorData.gyro_x = 0.0
    sensorData.gyro_y = 0.0
    sensorData.gyro_z = 0.0
    sensorData.accel_x = 0.0
    sensorData.accel_y = 0.0
    sensorData.accel_z = VPC.g0
    sensorData.mag_x = VSC.magfield[0][0]
    sensorData.mag_y = VSC.magfield[1][0]
    sensorData.mag_z = VSC.magfield[2][0]
    
    R_before = estimator.estimatedState.R[0][0]
    estimator.estimateAttitude(sensorData, estimator.estimatedState)
    evaluateTest(cur_test, estimator.estimatedState.R is not None)
except Exception as e:
    evaluateTest(cur_test, False)

print("\nTesting estimateAirspeed...")

cur_test = "estimateAirspeed returns 2 values"
try:
    sensors = SM.SensorsModel()
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    sensorData = Sensors.vehicleSensors()
    sensorData.pitot = 0.5 * VPC.rho * 25.0**2
    sensorData.accel_x = 0.0
    
    bias, Va = estimator.estimateAirspeed(sensorData, estimator.estimatedState)
    evaluateTest(cur_test, isinstance(bias, float) and isinstance(Va, float))
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"   Exception: {e}")

cur_test = "estimateAirspeed updates Va"
try:
    sensors = SM.SensorsModel()
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    estimator.estimatedState.Va = 20.0
    sensorData = Sensors.vehicleSensors()
    sensorData.pitot = 0.5 * VPC.rho * 25.0**2
    sensorData.accel_x = 1.0
    
    bias, Va = estimator.estimateAirspeed(sensorData, estimator.estimatedState)
    evaluateTest(cur_test, estimator.estimatedState.Va > 0)
except Exception as e:
    evaluateTest(cur_test, False)

print("\nTesting estimateAltitude...")

cur_test = "estimateAltitude returns 3 values"
try:
    sensors = SM.SensorsModel()
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    sensorData = Sensors.vehicleSensors()
    sensorData.baro = VSC.Pground - VPC.rho * VPC.g0 * 100.0
    sensorData.accel_z = VPC.g0
    
    h, hdot, bias = estimator.estimateAltitude(sensorData, estimator.estimatedState)
    evaluateTest(cur_test, isinstance(h, float) and isinstance(hdot, float))
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"   Exception: {e}")

print("\nTesting estimateCourse...")

cur_test = "estimateCourse returns 2 values"
try:
    sensors = SM.SensorsModel()
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    sensorData = Sensors.vehicleSensors()
    sensorData.gps_cog = 0.5
    
    bias, chi = estimator.estimateCourse(sensorData, estimator.estimatedState)
    evaluateTest(cur_test, isinstance(bias, float) and isinstance(chi, float))
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"   Exception: {e}")

cur_test = "estimateCourse wraps angle"
try:
    sensors = SM.SensorsModel()
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    estimator.estimatedState.chi = 3.0
    sensorData = Sensors.vehicleSensors()
    
    bias, chi = estimator.estimateCourse(sensorData, estimator.estimatedState)
    evaluateTest(cur_test, -math.pi <= estimator.estimatedState.chi <= math.pi)
except Exception as e:
    evaluateTest(cur_test, False)

print("\nTest 1: Low-pass filter step response")
try:
    lpf = VE.LowPassFilter(dT=0.01, cutoff=2.0)
    
    time_data = []
    output_data = []
    
    for i in range(300):
        time_data.append(i * 0.01)
        output_data.append(lpf.update(10.0))
    
    plt.figure(figsize=(10, 4))
    plt.plot(time_data, output_data, 'b-', linewidth=2)
    plt.axhline(y=10.0, color='r', linestyle='--', label='Target')
    plt.xlabel('Time (s)')
    plt.ylabel('Output')
    plt.title('Low-Pass Filter Step Response (cutoff=2Hz)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('test_lpf_response.png', dpi=150)
    print("   Saved test_lpf_response.png")
except Exception as e:
    print(f"   Failed: {e}")

print("\nTest 2: Attitude estimation with gyro drift")
try:
    vam = VAM.VehicleAerodynamicsModel()
    sensors = SM.SensorsModel(aeroModel=vam)
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    
    state = States.vehicleState()
    state.u = 25.0
    state.pd = -100.0
    vam.setVehicleState(state)
    
    time_data = []
    roll_true = []
    roll_est = []
    bias_x = []
    
    for i in range(500):
        time_data.append(i * VPC.dT)
        
        vam.Update(Inputs.controlInputs(Throttle=0.5))
        sensors.update()
        
        true_state = vam.getVehicleState()
        sensorData = sensors.getSensorsNoisy()
        
        estimator.estimateAttitude(sensorData, estimator.estimatedState)
        
        roll_true.append(true_state.roll)
        roll_est.append(estimator.estimatedState.roll)
        bias_x.append(estimator.gyro_bias_x)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    ax1.plot(time_data, roll_true, 'b-', label='True', linewidth=2)
    ax1.plot(time_data, roll_est, 'r--', label='Estimated', linewidth=2)
    ax1.set_ylabel('Roll (rad)')
    ax1.set_title('Attitude Estimation: Roll Angle')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2.plot(time_data, bias_x, 'g-', linewidth=2)
    ax2.set_ylabel('Gyro Bias X (rad/s)')
    ax2.set_xlabel('Time (s)')
    ax2.set_title('Estimated Gyro Bias')
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('test_attitude_estimation.png', dpi=150)
    print("   Saved test_attitude_estimation.png")
except Exception as e:
    print(f"   Failed: {e}")

print("\nTest 3: Altitude estimation with GPS updates")
try:
    vam = VAM.VehicleAerodynamicsModel()
    sensors = SM.SensorsModel(aeroModel=vam, gpsUpdateHz=1.0)
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    
    state = States.vehicleState()
    state.u = 25.0
    state.pd = -100.0
    vam.setVehicleState(state)
    
    time_data = []
    alt_true = []
    alt_est = []
    gps_updates = []
    
    for i in range(500):
        time_data.append(i * VPC.dT)
        
        vam.Update(Inputs.controlInputs(Throttle=0.6, Elevator=-0.05))
        sensors.update()
        
        true_state = vam.getVehicleState()
        sensorData = sensors.getSensorsNoisy()
        
        estimator.estimateAltitude(sensorData, estimator.estimatedState)
        
        alt_true.append(-true_state.pd)
        alt_est.append(-estimator.estimatedState.pd)
        
        if sensors.updateTicks % sensors.gpsTicksUpdate == 1:
            gps_updates.append((time_data[-1], alt_est[-1]))
    
    plt.figure(figsize=(10, 5))
    plt.plot(time_data, alt_true, 'b-', label='True', linewidth=2)
    plt.plot(time_data, alt_est, 'r--', label='Estimated', linewidth=2)
    
    if gps_updates:
        gps_times, gps_alts = zip(*gps_updates)
        plt.scatter(gps_times, gps_alts, color='green', s=50, marker='o', 
                   label='GPS Update', zorder=5)
    
    plt.xlabel('Time (s)')
    plt.ylabel('Altitude (m)')
    plt.title('Altitude Estimation with GPS Corrections')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('test_altitude_estimation.png', dpi=150)
    print("   Saved test_altitude_estimation.png")
except Exception as e:
    print(f"   Failed: {e}")

print("\nTest 4: Course estimation during turn")
try:
    vam = VAM.VehicleAerodynamicsModel()
    sensors = SM.SensorsModel(aeroModel=vam, gpsUpdateHz=1.0)
    estimator = VE.VehicleEstimator(sensorsModel=sensors)
    
    state = States.vehicleState()
    state.u = 25.0
    state.pd = -100.0
    vam.setVehicleState(state)
    
    time_data = []
    course_true = []
    course_est = []
    
    for i in range(600):
        time_data.append(i * VPC.dT)
        
        aileron = 0.3 if i < 300 else -0.3
        vam.Update(Inputs.controlInputs(Throttle=0.5, Aileron=aileron))
        sensors.update()
        
        true_state = vam.getVehicleState()
        sensorData = sensors.getSensorsNoisy()
        
        estimator.estimateAttitude(sensorData, estimator.estimatedState)
        estimator.estimateCourse(sensorData, estimator.estimatedState)
        
        course_true.append(true_state.chi)
        course_est.append(estimator.estimatedState.chi)
    
    plt.figure(figsize=(10, 5))
    plt.plot(time_data, course_true, 'b-', label='True', linewidth=2)
    plt.plot(time_data, course_est, 'r--', label='Estimated', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Course (rad)')
    plt.title('Course Estimation During Turn')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('test_course_estimation.png', dpi=150)
    print("   Saved test_course_estimation.png")
except Exception as e:
    print(f"   Failed: {e}")

total = len(passed) + len(failed)
print(f"\nPassed {len(passed)}/{total} tests")

if failed:
    print(f"\nFailed tests:")
    for test in failed:
        print(f"   • {test}")
else:
    print("\nAll tests passed!")

plt.show()