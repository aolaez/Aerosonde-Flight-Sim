import math
import sys
sys.path.append("..")

import matplotlib.pyplot as plt
import ece163.Sensors.SensorsModel as SM
import ece163.Modeling.VehicleAerodynamicsModel as VAM
import ece163.Containers.Inputs as Inputs
import ece163.Containers.States as States
import ece163.Constants.VehiclePhysicalConstants as VPC

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

print("\nTesting GaussMarkov basic functionality...")

cur_test = "GaussMarkov init"
try:
    gm = SM.GaussMarkov(dT=0.01, tau=100.0, eta=0.1)
    evaluateTest(cur_test, gm.v == 0.0 and gm.tau == 100.0)
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"   Exception: {e}")

cur_test = "GaussMarkov reset"
try:
    gm = SM.GaussMarkov()
    gm.v = 5.0
    gm.reset()
    evaluateTest(cur_test, gm.v == 0.0)
except Exception as e:
    evaluateTest(cur_test, False)

cur_test = "GaussMarkov update accumulates"
try:
    gm = SM.GaussMarkov(dT=0.01, tau=1e6, eta=0.0)
    v1 = gm.update(vnoise=1.0)
    v2 = gm.update(vnoise=1.0)
    evaluateTest(cur_test, v2 > v1)
except Exception as e:
    evaluateTest(cur_test, False)

print("\nTesting GaussMarkovXYZ")

cur_test = "GaussMarkovXYZ init with shared parameters"
try:
    gmxyz = SM.GaussMarkovXYZ(tauX=100.0, etaX=0.1)
    evaluateTest(cur_test, gmxyz.tauX == 100.0 and gmxyz.tauY == 100.0)
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"   Exception: {e}")

cur_test = "GaussMarkovXYZ reset"
try:
    gmxyz = SM.GaussMarkovXYZ()
    gmxyz.vX = gmxyz.vY = gmxyz.vZ = 5.0
    gmxyz.reset()
    evaluateTest(cur_test, gmxyz.vX == 0.0 and gmxyz.vY == 0.0)
except Exception as e:
    evaluateTest(cur_test, False)


print("\nTesting SensorsModel initialization...")

cur_test = "SensorsModel init"
try:
    sensors = SM.SensorsModel()
    evaluateTest(cur_test, sensors.updateTicks == 0)
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"   Exception: {e}")

cur_test = "SensorsModel initializeBiases returns vehicleSensors"
try:
    sensors = SM.SensorsModel()
    evaluateTest(cur_test, sensors.sensorBiases is not None)
except Exception as e:
    evaluateTest(cur_test, False)

print("\nTesting sensor measurement functions...")

cur_test = "updateGyrosTrue returns 3 values"
try:
    sensors = SM.SensorsModel()
    state = States.vehicleState()
    state.p = 0.1
    state.q = 0.2
    state.r = 0.3
    gyro = sensors.updateGyrosTrue(state)
    evaluateTest(cur_test, len(gyro) == 3 and math.isclose(gyro[0], 0.1))
except Exception as e:
    evaluateTest(cur_test, False)
    print(f"   Exception: {e}")

cur_test = "updatePressureSensorsTrue returns baro and pitot"
try:
    sensors = SM.SensorsModel()
    state = States.vehicleState()
    state.pd = -100.0
    state.Va = 25.0
    pressure = sensors.updatePressureSensorsTrue(state)
    evaluateTest(cur_test, len(pressure) == 2 and pressure[1] > 0)
except Exception as e:
    evaluateTest(cur_test, False)



# Test 1: Gauss-Markov Drift Visualization
print("\nTest 1: Gauss-Markov drift process")
try:
    gm = SM.GaussMarkov(dT=0.01, tau=100.0, eta=0.5)
    
    time_data = []
    drift_data = []
    
    for i in range(1000):
        time_data.append(i * 0.01)
        drift_data.append(gm.update())
    
    plt.figure(figsize=(10, 4))
    plt.plot(time_data, drift_data, 'b-', linewidth=1)
    plt.xlabel('Time (s)')
    plt.ylabel('Drift Value')
    plt.title('Gauss-Markov Drift Process (tau=100s, eta=0.5)')
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('test_gaussmarkov_drift.png', dpi=150)
except Exception as e:
    print(f"   Failed: {e}")

# Test 2: Sensor Noise Visualization
print("\nTest 2: Sensor true vs noisy comparison")
try:
    vam = VAM.VehicleAerodynamicsModel()
    sensors = SM.SensorsModel(aeroModel=vam)
    
    # Set up level flight
    state = States.vehicleState()
    state.u = 25.0
    state.pd = -100.0
    state.Va = 25.0
    vam.setVehicleState(state)
    
    inputs = Inputs.controlInputs(Throttle=0.5, Elevator=0.0, Aileron=0.0, Rudder=0.0)
    
    time_data = []
    gyro_true_x = []
    gyro_noisy_x = []
    accel_true_z = []
    accel_noisy_z = []
    
    for i in range(500):
        time_data.append(i * VPC.dT)
        
        vam.Update(inputs)
        sensors.update()
        
        sensorsTrue = sensors.getSensorsTrue()
        sensorsNoisy = sensors.getSensorsNoisy()
        
        gyro_true_x.append(sensorsTrue.gyro_x)
        gyro_noisy_x.append(sensorsNoisy.gyro_x)
        accel_true_z.append(sensorsTrue.accel_z)
        accel_noisy_z.append(sensorsNoisy.accel_z)
    
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    ax1.plot(time_data, gyro_true_x, 'b-', label='True', linewidth=2, alpha=0.7)
    ax1.plot(time_data, gyro_noisy_x, 'r-', label='Noisy', linewidth=1, alpha=0.8)
    ax1.set_ylabel('Gyro X (rad/s)')
    ax1.set_title('Gyroscope: True vs Noisy')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2.plot(time_data, accel_true_z, 'b-', label='True', linewidth=2, alpha=0.7)
    ax2.plot(time_data, accel_noisy_z, 'r-', label='Noisy', linewidth=1, alpha=0.8)
    ax2.set_ylabel('Accel Z (m/s²)')
    ax2.set_xlabel('Time (s)')
    ax2.set_title('Accelerometer: True vs Noisy')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('test_sensor_noise.png', dpi=150)
    print("   Saved test_sensor_noise.png")
except Exception as e:
    print(f"   Failed: {e}")

print("\nTest 3: GPS zero-order hold")
try:
    vam = VAM.VehicleAerodynamicsModel()
    sensors = SM.SensorsModel(aeroModel=vam, gpsUpdateHz=1.0)
    
    state = States.vehicleState()
    state.u = 25.0
    state.pd = -100.0
    state.Va = 25.0
    vam.setVehicleState(state)
    
    inputs = Inputs.controlInputs(Throttle=0.5, Elevator=-0.05, Aileron=0.0, Rudder=0.0)
    
    time_data = []
    gps_alt_true = []
    gps_alt_noisy = []
    
    for i in range(300):
        time_data.append(i * VPC.dT)
        
        vam.Update(inputs)
        sensors.update()
        
        gps_alt_true.append(sensors.getSensorsTrue().gps_alt)
        gps_alt_noisy.append(sensors.getSensorsNoisy().gps_alt)
    
    plt.figure(figsize=(10, 5))
    plt.plot(time_data, gps_alt_true, 'b-', label='True', linewidth=2, alpha=0.7)
    plt.plot(time_data, gps_alt_noisy, 'r-', label='Noisy (ZOH)', linewidth=2, drawstyle='steps-post')
    plt.xlabel('Time (s)')
    plt.ylabel('GPS Altitude (m)')
    plt.title('GPS Zero-Order Hold Effect')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('test_gps_zoh.png', dpi=150)
except Exception as e:
    print(f"   Failed: {e}")


total = len(passed) + len(failed)
print(f"Passed {len(passed)}/{total} tests")

if failed:
    print(f"\nFailed tests:")
    for test in failed:
        print(f"   • {test}")
else:
    print("\nAll tests passed!")

plt.show()