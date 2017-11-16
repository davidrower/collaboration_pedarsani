#!/usr/bin/env python

#@file runner.py

import os
import sys
import optparse
import subprocess
import random
import pdb
import matplotlib.pyplot as plt
import math
import numpy as np
import scipy.io

import settings
settings.init()

# import python modules from $SUMO_HOME/tools directory
try:
    sys.path.append(os.path.join(os.path.dirname(os.path.realpath(
       __file__)), '..', "tools"))
    sys.path.append(os.path.join(os.environ.get("SUMO_HOME", os.path.join(
        os.path.dirname(os.path.realpath(
       __file__)), "..")), "tools"))
    from sumolib import checkBinary
except ImportError:
    sys.exit("please declare environment variable 'SUMO_HOME' as the root directory of your sumo installation (it should contain folders 'bin', 'tools' and 'docs')")

from platoon_functions_modified import *

import traci
PORT = 8873 # the port used for communicating with your sumo instance

platooning    = True
run_time      = 60*60# seconds*minutes
tau_effective = lambda x: 1.0 + (5.0 + 1.0)/x #effective tau is sumo_tau + (lengthOfCar + minGap)/avgSpeed, on this lane, avgSpeed is 30m/s and from rou.xml, lengthOfCar = 5, minGap = 0.5, sumo_tau = 1


# Runs the simulation, and allows you to change traffic phase
def run(run_time):
    ## execute the TraCI control loop
    traci.init(PORT)
    programPointer = 0 # initiates at start # len(PROGRAM) - 1 # initiates at end
    step          = 0
    flow_count    = 0
    first_car     = True
    prev_veh_id   = ' '
    leaving_times = []
    car_speeds    = []
    
    ## Platoons Settings
    platoon_check = 50; # how often platoons update and are checked, every X ticks
    platoon_comm = 20; # how often platoons communicate, every X ticks
    numplatoons = 0;


    while traci.simulation.getMinExpectedNumber() > 0 and step <= run_time*(1/settings.step_length): 
        traci.simulationStep() # advance a simulation step

        programPointer = step*int(1/settings.step_length)

        sensor_data = traci.inductionloop.getVehicleData("sensor")

        if len(sensor_data) != 0:
            if first_car: #if its the first car, record the time that it comes in
                first_time = sensor_data[0][2]
                print first_time
                first_car = False


            veh_id = sensor_data[0][0]
            if veh_id != prev_veh_id: #if the vehicle coming in has a different id than the previous vehicle, count it towards total flow
                flow_count += 1
                car_speeds.append(traci.inductionloop.getLastStepMeanSpeed("sensor"))


            if sensor_data[0][3] != -1: #if the vehicle is leaving the sensor, record the time it left
                leaving_times.append(sensor_data[0][2])
            prev_veh_id = veh_id

        ## PLATOON CREATION
        # Creates platoons if active, one line for each intersection and road segment.
        start_range = 1;   end_range    = 120;
        targetTau   = 0.1; targetMinGap = 2.0;
        caccTau     = 0.1; caccMinGap   = 2.0;

        if platooning and (step % platoon_check == 0):
            create_platoons("gneE2", "_0", 0, 1e10, caccTau, caccMinGap, targetTau, targetMinGap, programPointer)

        ## PLATOON CONTROL
        if platooning and (step % platoon_comm == 0):
            platoon_control(caccTau, caccMinGap, targetTau, targetMinGap, platoon_comm)

        step  += 1
        #print str(step)
   
    print "\n \n"
    print "-------------------------------------------------------- \n"
    print "Total number of cars that have passed: " + str(flow_count)
    tau = np.diff(leaving_times)
    print "Total throughput extrapolated to 1hr: " + str(flow_count*(3600/(run_time-first_time)))


    print "Max Theoretical throughput: " + str(3600/min(tau))
    print "Min Theoretical throughput: " + str(3600/max(tau))
    print tau

    print "Mean tau: " + str(np.mean(tau)) + "\n"
    print "Var tau: " + str(np.var(tau)) + "\n"
    print "Standard Dev tau: " + str(np.std(tau)) +"\n"

    traci.close()
    sys.stdout.flush()
    return [np.mean(tau),np.var(tau),np.std(tau)]



#get_options function for SUMO
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=True, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if (options.nogui):
        sumoBinary = checkBinary('sumo-gui')
    else:
        sumoBinary = checkBinary('sumo-gui')

    run_times = np.multiply([20],60)
    output = []
    for x in run_times:

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
        sumoProcess = subprocess.Popen([sumoBinary, "-c", "./network/single.sumocfg.xml","--step-length", str(settings.step_length), "--remote-port", str(PORT)], stdout=sys.stdout, stderr=sys.stderr)


        output.append(run(x))

        sumoProcess.wait()

    print output

    means = [item[0] for item in output]
    stdevs = [item[2] for item in output]
    plt.errorbar(np.divide(run_times,60), means, yerr=stdevs, fmt = 'o',label = 'Measured')
    plt.plot(np.divide(run_times,60),np.multiply(np.ones(len(run_times)),(tau_effective(30))),'k-' , label = 'Theoretical')
    plt.xlabel('Simulation Time (min)')
    plt.ylabel('Tau (s)')
    plt.legend()
    plt.title('Theoretical Tau vs Measured Tau in simulations of varying length')
    plt.axis([run_times[0]/60-10, run_times[-1]/60+10, 1, 1.5])
    plt.show()

