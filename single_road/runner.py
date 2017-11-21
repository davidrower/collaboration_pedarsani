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

import xml.etree.ElementTree as ET

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

platooning = True

# Runs the simulation, and allows you to change traffic phase
def run(run_time):
    ## execute the TraCI control loop
    traci.init(PORT)
    programPointer = 0 # initiates at start # len(PROGRAM) - 1 # initiates at end
    step          = 0
    flow_count    = 0
    first_car     = True
    throughput_window = 1000
    prev_count    = 0
    prev_veh_id   = ' '
    leaving_times = []
    car_speeds    = []

    ## Platoons Settings
    platoon_check = 50; # how often platoons update and are checked, every X ticks
    platoon_comm = 20; # how often platoons communicate, every X ticks
    numplatoons = 0;

    ## Computing throughput 
    throughput_vec = []
    
    while traci.simulation.getMinExpectedNumber() > 0 and step <= run_time*(1/settings.step_length): 
        traci.simulationStep() # advance a simulation step

        programPointer = step*int(1/settings.step_length)

        sensor_data = traci.inductionloop.getVehicleData("sensor")

        if len(sensor_data) != 0:
            if first_car: #if its the first car, record the time that it comes in
                first_time = sensor_data[0][2]
                first_car  = False
                #print first_time, step

            veh_id = sensor_data[0][0]
            if veh_id != prev_veh_id: #if the vehicle coming in has a different id than the previous vehicle, count it towards total flow
                flow_count += 1
                car_speeds.append(traci.inductionloop.getLastStepMeanSpeed("sensor"))


            if sensor_data[0][3] != -1: #if the vehicle is leaving the sensor, record the time it left
                leaving_times.append(sensor_data[0][2])
            prev_veh_id = veh_id

        ## PLATOON CREATION
        # Creates platoons if active, one line for each intersection and road segment.
        targetTau   = 0.1;  targetMinGap = 2.0;
        caccTau     = 2.05; caccMinGap   = 4.0;

        if platooning and (step % platoon_check == 0):
            create_platoons("gneE2", "_0", 0, 1e10, caccTau, caccMinGap, 
                            targetTau, targetMinGap, programPointer)

        ## PLATOON CONTROL
        if platooning and (step % platoon_comm == 0):
            platoon_control(caccTau, caccMinGap, targetTau, targetMinGap, platoon_comm)

        ## MY THROUGHPUT ANALYSIS - DAVID 
        if step % throughput_window == 0:
            if prev_count > 0:
                curr  = flow_count - prev_count
                curr /= throughput_window * settings.step_length
                throughput_vec.append(curr)
            prev_count = flow_count

        step  += 1
   
    #print "Throughput Vector:"
    #print "Mean:", np.mean(throughput_vec)
    #print "STD:", np.std(throughput_vec)
     
    '''
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
    '''
    
    traci.close()
    sys.stdout.flush()
    return [np.mean(throughput_vec),np.var(throughput_vec),np.std(throughput_vec), np.mean(car_speeds)]

#get_options function for SUMO
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=True, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options

def changeRouteFile(alpha, path):
    tree = ET.parse(path)
    root = tree.getroot()
    for vType in root.iter('vType'):
        if 'CACC' in vType.attrib['id']:
            vType.attrib['probability'] = str(alpha)
        elif 'M' in vType.attrib['id']: 
            vType.attrib['probability'] = str(1-alpha)
    tree.write(path)
        
def C_1(alpha, h_p, h_np, d, l):
    """
    Capacity model 1
    """
    return d / (alpha * h_p + (1 - alpha) * h_np + l)

def C_2(alpha, h_p, h_np, d, l):
    """
    Capacity model 2
    """
    return d / (alpha ** 2 * h_p + (1 - alpha ** 2) * h_np + l)
    
# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if (options.nogui):
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    alphas    = list(np.linspace(0,1,10))
    run_time  = 10 * 60
    path      = "./network/single.rou.xml"
    if not os.path.exists(path): 
        sys.exit('Cant find route file for sumo!')
    output    = []
    for alpha in alphas:
        print "---------------------------------------------------------------"
        print "Running trial with alpha = %.3f" % alpha
        # this is the normal way of using traci. sumo is started as a
        # subprocess and then the python script connects and runs
        settings.init()
        changeRouteFile(alpha, path)
        sumoProcess = subprocess.Popen([sumoBinary, "-c", "./network/single.sumocfg.xml",
                                        "--step-length", str(settings.step_length), 
                                        "--remote-port", str(PORT)], stdout=sys.stdout, 
                                        stderr=sys.stderr)
        output.append(run(run_time))
        #sumoProcess.wait()
        sumoProcess.kill()
    
    print [x[0] for x in output]
    
    means  = [item[0] for item in output]
    stdevs = [item[2] for item in output]
    speeds = [item[3] for item in output]
    print(speeds)
    h_p, h_np, d, l = 2.0, 4.0, 3.3, 5.0
    alphaSample = np.linspace(0,1,5)

    plt.errorbar(alphas, means, yerr=stdevs, fmt = 'o',label = 'Measured')
    plt.plot(alphaSample, C_1(alphaSample, h_p, h_np, d, l), 'k-' , label = 'Capacity Model 1')
    plt.plot(alphaSample, C_2(alphaSample, h_p, h_np, d, l), 'k--' , label = 'Capacity Model 2')
    plt.xlabel('alpha (autonomous proportion of traffic)')
    plt.ylabel('Throughput (vehicles/timestep)')
    plt.legend(loc=1)
    plt.title('Throughput vs autonomy level of road')
    plt.show()
