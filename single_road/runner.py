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
    flow_count1   = 0
    flow_count2   = 0
    first_car     = True
    capacity_window = 100
    prev_veh_id1  = ' '
    prev_veh_id2  = ' '
    leaving_times = []
    car_speeds1   = []
    car_speeds2   = []

    ## Platoons Settings
    platoon_check = 50; # how often platoons update and are checked, every X ticks
    platoon_comm  = 10; # how often platoons communicate, every X ticks
    numplatoons   = 0;

    ## Computing capacity 
    capacity_vec = []
    
    while traci.simulation.getMinExpectedNumber() > 0 and step <= run_time*(1/settings.step_length): 
        traci.simulationStep() # advance a simulation step

        programPointer = step*int(1/settings.step_length)

        sensor1_data = traci.inductionloop.getVehicleData("sensor1")
        sensor2_data = traci.inductionloop.getVehicleData("sensor2")

        if len(sensor1_data) != 0:
            if first_car: #if its the first car, record the time that it comes in
                first_time = sensor1_data[0][2]
                first_car  = False
                #print first_time, step
            
            veh_id1 = sensor1_data[0][0]
            if veh_id1 != prev_veh_id1: #if the vehicle coming in has a different id than the previous vehicle, count it towards total flow
                flow_count1 += 1
                car_speeds1.append(traci.inductionloop.getLastStepMeanSpeed("sensor1"))

            if sensor1_data[0][3] != -1: #if the vehicle is leaving the sensor, record the time it left
                leaving_times.append(sensor1_data[0][2])

            prev_veh_id1 = veh_id1

        if len(sensor2_data) != 0:
            veh_id2 = sensor2_data[0][0]
            if veh_id2 != prev_veh_id2: #if the vehicle coming in has a different id than the previous vehicle, count it towards total flow
                flow_count2 += 1
                car_speeds2.append(traci.inductionloop.getLastStepMeanSpeed("sensor2"))
            prev_veh_id2 = veh_id2

        ## PLATOON CREATION
        # Creates platoons if active, one line for each intersection and road segment.
        targetTau   = 2.0; targetMinGap = 4.0;
        caccTau     = 3.0; caccMinGap   = 4.0;

        if platooning and (step % platoon_check == 0):
            create_platoons("gneE2", "_0", 0, 1e10, caccTau, caccMinGap, 
                            targetTau, targetMinGap, programPointer)

        ## PLATOON CONTROL
        if platooning and (step % platoon_comm == 0):
            platoon_control(caccTau, caccMinGap, targetTau, targetMinGap, platoon_comm)

        ## MY CAPACITY ANALYSIS - DAVID 
        if step % capacity_window == 0:
            if flow_count2 > 0:
                curr  = flow_count1 - flow_count2
                capacity_vec.append(curr)

        step  += 1
   
    '''
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
    
    print "\n \n"
    print "Capacity Vector:"
    print "Mean:", np.mean(capacity_vec)
    print "STD:", np.std(capacity_vec)
    #print "Mean, std speed: %f, %f" % (np.mean(car_speeds), np.std(car_speeds)) + "\n"
    print "Speeds Vector Stats 1", np.mean(car_speeds1), np.std(car_speeds1), "\n"
    print "Speeds Vector Stats 2", np.mean(car_speeds2), np.std(car_speeds2), "\n"
    traci.close()
    sys.stdout.flush()
    return [np.mean(capacity_vec),np.var(capacity_vec),np.std(capacity_vec)]

#get_options function for SUMO
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--gui", action="store_true",
                         default=False, help="run the commandline version of sumo")
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
    if (options.gui):
        sumoBinary = checkBinary('sumo-gui')
    else:
        sumoBinary = checkBinary('sumo')

    alphas    = list(np.linspace(0.0, 1, 10))
    #alphas    = list(np.arange(0.95, 1.0, 0.01))
    run_time  = 10 * 60
    path      = "./network/single.rou.xml"
    if not os.path.exists(path): 
        sys.exit('Cant find route file for sumo!')
    output    = []
    for alpha in alphas:
        print "\n"
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
    
    print "\n"
    print [x[0] for x in output]
    
    means  = [item[0] for item in output]
    stdevs = [item[1] for item in output]
    h_np = 3.0 * 10.0 + 4.  # tau_platoon * speed limit of road + minGap
    h_p  = 2.0 * 10.0 + 4.  # tau_manual  * speed limit of road + minGap
    d, l = 3500., 5.0   # distance between sensors, length of cars
    alphaSample = np.linspace(0,1,20)
    print alphaSample, C_2(alphaSample, h_p, h_np, d, l)
    plt.errorbar(alphas, means, yerr=stdevs, fmt = 'o',label = 'Measured')
    plt.plot(alphaSample, C_1(alphaSample, h_p, h_np, d, l), 'k-' ,  label = 'Capacity Model 1')
    plt.plot(alphaSample, C_2(alphaSample, h_p, h_np, d, l), 'k--' , label = 'Capacity Model 2')
    plt.xlabel('Alpha (Autonomous proportion of traffic)')
    plt.ylabel('Capacity')
    plt.title('Capacity vs Autonomy Level of Road')
    plt.show()
    plt.savefig("capacity.pdf")
