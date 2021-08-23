"""

Extension of SIPP to multi-robot scenarios for the Multi-Robot Warehouse Environment (RWARE) 

Author: Tim Tsz-Kit Lau (@timlautk)

Based on the implementation of Ashwin Bose (@atb033) for CBS, and Filippos Christianos and Lukas Schafer for RWARE


See the article: 10.1109/ICRA.2011.5980306

"""

import argparse
import yaml
from math import fabs
from graph_generation import SippGraph, State
from sipp import SippPlanner

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map and dynamic obstacles")
    parser.add_argument("output", help="output file with the schedule")
    parser.add_argument("env_name", help="environment name from RWARE")
    
    args = parser.parse_args()
    
    # Read Map
    with open(args.map, 'r') as map_file:
        try:
            map = yaml.load(map_file, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    # Output file
    with open(args.output, 'r') as output_yaml:
        try:
            output = yaml.load(output_yaml, Loader=yaml.FullLoader)
        except yaml.YAMLError as exc:
            print(exc)

    for i in range(len(map["agents"])):
        sipp_planner = SippPlanner(map,i)
    
        if sipp_planner.compute_plan():
            plan = sipp_planner.get_plan()
            output["schedule"].update(plan)
            map["dynamic_obstacles"].update(plan)

            with open(args.output, 'w') as output_yaml:
                yaml.safe_dump(output, output_yaml)  
        else: 
            print("Plan not found")


if __name__ == "__main__":
    main()
