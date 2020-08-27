#!/usr/bin/python
import asyncio
import concurrent
import sys
from functools import partial, partialmethod

sys.path.append('scripts')
sys.path.append('/home/tristan/projects/hypermapper/scripts')
sys.path.append('/home/tristan/projects/ctp_py/src')

from org.combinators.ctp.py.optimization import mqttClient

import hypermapper
import uuid
import paho.mqtt.client as mqtt

use_simplification = True

print("1")
parameters_file = "/home/tristan/projects/ctp_py/src/org/combinators/ctp/py/optimization/ctp_scenario.json"
client = mqtt.Client()
client.on_connect = mqttClient.on_connect
print("trying to connect")
client.connect("koopa.cs.tu-dortmund.de", 1883)
print("Done Connecting")

planner_string_list = ["sbmp_planner_PRM", "sbmp_planner_PRMStar", "sbmp_planner_LazyPRM",
                       "sbmp_planner_LazyPRMStar", "sbmp_planner_SST", "sbmp_planner_RRT", "sbmp_planner_RRTStar",
                       "sbmp_planner_LBTRRT", "sbmp_planner_TRRT", "sbmp_planner_LazyRRT",
                       "sbmp_planner_RRTConnect", "sbmp_planner_EST", "sbmp_planner_SBL", "sbmp_planner_LBKPIECE1",
                       "sbmp_planner_KPIECE1", "sbmp_planner_BKPIECE1",
                       "sbmp_planner_STRIDE", "sbmp_planner_PDST", "sbmp_planner_FMT", "sbmp_planner_BFMT",
                       "sbmp_planner_RRTsharp", "sbmp_planner_RRTXstatic", "sbmp_planner_InformedRRTstar",
                       "sbmp_planner_BITstar"]
sampler_string_list = ["sbmp_uniform_valid_state_sampler", "sbmp_obstacle_valid_state_sampler",
                       "sbmp_gaussian_valid_state_sampler", "sbmp_max_clearance_valid_state_sampler",
                       "sbmp_uniform_space_sampler", "sbmp_gaussian_space_sampler"]
motion_validator_string_list = ["sbmp_fcl_motion_validator", "sbmp_discrete_motion_validator"]
optimization_objective_list = ["sbmp_path_length", "sbmp_opt_integral"]
cost_list = ["sbmp_default_cost_state", "sbmp_cost_state_change_weighted"]
simplification_list = ["sbmp_use_simplification", "sbmp_no_simplification"]
dimensionality_list = ["dimensionality_two_d_t", "dimensionality_three_d_t", "dimensionality_n_d_t"]


def get_planner_string(i):
    return planner_string_list[i]


def get_sampler_string(i):
    return sampler_string_list[i]


def get_state_validator_string(i):
    return "any_sbmp_state_validator_type"


def get_motion_validator_string(i):
    return motion_validator_string_list[i]


def get_optimization_objective_string(i):
    return optimization_objective_list[i]


def get_cost_string(i):
    return cost_list[i]


def get_cost_string(i):
    return simplification_list[i]


def get_dimensionality_string(i):
    return dimensionality_list[i]


# check async possible?
def ctp_function(X, problem_instance):
    planner = X['planner']
    sampler = X['sampler']
    motion_validator = X['motionValidator']
    # simplification_time = X['simplification_time']
    planning_time = X['planning_time']

    new_uuid = uuid.uuid4()
    json_string = f"""{{
"planner" : "{planner_string_list[planner]}",
"sampler" : "{get_sampler_string(sampler)}",
"stateValidator" : "sbmp_fcl_validator",
"motionValidator" : "{get_motion_validator_string(motion_validator)}",
"costs" : "not_specified",
"optObjective" : "not_specified",
"simplification" : "sbmp_use_simplification",
"sceneInput" : "scene_input_data_file",
"dimensionality" : "dimensionality_three_d_t",
"id" : "{new_uuid}",
"configurableAlg" : true,
"withStates" : false
}}
"""

    mqttClient.send_message(client, json_string, "bmInitRequest")
    loop = asyncio.get_event_loop()
    f = loop.create_future()
    # loop.create_task(mqttRequest(client, json_string, fooo[0]))
    print("Setting new on_message")
    client.on_message = partial(mqttClient.on_message, future=f, asyncloop=loop)
    print(f"Subscribing to topic bmInitResponse.{new_uuid}")
    client.subscribe(f"bmInitResponse.{new_uuid}", 2)
    client.loop_start()
    print("Waiting for init request to complete.")
    loop.run_until_complete(f)
    print("Future completed. Stopping mqtt loop")
    client.loop_stop()

    if f.result() == 1:  # Success
        bmStartResponseTopic = f"bmStartResponse.{new_uuid}"
        bmGenericInputResponseTopic = f"bmGenericInputResponse.{new_uuid}"

        client.subscribe(f"bmResult.{new_uuid}", 2)
        print(f"Subscribing to {bmStartResponseTopic}")
        print(f"Subscribing to {bmGenericInputResponseTopic}")
        client.subscribe(bmStartResponseTopic, 2)
        client.subscribe(bmGenericInputResponseTopic, 2)

        loop4 = asyncio.get_event_loop()
        fGenericInputResponse = loop4.create_future()
        client.on_message = partial(mqttClient.on_message, future=fGenericInputResponse, asyncloop=loop4)
        client.loop_start()
        mqttClient.send_message(client,
                                f"""["{problem_instance}.cfg", "--computationtime={planning_time} --simplificationtime=2.0"]""",
                                # f"""{problem_instance}.cfg""",
                                f"bmGenericInput.{new_uuid}")
        try:
            print(f"Waiting for bmGenericInputResponse {new_uuid} ...")
            loop4.run_until_complete(asyncio.wait_for(fGenericInputResponse, 10.0))
        except Exception as e:
            client.loop_stop()
            print(f"TimeoutError")
            output = {}
            output['pathlength'] = 5000000.0
            output['computationtime'] = 5000000.0
            output['failures'] = 5000000.0
            print(f"Aborting with output: {output}")
            return output


        loop3 = asyncio.get_event_loop()
        fStartResponse = loop3.create_future()
        client.on_message = partial(mqttClient.on_message, future=fStartResponse, asyncloop=loop3)
        mqttClient.send_message(client,
                                """{"iterations" : 10,"async" : true,"maxTime" : 10,"maxMem" : 1024}""",
                                f"bmStartRequest.{new_uuid}")

        try:
            loop3.run_until_complete(asyncio.wait_for(fStartResponse, 10.0))
        except Exception as e:
            client.loop_stop()
            print(f"Timeout for StartResponse")
            output = {}
            output['pathlength'] = 5000000.0
            output['computationtime'] = 5000000.0
            output['failures'] = 5000000.0
            print(f"Aborting with output: {output}")
            return output

        print("Messages sent. Waiting for result.")
        loop2 = asyncio.get_event_loop()
        # client.loop_stop()
        # client.loop_start()
        f2 = loop2.create_future()
        client.on_message = partial(mqttClient.on_message, future=f2, asyncloop=loop2)
        loop2.run_until_complete(f2)
        client.loop_stop()

        print("Received result.")
        output = f2.result()
    else:
        output = {}
        output['pathlength'] = 5000000.0
        output['computationtime'] = 5000000.0
        output['failures'] = 5000000.0

    print(f"Output: {output}")
    return output




def printIndices():
    print(f"{[i for i, x in enumerate(sampler_string_list)]}")
    print(f"{[i for i, x in enumerate(motion_validator_string_list)]}")
    print(f"{[i for i, x in enumerate(optimization_objective_list)]}")
    print(f"{[i for i, x in enumerate(cost_list)]}")
    print(f"{[i for i, x in enumerate(simplification_list)]}")
    print(f"{[i for i, x in enumerate(dimensionality_list)]}")


if __name__ == "__main__":
    print("Starting Main")
    import argparse
    parser = argparse.ArgumentParser(description='ctp optimization')
    parser.add_argument('--problem', default="Abstract", help='name of the problem instance', type=str)

    args = parser.parse_args()
    ctp_fun = partial(ctp_function, problem_instance=args.problem)
    hypermapper.optimize(parameters_file, ctp_fun)
    print("End of ctp optimization.")
