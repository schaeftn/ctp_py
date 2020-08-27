import asyncio

import paho.mqtt.client as mqtt
import sys
import json
from io import StringIO

sys.path.append('/home/tristan/projects/ctp_py/src')

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {str(rc)}")

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    #client.subscribe("$SYS/#")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg, future, asyncloop):
    print(msg.topic + " " + f""""{str(msg.payload.decode("utf-8"))}""")

    if msg.topic.startswith('bmResult'):
        msg_payload = str(msg.payload.decode("utf-8"))
        print(f"""Decoding bmResult {msg_payload}""")
        t = json.load(StringIO(msg_payload))
        print(f"""Done decoding bmResult.""")
        dct = {}
        print(f"""Decoded dict: {t}""")
        if t['pathLengths']:
            dct['pathlength'] = sum(t['pathLengths']) / len(t['pathLengths'])
        else:
            dct['pathlength'] = 5000000.0
        if t['computationTimes']:
            dct['computationtime'] = sum(t['computationTimes']) / len(t['computationTimes'])
        else:
            dct['computationtime'] = 5000000.0

        dct['failures'] = float(t['failures'])
        print(f"returning dct: {dct}")
        asyncloop.call_soon_threadsafe(cb, future.set_result(dct))
    elif msg.topic.startswith('bmInitResponse'):
        if str(msg.payload.decode("utf-8")).startswith('Success'):
            print("mqtt Client: success, returning 1")
            asyncloop.call_soon_threadsafe(cb, future.set_result(1))
        else:
            print("Benchmark init: received failure message. Discarding run.")
            asyncloop.call_soon_threadsafe(cb, future.set_result(0))
    elif msg.topic.startswith('bmStartResponse'):
        print("Received startResponse.")
        asyncloop.call_soon_threadsafe(cb, future.set_result(1))
    elif msg.topic.startswith('bmGenericInputResponse'):
        print("Received inputResponse.")
        asyncloop.call_soon_threadsafe(cb, future.set_result(1))


def cb(*args):
    ()

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.

def send_message(client, s, topic):
    print(f"Sending message. Topic: {topic}, payload: {s}, client: {client} ")
    client.publish(topic, payload=s, qos=0, retain=True)
