import paho.mqtt.client as mqtt
import sys
import json

sys.path.append('/home/tristan/projects/ctp_py/src')

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    #client.subscribe("$SYS/#")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg, future, asyncloop):
    print(msg.topic+" "+f""""{str(msg.payload.decode("utf-8") )}""")

    if msg.topic.startswith('bmResult'):
        print(f"""Decoding bmResult {str(msg.payload.decode("utf-8"))}""")
        t = json.loads(str(msg.payload.decode("utf-8")))
        print(f"""Done decoding bmResult.""")
        dct = {}
        if t[0] == 0.0:
            dct['pathlength'] = 5000000.0
        else:
            dct['pathlength'] = t[0]
        dct['computationtime'] = t[1]
        dct['failures'] = t[2]
        print(f"returning dct: {dct}")
        asyncloop.call_soon_threadsafe(cb, future.set_result(dct))
    else:
        if msg.topic.startswith('bmInitResponse'):
            print("aaa")
            if str(msg.payload.decode("utf-8")).startswith('Success'):
                print("mqtt Client: success, returning 1")
                asyncloop.call_soon_threadsafe(cb, future.set_result(1))
            else:
                print("received failure message. Future remains open.")
                asyncloop.call_soon_threadsafe(cb, future.set_result(0))


def cb(*args):
    print(f"callback {args}")

# Blocking call that processes network traffic, dispatches callbacks and
# handles reconnecting.
# Other loop*() functions are available that give a threaded interface and a
# manual interface.

def send_message(client, s, topic):
    print(f"Sending message. Topic: {topic}, payload: {s}, client: {client} ")
    client.publish(topic, payload=s, qos=0, retain=False)
