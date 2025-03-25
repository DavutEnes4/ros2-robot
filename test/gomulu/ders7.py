import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish

def on_connect(mqttc, obj, flags, reason_code, properties):
        print("reason_code: " + str(reason_code))
def on_message(mqttc, obj, msg):
        print(msg.topic + " " + str(msg.qos) + " " + str(msg.payload))
        if(int(msg.payload)>500):
                publish.single("espIsik", "0", hostname="localhost")
        else:
                publish.single("espIsik", "1", hostname="localhost")
def on_subscribe(mqttc, obj, mid, reason_code_list, properties):
        print("Subscribed: " + str(mid) + " " + str(reason_code_list))
def on_log(mqttc, obj, level, string):
        print(string)
# If you want to use a specific client id, use
# mqttc = mqtt.Client("client-id")
# but note that the client id must be unique on the broker. Leaving the client
# id parameter empty will generate a random id for you.
mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
# Uncomment to enable debug messages
# mqttc.on_log = on_log
mqttc.connect("localhost", 1883, 60)
mqttc.subscribe("espLdr")
mqttc.loop_forever()