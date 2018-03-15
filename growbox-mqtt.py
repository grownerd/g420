#!/usr/bin/env python3

import paho.mqtt.client as paho
from datetime import datetime
import time
import serial
import sys
import os
import time
import threading
import json
import getopt

import rrdtool

rrd_path = '/ssd/g420/rrd'

tty_dev = '/dev/ttyAMA0'
tty_baudrate = 115200

#mqtt_server = 'wsan1'
#ca_certs = '/root/ca.crt'
#
#mqtt_user = 'growbox'
#mqtt_pass = 'zlxkwoxx003Ss&22Ab:DDTto'

mqtt_server = 'localhost'
mqtt_topic = 'G420'

ser_data = ''

#{"info": "Adding 12.21 ml of pHDown to 11.00 liters in Reservoir to lower pH by 5.05.", "time": "14.09.2017 12:18:35"}
#{"event": "PH Down Pump turned on for 12213 ms", "time": "14.09.2017 12:18:35"}
#{"event": "Finished adjusting pH", "time": "PH Down Pump"}
#{"info": "Adding 12.28 ml of pHDown to 11.00 liters in Reservoir to lower pH by 5.08.", "time": "14.09.2017 12:18:52"}
#{"event": "PH Down Pump turned on for 12283 ms", "time": "14.09.2017 12:18:52"}
#{"name":"datetime","content":{"date":"14.09.2017", "time":"12:18:54.003960", "unix":1505391534}}
#{"name":"Sensors","content":[
#        {"name":"Flowering Chamber Temperature", "value":20.98, "unit":"°C"},
#        {"name":"Flowering Chamber rel. Humidity", "value":64.14, "unit":"%"},
#        {"name":"Flowering Chamber Air Pressure", "value":997.03, "unit":"mBar"},
#        {"name":"Water Tank Temperature","value":18.75, "unit":"°C"},
#        {"name":"Reservoir Temperature","value":16.38, "unit":"°C"},
#        {"name": "Reservoir pH", "value":10.72},
#        {"name": "Reservoir EC", "value":0.32}
#]}
#{"name":"Main Light","content":{
#        "on_time":"10:00",
#        "off_time":"22:00",
#        "state":"On"
#}}
#{"name":"Main Exhaust","content":{
#        "max_temp":26.00,
#        "max_humi":60.00,
#        "min_temp":20.00,
#        "min_humi":40.00,
#        "state":"Off"
#}}
#{"name":"Coolant Control","content":{
#        "max_temp":16.49,
#        "min_temp":16.41,
#        "state":"Off"
#}}
#{"name":"Errors","content":[
#        {"name":"DS18B20 Water Tank Temperature", "count":17},
#        {"name":"DS18B20 Reservoir Temperature", "count":8},
#        {"name":"I2C Resets", "count":"24968"},
#        {"name":"Restarted by Watchdog", "value":"no"}
#]}
#{"name":"GPIO Output Channels","content":[
#        {"name":"Fill Pump", "state":0, "run for ms":0},
#        {"name":"Drain Pump", "state":0, "run for ms":0},
#        {"name":"Coolant Pump", "state":0, "run for ms":-1},
#        {"name":"Sewage Pump", "state":0, "run for ms":0},
#        {"name":"Dehumidifier Pump", "state":0, "run for ms":0},
#        {"name":"PH Down Pump", "state":1, "run for ms":11376},
#        {"name":"Flora Micro Pump", "state":0, "run for ms":0},
#        {"name":"Flora Gro Pump", "state":0, "run for ms":0},
#        {"name":"Flora Bloom Pump", "state":0, "run for ms":0},
#        {"name":"Deep Red Leds", "state":0, "run for ms":0},
#        {"name":"Nutrient Stirrers", "state":0, "run for ms":0}
#]}
#{"name":"Relays","content":[
#        {"relay_id":0, "state":1},
#        {"relay_id":1, "state":0},
#        {"relay_id":2, "state":0}
#]}
#{"name":"Capsense Channels","content":[
#        {"id":0, "capacitance":10592673},
#        {"id":1, "capacitance":10592673}
#]}
#{"name": "PWM Input", "id":0, "frequency":   0.00}
#{"name":"IRQ Inputs","content":[
#        {"name":"Blue Button", "state":0},
#        {"name":"", "state":0},
#        {"name":"Reservoir max. level", "state":0},
#        {"name":"", "state":0},
#        {"name":"Reservoir min. level", "state":0},
#        {"name":"Reservoir Alarm level", "state":1},
#        {"name":"unused 6", "state":0},
#        {"name":"Dehumidifier tank full", "state":0},
#        {"name":"Dehumidifier tank empty", "state":0},
#        {"name":"", "state":0},
#        {"name":"Sewage tank full", "state":1},
#        {"name":"Sewage tank empty", "state":1},
#        {"name":"unused 12", "state":0},
#        {"name":"unused 13", "state":0},
#        {"name":"", "state":0},
#        {"name":"Water tank empty", "state":0}
#]}
#{"name":"Misc. Settings","content":{
#        "res_liters_min":10.00,
#        "res_liters_max":12.00,
#        "res_liters_alarm":14.00,
#        "nutrient_factor":0.50,
#        "ec_k":2.8800,
#        "ec_temp_coef":0.0190,
#        "ec_r1_ohms":470,
#        "ec_ra_ohms":25,
#        "ph_cal401":3195,
#        "ph_cal686":2805,
#        "res_settling_time":5,
#        "sewage_pump_pause_s":7200,
#        "sewage_pump_run_s":60,
#        "fill_to_alarm_level":0}
#}
#{"name":"Global State","content":{
#        "sewage_pump_blocked":0,
#        "sewage_tank_full":0,
#        "sewage_tank_empty":1,
#        "drain_cycle_active":0,
#        "adjusting_ph":1,
#        "adding_nutrients":0,
#        "stirring_nutrients":0,
#        "reservoir_alarm":0,
#        "reservoir_max":1,
#        "reservoir_min":0,
#        "water_tank_empty":0,
#        "reservoir_state":"NORMAL_IDLE"}
#}
#{"event": "Finished adjusting pH", "time": "PH Down Pump"}
#{"info": "Adding 12.18 ml of pHDown to 11.00 liters in Reservoir to lower pH by 5.03.", "time": "14.09.2017 12:19:10"}
#{"event": "PH Down Pump turned on for 12179 ms", "time": "14.09.2017 12:19:10"}


def rrd_fetch(ds, res, start, end):
  fname = "{}/g420_{}.rrd".format(rrd_path, ds)
  topic = "{}/arch/data/{}".format(mqtt_topic, ds)
  data = rrdtool.fetch(fname, "AVERAGE", "-r", res, "-s", start, "-e", end);
  #print(data)
  client.publish(topic, json.dumps(data))

def process_json(obj):
  if (obj["name"]):
    #print(obj["name"])
    if (obj["content"]):
      if (obj["name"] == "Sensors"):
        process_sensors(obj)
      elif (obj["name"] == "Errors"):
        process_errorcounter(obj)

def process_sensors(obj):
  ds_type = "GAUGE"
  for kv in obj["content"]:
    #print(kv)
    if (kv["name"]):
      name = kv["name"].replace(" ", "_")
      rrd_filename = "{}/g420_{}.rrd".format(rrd_path, name)
      if (os.path.isfile(rrd_filename)):
        val = "N:{}".format(kv["value"])
        rrdtool.update(rrd_filename, val)

      else:
        create_rrd(rrd_filename, name, ds_type)

def process_errorcounter(obj):
  ds_type = "COUNTER"

def create_rrd(rrd_filename, ds_name, ds_type):
  print("creating rrd")
  ds_line = "DS:{}:{}:300:U:U".format(ds_name, ds_type)
  try:
    rrdtool.create(rrd_filename,
    "--start", "now", "--step", "5s",
    ds_line,
    "RRA:AVERAGE:0.5:5s:1d",
    "RRA:AVERAGE:0.5:1m:7d",
    "RRA:AVERAGE:0.5:1h:3M")
  except:
    print("error")

def sw(serial, text):
  serial.write(bytes(text, 'UTF-8'))
  serial.flush()
  #time.sleep(0.2)
  #print(serial.read(serial.inWaiting()))

def get_all_loop():
  threading.Timer(5.0, get_all_loop).start()
  sw(ser, "get all\n")
 
def on_connect(client, userdata, flags, rc):
  print("CONNACK received with code %d." % (rc))
  client.subscribe("G420/cmd/#")
  client.subscribe("G420/arch/#")

def on_message(client, userdata, msg):
  print(msg.topic+" "+str(msg.payload))
  if (msg.topic == "G420/cmd"):
    sw(ser, "{}\n".format(msg.payload.decode('UTF-8')))
  elif (msg.topic == "G420/arch/fetch"):
    pstr = msg.payload.decode('UTF-8')
    ds, res, start, end = pstr.split()
    rrd_fetch(ds, res, start, end)
  
def set_datetime():
  d = datetime.utcnow() # UTC because fuck DST!
  timestring = d.strftime('set rtc %d.%m.%y.%w;%H:%M:%S\n')
  sw(ser, timestring)
  ser.flush()
  
def get_datetime():
  sw(ser, "get rtc\n")
  print(ser.read(500))
  

def serial_to_mqtt():
  #ser_line = ser.readline()

  global ts_now
  global ts_last
  global ser_data

  #ser_data = ser.read(4096)
  bytes_available = ser.inWaiting()

  in_json = False;
  ser_byte = b''


  if (bytes_available > 0):


    # discard data if older than a second
    ts_now = time.time()
    if ts_now > (ts_last + 1):
      ser_data = ''

    ts_last = ts_now

    try:
      #ser_data += (ser.read(bytes_available).decode('UTF-8'))
      ser_data += (ser.readline().decode('UTF-8'))
      #print(ser_data)
      try:
        json_obj = json.loads(ser_data)
        process_json(json_obj)
        client.publish(mqtt_topic, json.dumps(json_obj))
        print(json.dumps(json_obj))
        ser_data = ''
      except:
        foo=1
        #print("string not ready")
        
    except:
      print("unreadable character received")


def usage():
  print("buh!")


if __name__ == "__main__":
  ser = serial.Serial(tty_dev, tty_baudrate, timeout=1, parity=serial.PARITY_NONE, rtscts=0)
  try:
    opts, args = getopt.getopt(sys.argv[1:], "htd", ["help", "time", "defaults"])
  except getopt.GetoptError as err:
    # print help information and exit:
    print(err) # will print something like "option -a not recognized"
    usage()
    sys.exit(2)

  opt_datetime = False

  for o, a in opts:
    if o in ("-t", "--time"):
      opt_datetime = True
    elif o in ("-h", "--help"):
      usage()
      sys.exit()
    else:
      assert False, "unhandled option"

   
  client = paho.Client()
  client.on_connect = on_connect
  client.on_message = on_message
  #client.tls_set(ca_certs)
  #client.username_pw_set(mqtt_user, password=mqtt_pass)
  client.connect(mqtt_server, 1883)
  client.loop_start()

  
  global_i=0
  ts_now=time.time()
  ts_last=time.time()

  if (opt_datetime):
    set_datetime()
  get_all_loop()

  while(True):
    serial_to_mqtt()

  client.loop_stop()
  ser.close()
