#!/usr/bin/env python

import yaml
import os
import xml.etree.ElementTree as etree
import fileinput

global txt 

stream = open("../../../../cfg/conf.d/image_recognition.yaml", "r") 
doc = yaml.load(stream)
txt = doc["plugins/image_recognition"]["labels"]
home = os.environ["HOME"]
labelPath = home + txt
f = open(labelPath,"r")
t = open("ImageRecognitionInterfaceTemplate.xml")
lines = t.readlines()
for line in f:
    i = lines.index('  </enum>\n')
    lines.insert(i,"    <item name = \"" +line.rstrip().replace(" ","") +"\">"+line.rstrip() +"</item>")

thefile = open("ImageRecognitionInterface.xml","w")
for item in lines:
    thefile.write("%s\n" % item.rstrip())

f.close()
t.close()
