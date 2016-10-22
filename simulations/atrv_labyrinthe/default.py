#! /usr/bin/env morseexec

from morse.builder import *
import random
from math import floor
from lxml import etree

# -------- Chargement des paramètres du système à partir de modele.xml ---------
tree = etree.parse("/home/mix/workspace/src/open_rm/simulations/atrv_labyrinthe/modele.xml")
for args in tree.xpath("/launch/arg"):
    if args.get("name") == "Z_cam":
        Z_cam = float(args.get("value"))
    if args.get("name") == "Z_las":
        Z_las = float(args.get("value"))
    if args.get("name") == "Z_rob":
        Z_rob = float(args.get("value"))


# -------- Initialisation aléatoire du robot dans le labyrinthe --------
# Taille du labyrinthe :
nRow = 15
nCol = 15
tCell = 4 # largeur d'une cellule
eMur = 0.5 # Epaisseur des murs
# Cellule de départ du robot
row = random.randint(-floor(nRow/2.),floor(nRow/2.))
col = random.randint(-floor(nCol/2.),floor(nCol/2.))
# Initialisation aléatoire dans la cellule choisie
poseX = row*(tCell+2*eMur) + random.uniform(-tCell/2.+0.5,tCell/2.-0.5)
poseY = col*(tCell+2*eMur) + random.uniform(-tCell/2.+0.5,tCell/2.-0.5)
poseTheta = random.uniform(0,6.28)

# Déclaration et positionnement du robot
robot = ATRV()
robot.translate(poseX, poseY, Z_rob)
robot.rotate(0.0, 0.0, poseTheta)

# On ajoute une commande (v,omega) au robot
motion = MotionVW()
robot.append(motion)

# On ajoute une interface clavier au robot
keyboard = Keyboard()
robot.append(keyboard)
keyboard.properties(ControlType = 'Position')

# On ajoute un odomètre qui exprime la pose du
# robot dans le repère monde
pose = Pose()
robot.append(pose)

# On ajoute un Hokuyo au robot
# 1 point tous les 0.5°, portée de 30m
# champ de vue à 360°
hokuyo=Hokuyo()
hokuyo.properties(resolution = 0.5)
hokuyo.properties(scan_window = 360.)
hokuyo.properties(laser_range = 30.0)
hokuyo.name = 'laser/scan'
hokuyo.translate(x = 0.0,y = 0.0, z = Z_las)
hokuyo.rotate(x = 0.0,y = 0.0, z = 0.0)
robot.append(hokuyo)

# On ajoute une caméra au robot
# Résolution : 500*500
# Positionnement : Bird Eye View
camera=VideoCamera()
camera.properties(cam_height=500,cam_width=500,cam_focal=15)
camera.translate(x = 0.0,y = 0.0, z = Z_cam)
camera.rotate(x = 0.0,y =-1.57, z = 0.0)
robot.append(camera)

# On interface la simulation avec ROS
robot.add_default_interface('ros')

# On charge le modèle de l'environnement
env = Environment('/home/mix/workspace/src/open_rm/simulations/maps/lab_15x15.blend', fastmode = False)

# On place une caméra dans blender pour la visualisation du scénario
env.set_camera_location([.0, .0, 95.0])
env.set_camera_rotation([0, 0, -1.57])
