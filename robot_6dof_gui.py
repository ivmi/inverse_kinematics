"""
Module for diplaying 6DOF robot.
"""

import warnings
with warnings.catch_warnings():
    warnings.simplefilter("ignore")
    from IPython.html import widgets
import time
import matplotlib.pyplot as plt
import numpy as np

#import GUI_Tools
    
import pythreejs as tjs
#from IPython.utils.traitlets import (HasTraits, List, CFloat, Bool, link, dlink)
from traitlets import (HasTraits, List, CFloat, Bool, link, dlink)

from tornado import ioloop
        
class RobotGUI(widgets.Box):
    angles = List(CFloat, [0.9, 0.2 , 0.3, 0.4, 0, 0], sync=True)
    
    def __init__(self, links=[200, 100, 100], *args, **kwargs):
        self.links = links
        
        self.create_scene()
        
        self._dom_classes = ['robot_gui']
        
        self.on_trait_change(self.update_angles, 'angles')

        kwargs["orientation"] = 'vertical'
        kwargs["children"] = [ self.renderer ]
        super(RobotGUI, self).__init__(*args, **kwargs)
        
    def update_angles(self):
        self.quaternion_from_axis_angle(self.manip, [0,1,0], self.angles[4])
        self.quaternion_from_axis_angle(self.link3, [0,0,1], self.angles[3])
        self.quaternion_from_axis_angle(self.link2, [0,0,1], self.angles[2])
        self.quaternion_from_axis_angle(self.link1, [0,0,1], self.angles[1])
        self.quaternion_from_axis_angle(self.rot0, [0,1,0], self.angles[0])

    def create_scene(self):
        #manipulator
        manip_g = tjs.Mesh(geometry=tjs.BoxGeometry(width=40.0, height=10.0, depth=40.0), material=tjs.LambertMaterial(color='red'), position=[0,5.0,0])
        self.manip = tjs.Object3d(children=[manip_g], position=[0,self.links[2],0])

        #link 3
        link3_g = tjs.Mesh(geometry=tjs.BoxGeometry(width=20.0, height=self.links[2], depth=20.0), material=tjs.LambertMaterial(color='blue'), position=[0,self.links[2]/2,0])
        link3_s = tjs.Mesh(geometry=tjs.SphereGeometry(radius=25.0), material=tjs.LambertMaterial(color='blue'), position=[0,0,0])
        self.link3 = tjs.Object3d(children=[link3_g, link3_s, self.manip], position=[0,self.links[1],0])
        
        #link 2
        link2_g = tjs.Mesh(geometry=tjs.BoxGeometry(width=20.0, height=self.links[1], depth=20.0), material=tjs.LambertMaterial(color='blue'), position=[0,self.links[1]/2,0])
        link2_s = tjs.Mesh(geometry=tjs.SphereGeometry(radius=25.0), material=tjs.LambertMaterial(color='blue'), position=[0,0,0])
        self.link2 = tjs.Object3d(children=[link2_g, link2_s, self.link3], position=[0,self.links[0],0])

        #link 1
        link1_g = tjs.Mesh(geometry=tjs.BoxGeometry(width=20.0, height=self.links[0], depth=20.0), material=tjs.LambertMaterial(color='blue'), position=[0,self.links[0]/2,0])
        link1_s = tjs.Mesh(geometry=tjs.SphereGeometry(radius=25.0), material=tjs.LambertMaterial(color='blue'), position=[0,0,0])
        self.link1 = tjs.Object3d(children=[link1_g, link1_s, self.link2], position=[0,0.0,0])

        self.rot0 = tjs.Object3d(children=[self.link1], position=[0,0.0,0])
        
        self.update_angles()
                     
        base = tjs.Mesh(geometry=tjs.BoxGeometry(width=200.0, height=5.0, depth=200.0), material=tjs.LambertMaterial(color='gray'), position=[0,0,0])

        objlist = [base, self.rot0, tjs.AmbientLight(color=0x777777)]
        scene = tjs.Scene(children=objlist)
        c = tjs.PerspectiveCamera(position=[0,300,800], up=[0,1,0], children=[tjs.DirectionalLight(color='white', 
                                position=[300,500,100], intensity=0.5)])

        
        self.oc = tjs.OrbitControls(controlling=c)
        self.renderer = tjs.Renderer(camera=c, scene = scene, controls=[self.oc], background=0xEEEEEE)

        self.oc.target = [0, 200, 0]
        c.position = [50, 400, 800]
        
    def quaternion_from_axis_angle(self, obj, axis, angle):
        halfAngle = angle / 2
        s = np.sin( halfAngle );
        obj.quaternion = [axis[0]*s, axis[1]*s, axis[2]*s, np.cos(halfAngle)]   
            
    def look_at(self, eye, target):
        # fixes camera behaviour
        self.oc.target = target
        self.oc.controlling.position = eye
        self.oc.controlling.quaternion = self.oc.target + [1] # nasty hack to force camera quat update
        

class Ticker(object):
    def __init__(self, period, cb_update=None):
        self.period = period
        if cb_update:
            self.cb_update = cb_update
        self.timer = ioloop.PeriodicCallback(self.update, period)
        self.start_time = None
        self.time_1 = None

    def start(self):
        self.timer.start()
        self.start_time = time.time()
        self.time_1 = self.start_time

    def stop(self):
        self.timer.stop()
        
    def update(self):
        t = time.time() - self.start_time
        dt = t - self.time_1
        self.time_1 = t

        if self.cb_update:
            self.cb_update(t, dt, self.period, self)
        
 
            
    
        
        