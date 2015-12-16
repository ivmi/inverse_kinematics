"""
Module for diplaying 6DOF robot.
"""

import warnings
#with warnings.catch_warnings():
#    warnings.simplefilter("ignore")
#    from IPython.html import widgets
from ipywidgets import widgets
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
        self.manip.children[0].position = (self.angles[5]*20, 20, 0)
        self.manip.children[1].position = (-self.angles[5]*20, 20, 0)
        self.quaternion_from_axis_angle(self.manip, [0,1,0], self.angles[4])
        self.quaternion_from_axis_angle(self.link3, [0,0,1], self.angles[3])
        self.quaternion_from_axis_angle(self.link2, [0,0,1], self.angles[2])
        self.quaternion_from_axis_angle(self.link1, [0,0,1], self.angles[1])
        self.quaternion_from_axis_angle(self.rot0, [0,1,0], self.angles[0])

    def create_scene(self):
        #manipulator
        manip_g1 = tjs.Mesh(geometry=tjs.BoxGeometry(width=5.0, height=40.0, depth=40.0), material=tjs.LambertMaterial(color='red'), position=[50,20.0,0])
        manip_g2 = tjs.Mesh(geometry=tjs.BoxGeometry(width=5.0, height=40.0, depth=40.0), material=tjs.LambertMaterial(color='red'), position=[-50,20.0,0])
        self.manip = tjs.Object3d(children=[manip_g1, manip_g2], position=[0,self.links[2],0])

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
        c = tjs.PerspectiveCamera(position=[0,300,800], up=[0,1,0], far = 30000.0, children=[tjs.DirectionalLight(color='white', 
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

        
class FKGUI(widgets.FlexBox):
    def __init__(self, robot_gui, *args, **kwargs):
        # create sliders for angle selection 
        self.robot_gui = robot_gui
        
        spacer = widgets.HTML(value="&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;")
        self.fs0 = widgets.FloatSlider(
            value=0.0,
            min=0,
            max=np.pi,
            step=0.1,
            description='a0',
            orientation='vertical',
            width='148px',
        )
        
        self.fs1 = widgets.FloatSlider(
            value=0.0,
            min=-np.pi,
            max=np.pi,
            step=0.1,
            description='a1',
            orientation='vertical',
        )

        self.fs2 = widgets.FloatSlider(
            value=0.0,
            min=-np.pi,
            max=np.pi,
            step=0.1,
            description='a2',
            orientation='vertical',
        )

        self.fs3 = widgets.FloatSlider(
            value=0.0,
            min=-np.pi,
            max=np.pi,
            step=0.1,
            description='a3',
            orientation='vertical',
        )
        
        self.fs4 = widgets.FloatSlider(
            value=0.0,
            min=-np.pi,
            max=np.pi,
            step=0.1,
            description='a4',
            orientation='vertical',
        )

        self.fs5 = widgets.FloatSlider(
            value=0.0,
            min=0,
            max=1,
            step=0.1,
            description='a5',
            orientation='vertical',
        )
        
        self.fs0.on_trait_change(self.on_val_change, 'value')
        self.fs1.on_trait_change(self.on_val_change, 'value')
        self.fs2.on_trait_change(self.on_val_change, 'value')
        self.fs3.on_trait_change(self.on_val_change, 'value')
        self.fs4.on_trait_change(self.on_val_change, 'value')
        self.fs5.on_trait_change(self.on_val_change, 'value')

        
        kwargs["children"]=[self.fs0, spacer, self.fs1, spacer, self.fs2, spacer, self.fs3, spacer, self.fs4, spacer, self.fs5]
        kwargs["orientation"] = 'horizontal'
        super(FKGUI, self).__init__(*args, **kwargs)
    
    def on_val_change(self, name, value):
        self.robot_gui.angles = (self.fs0.value, self.fs1.value, self.fs2.value, self.fs3.value, self.fs4.value, self.fs5.value)

def inv_kin(p, ga, l=[100,100,100]):
    w = np.array([0]*4,dtype=float) #horizontal coordinate
    z = np.array([0]*4,dtype=float) #vertical coordinate

    gripper_angle=ga

    w[3] = np.sqrt ( np.square ( p[2] ) + np.square ( p[0] )) 
    z[3] = p[1]

    w[2] = w[3] - l[2] * np.cos(gripper_angle)
    z[2] = z[3] - l[2] * np.sin(gripper_angle)

    l12 = np.sqrt ( np.square ( w[2] ) + np.square ( z[2] ))
    a12 = np.arctan2(z[2], w[2])

    a = [0]*4
    a[0] = np.arctan2(p[0],p[2])
    a[1] = np.arccos ( ( np.square( l[0] ) + np.square ( l12 ) - np.square ( l[1])) / (2 * l[0] * l12 )) + a12
    w[1] = l[0] * np.cos(a[1])
    z[1] = l[0] * np.sin(a[1])
    a[2] = np.arctan2 ( ( z[2] - z[1] ) , ( w[2] - w[1] ) ) - a[1]
    a[3] = gripper_angle - a[1] - a[2]
  
    a[1]=a[1]-np.pi/2
    return a        
        
class IKGUI(widgets.FlexBox):
    def __init__(self, robot_gui, *args, **kwargs):
        # create sliders for angle selection 
        self.robot_gui = robot_gui
        
        spacer = widgets.HTML(value="&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;")
        self.fs0 = widgets.FloatSlider(
            value=0.0,
            min=0,
            max=400,
            step=0.1,
            description='x',
            orientation='vertical',
            width='148px',
        )
        
        self.fs1 = widgets.FloatSlider(
            value=0.0,
            min=0,
            max=400,
            step=0.1,
            description='y',
            orientation='vertical',
        )

        self.fs2 = widgets.FloatSlider(
            value=0.0,
            min=0,
            max=400,
            step=0.1,
            description='z',
            orientation='vertical',
        )

        self.fs3 = widgets.FloatSlider(
            value=0.0,
            min=-np.pi,
            max=np.pi,
            step=0.1,
            description='gripper angle',
            orientation='vertical',
        )
        
        self.fs0.on_trait_change(self.on_val_change, 'value')
        self.fs1.on_trait_change(self.on_val_change, 'value')
        self.fs2.on_trait_change(self.on_val_change, 'value')
        self.fs3.on_trait_change(self.on_val_change, 'value')
        
        kwargs["children"]=[self.fs0, spacer, self.fs1, spacer, self.fs2, spacer, self.fs3]
        kwargs["orientation"] = 'horizontal'
        super(IKGUI, self).__init__(*args, **kwargs)
    
    def on_val_change(self, name, value):
        ang = inv_kin([self.fs0.value, self.fs1.value, self.fs2.value], self.fs3.value, self.robot_gui.links)
        self.robot_gui.angles = ang + [0,0]
        #self.robot_gui.angles = (self.fs0.value, self.fs1.value, self.fs2.value, self.fs3.value, self.fs4.value, self.fs5.value)        
        
        
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
        
 
            
    
        
        