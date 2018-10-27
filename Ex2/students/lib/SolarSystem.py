#!/usr/bin/python3

### import guacamole libraries ###
import avango
import avango.gua
from avango.script import field_has_changed
import avango.daemon

### import framework libraries ###
from lib.SolarObject import SolarObject
import lib.SolarParameters as SP


class SolarSystem(avango.script.Script):

    ## input fields
    sf_key0 = avango.SFFloat()
    sf_key1 = avango.SFFloat()
  
    ## output_fields
    sf_time_scale_factor = avango.SFFloat()
    sf_time_scale_factor.value = 1.0


    ### constructor
    def __init__(self):
        self.super(SolarSystem).__init__() # call base-class constructor

        ## init device sensor
        self.keyboard_sensor = avango.daemon.nodes.DeviceSensor(DeviceService = avango.daemon.DeviceService())
        self.keyboard_sensor.Station.value = "gua-device-keyboard"

        self.sf_key0.connect_from(self.keyboard_sensor.Button12)
        self.sf_key1.connect_from(self.keyboard_sensor.Button13)


    def my_constructor(self, PARENT_NODE):

        # init Sun
        self.sun = SolarObject(
            NAME = "sun",
            TEXTURE_PATH = SP.SUN_TEXTURE,
            PARENT_NODE = PARENT_NODE,
            SF_TIME_SCALE = self.sf_time_scale_factor,
            DIAMETER = SP.SUN_DIAMETER,
            ORBIT_RADIUS = 0.0,
            ORBIT_INCLINATION = 0.0,
            ORBIT_DURATION = 0.0,
            ROTATION_INCLINATION = 0.0,
            ROTATION_DURATION = 0.0,
            )
                                                                            
        # init lightsource (only for sun)
        self.sun_light = avango.gua.nodes.LightNode(Name = "sun_light", Type = avango.gua.LightType.POINT)
        self.sun_light.Color.value = avango.gua.Color(1.0, 1.0, 1.0)
        self.sun_light.Brightness.value = 25.0
        self.sun_light.Falloff.value = 0.2
        self.sun_light.EnableShadows.value = True
        self.sun_light.ShadowMapSize.value = 2048
        self.sun_light.Transform.value = avango.gua.make_scale_mat(50.0) # light volume defined by scale
        self.sun_light.ShadowNearClippingInSunDirection.value = 0.1 / 50.0
        self.sun.get_orbit_node().Children.value.append(self.sun_light)


        ## TODO: init planets and moons below here
        # init Earth
        self.earth = SolarObject(
            NAME = "earth",
            TEXTURE_PATH = SP.EARTH_TEXTURE,
            PARENT_NODE = self.sun.get_orbit_node(),
            SF_TIME_SCALE = self.sf_time_scale_factor,
            DIAMETER = SP.EARTH_DIAMETER,
            ORBIT_RADIUS = SP.EARTH_ORBIT_RADIUS,
            ORBIT_INCLINATION = SP.EARTH_ORBIT_INCLINATION,
            ORBIT_DURATION = SP.EARTH_ORBIT_DURATION,
            ROTATION_INCLINATION = SP.EARTH_ROTATION_INCLINATION,
            ROTATION_DURATION = SP.EARTH_ROTATION_DURATION
            )

        self.earth_moon = SolarObject(
            NAME = "earth_moon",
            TEXTURE_PATH = SP.EARTH_MOON_TEXTURE,
            PARENT_NODE = self.earth.get_orbit_node(),
            SF_TIME_SCALE = self.sf_time_scale_factor,
            DIAMETER = SP.EARTH_MOON_DIAMETER,
            ORBIT_RADIUS = SP.EARTH_MOON_ORBIT_RADIUS,
            ORBIT_INCLINATION = SP.EARTH_MOON_ORBIT_INCLINATION,
            ORBIT_DURATION = SP.EARTH_MOON_ORBIT_DURATION,
            ROTATION_INCLINATION = SP.EARTH_MOON_ROTATION_INCLINATION,
            ROTATION_DURATION = SP.EARTH_MOON_ROTATION_DURATION
            )



    ### callback functions ###
    @field_has_changed(sf_key0)
    def sf_key0_changed(self):
        if self.sf_key0.value == True: # button pressed
            _new_factor = self.sf_time_scale_factor.value * 1.5 # increase factor to 150% 

            self.set_time_scale_factor(_new_factor)
      
    @field_has_changed(sf_key1)
    def sf_key1_changed(self): 
        if self.sf_key1.value == True: # button pressed
            _new_factor = self.sf_time_scale_factor.value * 0.5 # decrease factor to 50%

            self.set_time_scale_factor(_new_factor)


    ### functions ###
    def set_time_scale_factor(self, FLOAT): 
        self.sf_time_scale_factor.value = min(10000.0, max(1.0, FLOAT)) # clamp value to reasonable intervall
        
