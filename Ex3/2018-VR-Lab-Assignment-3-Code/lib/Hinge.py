#!/usr/bin/python

# import guacamole libraries
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed

import lib.Utilities

# import python libraries
import math


class Accumulator(avango.script.Script):

    ## declaration of fields        
    sf_rot_input = avango.SFFloat()    
    
    sf_mat = avango.gua.SFMatrix4()
    sf_mat.value = avango.gua.make_identity_mat() # initialize field with identity matrix


    ## constructor
    def __init__(self):
        self.super(Accumulator).__init__() # call base-class constructor


    ## callback functions
    def evaluate(self):
        # perform update when fields change (with dependency evaluation)
        
        # ToDo: accumulate rotation input here        
        self.sf_mat.value = avango.gua.make_rot_mat(self.sf_rot_input.value, 0,1,0) * self.sf_mat.value



class Constraint(avango.script.Script):

    ## declaration of fields    
    sf_mat = avango.gua.SFMatrix4()
    sf_mat.value = avango.gua.make_identity_mat() # initialize field with identity matrix

    ## constructor
    def __init__(self):
        self.super(Constraint).__init__() # call base-class constructor

        ## variables
        self.min_angle = -180.0 # in degrees
        self.max_angle = 180.0 # in degrees


    def set_min_max_values(self, MIN, MAX):
        self.min_angle = MIN
        self.max_angle = MAX
    

    ## callback functions
    def evaluate(self):
        # perform update when fields change (with dependency evaluation)
      
        # check and apply rotation constraints
        _head, _pitch, _roll = lib.Utilities.get_euler_angles(self.sf_mat.value)

        #check constraints with head variable
        if _head > self.max_angle:
            self.sf_mat.value = avango.gua.make_rot_mat(self.max_angle, 0,1,0)
        elif _head < self.min_angle:
            self.sf_mat.value = avango.gua.make_rot_mat(self.min_angle, 0,1,0)



class Hinge:

    ## class variables

    # Number of Hinge instances that have already been created.
    number_of_instances = 0   

    # constructor
    def __init__(self,
        PARENT_NODE = None,
        DIAMETER = 0.1, # in meter
        HEIGHT = 0.1, # in meter
        ROT_OFFSET_MAT = avango.gua.make_identity_mat(), # the rotation offset relative to the parent coordinate system
        SF_ROT_INPUT = None,
        MIN = 0,
        MAX = 90
        ):

        ## get unique id for this instance
        self.id = Hinge.number_of_instances
        Hinge.number_of_instances += 1


        ## scenegraph nodes

        self.hinge_rot_offset_node = avango.gua.nodes.TransformNode(Name = "hinge{0}_rot_offset_node".format(str(self.id)))
        self.hinge_rot_offset_node.Transform.value = ROT_OFFSET_MAT # apply initial rotation offset 
        PARENT_NODE.Children.value.append(self.hinge_rot_offset_node)

        self.hinge_node = avango.gua.nodes.TransformNode(Name = "hinge{0}_node".format(str(self.id)))
        self.hinge_rot_offset_node.Children.value.append(self.hinge_node)

        _loader = avango.gua.nodes.TriMeshLoader() # get trimesh loader to load external tri-meshes

        self.hinge_geometry = _loader.create_geometry_from_file("hinge{0}_geometry".format(str(self.id)), "data/objects/cylinder.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.hinge_geometry.Transform.value = avango.gua.make_scale_mat(DIAMETER, HEIGHT, DIAMETER)
        self.hinge_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(0.8,0.0,0.0,1.0))
        self.hinge_node.Children.value.append(self.hinge_geometry)


        ## sub-classes
        self.acc = Accumulator()
        self.acc.sf_mat.value = self.hinge_node.Transform.value # consider (potential) rotation offset 

        # init Constraint here
        self.constraint = Constraint()
        self.constraint.set_min_max_values(MIN, MAX)

        # field connections here
        self.acc.sf_rot_input.connect_from(SF_ROT_INPUT)
        self.constraint.sf_mat.connect_from(self.acc.sf_mat)

        #Connect the accumulated matrix to the hinge matrix
        self.hinge_node.Transform.connect_from(self.constraint.sf_mat)

        #handle cyclic dependency
        self.acc.sf_mat.connect_weak_from(self.constraint.sf_mat)



       
        
