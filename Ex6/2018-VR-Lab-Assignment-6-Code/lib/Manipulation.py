#!/usr/bin/python

### import guacamole libraries ###
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed
import avango.daemon

### import python libraries ###
import math
import sys
import time


class ManipulationManager(avango.script.Script):

    ## input fields
    sf_toggle_button = avango.SFBool()

    ## constructor
    def __init__(self):
        self.super(ManipulationManager).__init__()    
    
    
    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        HEAD_NODE = None,
        POINTER_INPUT = None,
        TARGET_LIST = []
        ):

        
        ### variables ###
        self.active_manipulation_technique = None
        self.active_manipulation_technique_index = None
        self.sf_toggle_button.connect_from(POINTER_INPUT.sf_toggle_button)

    
        ## init manipulation techniques
        self.ray = Ray()
        self.ray.my_constructor(SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT)


        self.depthRay = DepthRay()
        self.depthRay.my_constructor(SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT)


        self.goGo = GoGo()
        self.goGo.my_constructor(SCENEGRAPH, NAVIGATION_NODE, HEAD_NODE, POINTER_INPUT, TARGET_LIST)


        self.virtualHand = VirtualHand()
        self.virtualHand.my_constructor(SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT,TARGET_LIST)

    
        ### set initial states ###
        #change method
        self.set_manipulation_technique(1) # switch to virtual-ray manipulation technique



    ### functions ###
    def set_manipulation_technique(self, INT):
        # possibly disable prior technique
        if self.active_manipulation_technique is not None:
            self.active_manipulation_technique.enable(False)
    
        # enable new technique
        if INT == 0: # ray
            print("switch to Ray technique")
            self.active_manipulation_technique = self.ray

        elif INT == 1: # depth ray
            print("switch to Depth-Ray technique")
            self.active_manipulation_technique = self.depthRay

        elif INT == 2: # go-go
            print("switch to Go-Go technique")
            self.active_manipulation_technique = self.goGo

        elif INT == 3: # HOMER
            print("switch to Virtual-Hand (PRISM) technique")
            self.active_manipulation_technique = self.virtualHand

        self.active_manipulation_technique_index = INT
        self.active_manipulation_technique.enable(True)


    ### callback functions ###
    @field_has_changed(sf_toggle_button)
    def sf_toggle_button_changed(self):
        if self.sf_toggle_button.value == True: # key is pressed
            next_index = (self.active_manipulation_technique_index + 1) % 4
            self.set_manipulation_technique(next_index) # switch to Ray manipulation technique



class ManipulationTechnique(avango.script.Script):

    ## input fields
    sf_button = avango.SFBool()

    ## constructor
    def __init__(self):
        self.super(ManipulationTechnique).__init__()
               

    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        POINTER_INPUT = None,
        ):


        ### external references ###
        self.SCENEGRAPH = SCENEGRAPH
        self.POINTER_INPUT = POINTER_INPUT
            
            
        ### variables ###
        self.enable_flag = False
        
        self.selected_node = None
        self.dragged_node = None
        self.dragging_offset_mat = avango.gua.make_identity_mat()
                
        self.mf_pick_result = []
        self.pick_result = None # chosen pick result
        self.ray_selected_objects = None
        self.white_list = []   
        self.black_list = ["invisible"]

        #self.pick_options = avango.gua.PickingOptions.PICK_ONLY_FIRST_OBJECT \
        #                    | avango.gua.PickingOptions.PICK_ONLY_FIRST_FACE \
        #                    | avango.gua.PickingOptions.GET_POSITIONS \
        #                    | avango.gua.PickingOptions.GET_NORMALS \
        #                    | avango.gua.PickingOptions.GET_WORLD_POSITIONS \
        #                    | avango.gua.PickingOptions.GET_WORLD_NORMALS

        self.pick_options = avango.gua.PickingOptions.PICK_ONLY_FIRST_FACE \
                            | avango.gua.PickingOptions.GET_POSITIONS \
                            | avango.gua.PickingOptions.GET_NORMALS \
                            | avango.gua.PickingOptions.GET_WORLD_POSITIONS \
                            | avango.gua.PickingOptions.GET_WORLD_NORMALS



        ### resources ###

        ## init nodes
        self.pointer_node = avango.gua.nodes.TransformNode(Name = "pointer_node")
        self.pointer_node.Tags.value = ["invisible"]
        NAVIGATION_NODE.Children.value.append(self.pointer_node)
        

        self.ray = avango.gua.nodes.Ray() # required for trimesh intersection

        ## init field connections
        self.sf_button.connect_from(self.POINTER_INPUT.sf_button0)
        self.pointer_node.Transform.connect_from(self.POINTER_INPUT.sf_pointer_mat)
            
        self.always_evaluate(True) # change global evaluation policy


    ### functions ###
    def enable(self, BOOL):
        self.enable_flag = BOOL
        
        if self.enable_flag == True:
            self.pointer_node.Tags.value = [] # set tool visible
        else:
            self.stop_dragging() # possibly stop active dragging process
            
            self.pointer_node.Tags.value = ["invisible"] # set tool invisible

       
    def start_dragging(self, NODE):
        self.dragged_node = NODE        
        self.dragging_offset_mat = avango.gua.make_inverse_mat(self.pointer_node.WorldTransform.value) * self.dragged_node.WorldTransform.value # object transformation in pointer coordinate system

  
    def stop_dragging(self): 
        self.dragged_node = None
        self.dragging_offset_mat = avango.gua.make_identity_mat()


    def dragging(self):
        if self.dragged_node is not None: # object to drag
            _new_mat = self.pointer_node.WorldTransform.value * self.dragging_offset_mat # new object position in world coodinates
            _new_mat = avango.gua.make_inverse_mat(self.dragged_node.Parent.value.WorldTransform.value) * _new_mat # transform new object matrix from global to local space
        
            self.dragged_node.Transform.value = _new_mat


    def get_roll_angle(self, MAT4):
        _dir_vec = avango.gua.make_rot_mat(MAT4.get_rotate()) * avango.gua.Vec3(0.0,0.0,-1.0)
        _dir_vec = avango.gua.Vec3(_dir_vec.x, _dir_vec.y, _dir_vec.z) # cast to Vec3
        
        _ref_side_vec = avango.gua.Vec3(1.0,0.0,0.0)
   
        _up_vec = _dir_vec.cross(_ref_side_vec)
        _up_vec.normalize()
        _ref_side_vec = _up_vec.cross(_dir_vec)
        _ref_side_vec.normalize()      
        
        _side_vec = avango.gua.make_rot_mat(MAT4.get_rotate()) * avango.gua.Vec3(1.0,0.0,0.0)    
        _side_vec = avango.gua.Vec3(_side_vec.x, _side_vec.y, _side_vec.z) # cast to Vec3
        #print(_ref_side_vec, _side_vec)

        _axis = _ref_side_vec.cross(_side_vec)
        _axis.normalize()
    
        _angle = math.degrees(math.acos(min(max(_ref_side_vec.dot(_side_vec), -1.0), 1.0)))
        #print(_angle)
        
        if _side_vec.y > 0.0: # simulate rotation direction
            _angle *= -1.0
                
        return _angle



    def update_intersection(self, PICK_MAT = avango.gua.make_identity_mat(), PICK_LENGTH = 1.0):
        # update ray parameters
        self.ray.Origin.value = PICK_MAT.get_translate()

        _vec = avango.gua.make_rot_mat(PICK_MAT.get_rotate_scale_corrected()) * avango.gua.Vec3(0.0,0.0,-1.0)
        _vec = avango.gua.Vec3(_vec.x,_vec.y,_vec.z)

        self.ray.Direction.value = _vec * PICK_LENGTH

        ## trimesh intersection
        self.mf_pick_result = self.SCENEGRAPH.ray_test(self.ray, self.pick_options, self.white_list, self.black_list)

   
   
    def selection(self):
        if len(self.mf_pick_result.value) > 0: # intersection found
            self.pick_result = self.mf_pick_result.value[0] # get first pick result

        else: # nothing hit
            self.pick_result = None


        ## disable previous node highlighting
        if self.selected_node is not None:
            for _child_node in self.selected_node.Children.value:
                if _child_node.get_type() == 'av::gua::TriMeshNode':
                    _child_node.Material.value.set_uniform("enable_color_override", False)


        if self.pick_result is not None: # something was hit
            self.selected_node = self.pick_result.Object.value # get intersected geometry node
            self.selected_node = self.selected_node.Parent.value # take the parent node of the geomtry node (the whole object)

        else:
            self.selected_node = None


        ## enable node highlighting
        if self.selected_node is not None:
            for _child_node in self.selected_node.Children.value:
                if _child_node.get_type() == 'av::gua::TriMeshNode':
                    _child_node.Material.value.set_uniform("enable_color_override", True)
                    _child_node.Material.value.set_uniform("override_color", avango.gua.Vec4(1.0,0.0,0.0,0.3)) # 30% color override


                


    ### callback functions ###

    @field_has_changed(sf_button)
    def sf_button_changed(self):
        if self.sf_button.value == True: # button pressed
            if self.selected_node is not None:
                self.start_dragging(self.selected_node)

        else: # button released
            self.stop_dragging()
            
            
    def evaluate(self): # evaluated every frame
        raise NotImplementedError("To be implemented by a subclass.")
            
            

class Ray(ManipulationTechnique):

    ## constructor
    def __init__(self):
        self.super(Ray).__init__()


    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        POINTER_INPUT = None,
        ):

        ManipulationTechnique.my_constructor(self, SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT) # call base-class constructor


        ### parameters ###
        self.ray_length = 2.5 # in meter
        self.ray_thickness = 0.01 # in meter

        self.intersection_point_size = 0.03 # in meter


        ### resources ###
        _loader = avango.gua.nodes.TriMeshLoader()

        self.ray_geometry = _loader.create_geometry_from_file("ray_geometry", "data/objects/cylinder.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.ray_geometry.Transform.value = \
            avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.5) * \
            avango.gua.make_scale_mat(self.ray_thickness, self.ray_thickness, self.ray_length)
        self.ray_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))
        self.pointer_node.Children.value.append(self.ray_geometry)


        self.intersection_geometry = _loader.create_geometry_from_file("intersection_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.intersection_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))
        SCENEGRAPH.Root.value.Children.value.append(self.intersection_geometry)


        ### set initial states ###
        self.enable(False)



    ### functions ###
    def enable(self, BOOL): # extend respective base-class function
        ManipulationTechnique.enable(self, BOOL) # call base-class function

        if self.enable_flag == False:
            self.intersection_geometry.Tags.value = ["invisible"] # set intersection point invisible


    def update_ray_visualization(self, PICK_WORLD_POS = None, PICK_DISTANCE = 0.0):
        if PICK_WORLD_POS is None: # nothing hit
            # set ray to default length
            self.ray_geometry.Transform.value = \
                avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.5) * \
                avango.gua.make_scale_mat(self.ray_thickness, self.ray_thickness, self.ray_length)
        
            self.intersection_geometry.Tags.value = ["invisible"] # set intersection point invisible

        else: # something hit
            # update ray length and intersection point
            self.ray_geometry.Transform.value = \
                avango.gua.make_trans_mat(0.0,0.0,PICK_DISTANCE * -0.5) * \
                avango.gua.make_scale_mat(self.ray_thickness, self.ray_thickness, PICK_DISTANCE)

            self.intersection_geometry.Tags.value = [] # set intersection point visible
            self.intersection_geometry.Transform.value = avango.gua.make_trans_mat(PICK_WORLD_POS) * avango.gua.make_scale_mat(self.intersection_point_size)


    ### callback functions ###
    def evaluate(self): # implement respective base-class function
        if self.enable_flag == False:
            return
    

        ## calc ray intersection
        ManipulationTechnique.update_intersection(self, PICK_MAT = self.pointer_node.WorldTransform.value, PICK_LENGTH = self.ray_length) # call base-class function

        ## update object selection
        ManipulationTechnique.selection(self) # call base-class function

        ## update visualizations
        if self.pick_result is None:
            self.update_ray_visualization() # apply default ray visualization
        else:
            _node = self.pick_result.Object.value # get intersected geometry node
    
            _pick_pos = self.pick_result.Position.value # pick position in object coordinate system
            _pick_world_pos = self.pick_result.WorldPosition.value # pick position in world coordinate system
    
            _distance = self.pick_result.Distance.value * self.ray_length # pick distance in ray coordinate system
    
            #print(_node, _pick_pos, _pick_world_pos, _distance)
        
            self.update_ray_visualization(PICK_WORLD_POS = _pick_world_pos, PICK_DISTANCE = _distance)

        
        ## possibly perform object dragging
        ManipulationTechnique.dragging(self) # call base-class function



class DepthRay(ManipulationTechnique):

    ## constructor
    def __init__(self):
        self.super(DepthRay).__init__()


    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        POINTER_INPUT = None,
        ):

        ManipulationTechnique.my_constructor(self, SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT) # call base-class constructor


        ### parameters ###
        self.ray_length = 2.5 # in meter
        self.ray_thickness = 0.01 # in meter
        self.depth_marker_size = 0.07
        self.depth = 0

        self.pick_options =  avango.gua.PickingOptions.GET_POSITIONS \
                    | avango.gua.PickingOptions.GET_NORMALS \
                    | avango.gua.PickingOptions.GET_WORLD_POSITIONS \
                    | avango.gua.PickingOptions.GET_WORLD_NORMALS


        ### resources ###
        _loader = avango.gua.nodes.TriMeshLoader()

        self.ray_geometry = _loader.create_geometry_from_file("ray_geometry", "data/objects/cylinder.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.ray_geometry.Transform.value = \
            avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.5) * \
            avango.gua.make_scale_mat(self.ray_thickness, self.ray_thickness, self.ray_length)
        self.ray_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))
        self.pointer_node.Children.value.append(self.ray_geometry)
        
        self.intersection_geometry = _loader.create_geometry_from_file("intersection_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.intersection_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))
        self.intersection_geometry.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.25) * avango.gua.make_scale_mat(self.depth_marker_size)
        self.pointer_node.Children.value.append(self.intersection_geometry)

        ### set initial states ###
        self.enable(False)

    
    def update_ray_visualization(self):

        roll_angle = self.get_roll_angle(self.pointer_node.Transform.value)

        # increment depth according to roll angle - rate control

        depth_factor = 0.0003
        self.depth -= (depth_factor * roll_angle);

        # limit to ray length
        if self.depth > self.ray_length:
            self.depth = self.ray_length
        elif self.depth < 0:
            self.depth = 0


        self.ray_geometry.Transform.value = \
            avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.5) * \
            avango.gua.make_scale_mat(self.ray_thickness, self.ray_thickness, self.ray_length)

        # render ball
        self.intersection_geometry.Transform.value = \
            avango.gua.make_trans_mat(0.0,0.0,-self.depth) * \
            avango.gua.make_scale_mat(self.depth_marker_size)

    ### callback functions ###
    def evaluate(self): # implement respective base-class function
        if self.enable_flag == False:
            return

        ## calc ray intersections
        ManipulationTechnique.update_intersection(self, PICK_MAT = self.pointer_node.WorldTransform.value, PICK_LENGTH = self.ray_length) # call base-class function

        # decide which object should be selected
        self.depth_ball_selection() 

        # render
        self.update_ray_visualization()

        ## possibly perform object dragging
        ManipulationTechnique.dragging(self) # call base-class function

    # compares ray intersections with ball position, returns object closest to ball
    def depth_ball_selection(self):

        closestThing = None

        if len(self.mf_pick_result.value) > 0: # intersection found

            min_distance = sys.float_info.max
            for thing in self.mf_pick_result.value:

                thing_depth = (self.pointer_node.WorldTransform.value.get_translate() - thing.WorldPosition.value).length()
                distance = abs(self.depth - thing_depth)
                if distance < min_distance :
                    min_distance = distance
                    closestThing = thing


            self.pick_result = closestThing

        else: # nothing hit
            self.pick_result = None

        # create list of parent nodes of intersected geometry
        ray_object_parents = []
        if len(self.mf_pick_result.value) > 0: # intersection found
            for thing in self.mf_pick_result.value: 
                node = thing.Object.value # get intersected geometry node
                node = node.Parent.value # take the parent node of the geomtry node (the whole object)
                ray_object_parents.append(node)
        # remove duplicates
        ray_object_parents = set(ray_object_parents)
        ray_object_parents = list(ray_object_parents)


        # save selected node
        if self.pick_result is not None: # something was hit
            self.selected_node = self.pick_result.Object.value # get intersected geometry node
            self.selected_node = self.selected_node.Parent.value # take the parent node of the geomtry node (the whole object)

        else:
            self.selected_node = None

        # unhighlight everything in ray path, that is not intersected on this frame
        if self.ray_selected_objects is not None: 
            for thing in self.ray_selected_objects.value :
                geom_node = thing.Object.value # get intersected geometry node
                geom_parent_node = geom_node.Parent.value # take the parent node of the geomtry node (the whole object)

                # check if parent is in new list of intersected objects
                should_unhighlight = True
                for hl_node in ray_object_parents :
                    if hl_node == geom_parent_node:
                        should_unhighlight = False
                        break

                if should_unhighlight :
                    for _child_node in geom_parent_node.Children.value:
                        if _child_node.get_type() == 'av::gua::TriMeshNode':
                            _child_node.Material.value.set_uniform("enable_color_override", False)

        # save new list of intersected objects
        self.ray_selected_objects = self.mf_pick_result

        # highlight everything in ray path apart from closest thing
        if self.ray_selected_objects is not None: 
            for thing in self.ray_selected_objects.value:
                node_to_colour = thing.Object.value # get intersected geometry node
                node_to_colour = node_to_colour.Parent.value # take the parent node of the geomtry node (the whole object)
                if node_to_colour != self.selected_node :
                    for _child_node in node_to_colour.Children.value:
                        if _child_node.get_type() == 'av::gua::TriMeshNode':
                            _child_node.Material.value.set_uniform("enable_color_override", True)
                            _child_node.Material.value.set_uniform("override_color", avango.gua.Vec4(1.0,0.0,0.0,0.3)) # 30% color override


        ## highlight closest node in yellow
        if self.selected_node is not None:
            for _child_node in self.selected_node.Children.value:
                if _child_node.get_type() == 'av::gua::TriMeshNode':
                    _child_node.Material.value.set_uniform("enable_color_override", True)
                    _child_node.Material.value.set_uniform("override_color", avango.gua.Vec4(1.0,0,0.0,1.0))

class GoGo(ManipulationTechnique):

    ## constructor
    def __init__(self):
        self.super(GoGo).__init__()


    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        HEAD_NODE = None,
        POINTER_INPUT = None,
        TARGET_LIST = []
        ):

        ManipulationTechnique.my_constructor(self, SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT) # call base class constructor


        ### external references ###
        self.HEAD_NODE = HEAD_NODE

        self.target_list = TARGET_LIST

        self.selected_node = None


        self.body_node = avango.gua.nodes.TransformNode(Name = "body_node")
        self.body_node.Tags.value = ["invisible"]
        self.body_node.Transform.value = avango.gua.make_trans_mat(0.0,-0.3,-0.1)
        self.HEAD_NODE.Children.value.append(self.body_node)




        self.gogo_hand_node = avango.gua.nodes.TransformNode(Name = "gogo_hand_node")
        self.gogo_hand_node.Transform.value = avango.gua.make_trans_mat(0.0,0.0,0.0)
        self.pointer_node.Children.value.append(self.gogo_hand_node)

        self.hand_end_node = avango.gua.nodes.TransformNode(Name = "hand_end_node")
        self.hand_end_node.Tags.value = ["invisible"]
        self.hand_end_node.Transform.value = avango.gua.make_trans_mat(0.0,0.0,-0.2)
        self.gogo_hand_node.Children.value.append(self.hand_end_node)
        

        ### parameters ###  
        self.intersection_point_size = 2.000 # in meter
        self.gogo_threshold = 0.5 # in meter

        self.gogo_vec = avango.gua.Vec3(0.0,0.0,-1.0)
        self.gogoindex = 0
        self.factor = 0
        self.k = 15


        ### resources ###
        _loader = avango.gua.nodes.TriMeshLoader()
 
        ## To-Do: init (geometry) nodes here
        self.hand_geometry = _loader.create_geometry_from_file("hand_geometry", "data/objects/hand.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.hand_geometry.Transform.value = \
                    avango.gua.make_scale_mat(self.intersection_point_size, self.intersection_point_size, self.intersection_point_size)
        self.hand_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))
        self.gogo_hand_node.Children.value.append(self.hand_geometry)
 
        ### set initial states ###
        self.enable(False)

    def update_hand_visualization(self):

        self.gogo_hand_node.Transform.value = avango.gua.make_trans_mat(self.gogo_vec*self.factor) 


    ### callback functions ###
    def evaluate(self): # implement respective base-class function
        if self.enable_flag == False:
            return

        # distance from body

        body_ps = self.body_node.WorldTransform.value * avango.gua.make_inverse_mat(self.pointer_node.WorldTransform.value )
        body_pos_ps = body_ps * avango.gua.Vec4(0,0,0,1.0)
        body_pos_ps = avango.gua.Vec3(body_pos_ps.x,body_pos_ps.y,body_pos_ps.z)

        self.gogo_vec = avango.gua.Vec3(0,0,0) - body_pos_ps

        # dist = self.gogo_vec.length()
        dist = (self.body_node.WorldTransform.value.get_translate() - self.pointer_node.WorldTransform.value.get_translate())
        dist.y = 0
        dist.x=0
        dist = dist.length()


        self.gogoindex += 1
        if dist > self.gogo_threshold:
            new_dist  = dist + self.k * (dist - self.gogo_threshold)**2
            self.factor = new_dist/dist - 1

            self.hand_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,1.0,0.0,1.0))

            
        else:
            self.factor = 0
            self.hand_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))


        # check for intersection at hand origin and end of hand
        _hand_pos = self.gogo_hand_node.WorldTransform.value.get_translate()
        _hand_end_pos = self.hand_end_node.WorldTransform.value.get_translate()

                ## disable previous node highlighting
        if self.selected_node is not None:
            for _child_node in self.selected_node.Children.value:
                if _child_node.get_type() == 'av::gua::TriMeshNode':
                    _child_node.Material.value.set_uniform("enable_color_override", False)
    
        self.selected_node = None
        for _node in self.target_list: # iterate over all target nodes
            _bb = _node.BoundingBox.value # get bounding box of a node
            
            if _bb.contains(_hand_pos) == True or _bb.contains(_hand_end_pos) == True: # hook inside bounding box of this node
                self.selected_node = _node
                break


        ## enable node highlighting
        if self.selected_node is not None:
            for _child_node in self.selected_node.Children.value:
                if _child_node.get_type() == 'av::gua::TriMeshNode':
                    _child_node.Material.value.set_uniform("enable_color_override", True)
                    _child_node.Material.value.set_uniform("override_color", avango.gua.Vec4(1.0,0.0,0.0,0.3)) # 30% color override


        ## possibly perform object dragging
        self.dragging() # call base-class function

        self.update_hand_visualization()



    def dragging(self):
        if self.dragged_node is not None: # object to drag
            _new_mat = self.gogo_hand_node.WorldTransform.value * self.dragging_offset_mat # new object position in world coodinates
            _new_mat = avango.gua.make_inverse_mat(self.dragged_node.Parent.value.WorldTransform.value) * _new_mat # transform new object matrix from global to local space
        
            self.dragged_node.Transform.value = _new_mat

    def start_dragging(self, NODE):
        self.dragged_node = NODE        
        self.dragging_offset_mat = avango.gua.make_inverse_mat(self.gogo_hand_node.WorldTransform.value) * self.dragged_node.WorldTransform.value # object transformation in pointer coordinate system

            

class VirtualHand(ManipulationTechnique):

    ## constructor
    def __init__(self):
        self.super(ManipulationTechnique).__init__()


    def my_constructor(self,
        SCENEGRAPH = None,
        NAVIGATION_NODE = None,
        POINTER_INPUT = None,
        TARGET_LIST = []
        ):

        ManipulationTechnique.my_constructor(self, SCENEGRAPH, NAVIGATION_NODE, POINTER_INPUT) # call base-class constructor


        ### parameters ###
        self.intersection_point_size = 1 # in meter
        self.ref_ball_size = 0.01

        self.min_vel = 0.01 / 60.0 # in meter/sec
        self.sc_vel = 0.15 / 60.0 # in meter/sec
        self.max_vel = 0.25 / 60.0 # in meter/sec

        self.target_list = TARGET_LIST

        ### resources ###
        _loader = avango.gua.nodes.TriMeshLoader()

        self.prism_hand_node = avango.gua.nodes.TransformNode(Name = "gogo_hand_node")
        self.prism_hand_node.Transform.value = avango.gua.make_trans_mat(0.0,0.0,0.0)
        self.pointer_node.Children.value.append(self.prism_hand_node)

        ## To-Do: init (geometry) nodes here
        self.hand_geometry = _loader.create_geometry_from_file("hand_geometry", "data/objects/hand.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.hand_geometry.Transform.value = \
                        avango.gua.make_scale_mat(self.intersection_point_size, self.intersection_point_size, self.intersection_point_size)
        self.hand_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))
        self.prism_hand_node.Children.value.append(self.hand_geometry)


        # reference ball
        self.ref_ball = _loader.create_geometry_from_file("ref_ball", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.ref_ball.Transform.value = \
                        avango.gua.make_scale_mat(self.ref_ball_size, self.ref_ball_size, self.ref_ball_size)
        self.ref_ball.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,1.0,1.0,1.0))
        self.pointer_node.Children.value.append(self.ref_ball)
        

        self.last_frame_time = time.time()

        self.last_position = avango.gua.Vec3(0,0,0)
        self.hand_translate =  avango.gua.Vec3(0,0,0)


        ### set initial states ###
        self.enable(False)

    def update_hand_visualization(self):

        # take rotate from pointer node and apply it to prism hand node
        rotate = self.pointer_node.Transform.value.get_rotate()

        self.pointer_node.Transform.value = \
            avango.gua.make_trans_mat(self.pointer_node.Transform.value.get_translate()) * \
            avango.gua.make_scale_mat(self.pointer_node.Transform.value.get_scale())

        # render hand
        self.prism_hand_node.Transform.value = avango.gua.make_trans_mat(self.hand_translate) * \
                                avango.gua.make_rot_mat(rotate) * \
                        avango.gua.make_scale_mat(self.intersection_point_size, self.intersection_point_size, self.intersection_point_size)





        ### callback functions ###
    def evaluate(self): # implement respective base-class function
        if self.enable_flag == False:
            return

        frame_time = time.time()
        frame_duration = frame_time - self.last_frame_time
        self.last_frame_time = frame_time

        pointer_pos = self.pointer_node.Transform.value.get_translate()

        frame_movement = pointer_pos - self.last_position

        move_speed = (pointer_pos - self.last_position) / (frame_duration * 60) #m/s


        self.hand_translate.x = self.update_offset(self.hand_translate.x, frame_movement.x, frame_duration)
        self.hand_translate.y = self.update_offset(self.hand_translate.y, frame_movement.y, frame_duration)
        self.hand_translate.z = self.update_offset(self.hand_translate.z, frame_movement.z, frame_duration)

        _hand_pos = self.prism_hand_node.WorldTransform.value.get_translate() 
        #         # disable previous node highlighting
        if self.selected_node is not None:
            for _child_node in self.selected_node.Children.value:
                if _child_node.get_type() == 'av::gua::TriMeshNode':
                    _child_node.Material.value.set_uniform("enable_color_override", False)
    
        self.selected_node = None
        for _node in self.target_list: # iterate over all target nodes
            _bb = _node.BoundingBox.value # get bounding box of a node
            
            if _bb.contains(_hand_pos) == True: # hook inside bounding box of this node
                self.selected_node = _node
                break

        ## enable node highlighting
        if self.selected_node is not None:
            for _child_node in self.selected_node.Children.value:
                if _child_node.get_type() == 'av::gua::TriMeshNode':
                    _child_node.Material.value.set_uniform("enable_color_override", True)
                    _child_node.Material.value.set_uniform("override_color", avango.gua.Vec4(1.0,0.0,0.0,0.3)) # 30% color override


        self.last_position = pointer_pos





         ## possibly perform object dragging
        ManipulationTechnique.dragging(self) # call base-class function
        self.update_hand_visualization()

    def update_offset(self, current_offset, frame_movement, frame_duration) :

        move_speed = abs(frame_movement) / (frame_duration * 60)

        if move_speed > self.max_vel and abs(current_offset) > 0:
            return current_offset *0.9

        else :
            return current_offset - (frame_movement * self.get_factor_value(move_speed))

    def get_factor_value(self,move_speed):
        if(move_speed < self.min_vel):
            factor = 1
        elif move_speed < self.sc_vel:
            factor = self.map_speed_to_movement_factor(move_speed)

        else :
            factor = 0
        return factor


    def map_speed_to_movement_factor(self, input):
        min_in = self.min_vel
        min_out = self.sc_vel
        max_in = 0
        max_out = 1
        output = max_in + ((max_out - max_in) / (min_out - min_in)) * (input - min_in)
        return (1-output)

    def dragging(self):
        if self.dragged_node is not None: # object to drag
            _new_mat = self.prism_hand_node.WorldTransform.value * self.dragging_offset_mat # new object position in world coodinates
            _new_mat = avango.gua.make_inverse_mat(self.dragged_node.Parent.value.WorldTransform.value) * _new_mat # transform new object matrix from global to local space
        
            self.dragged_node.Transform.value = _new_mat

    # def start_dragging(self, NODE):
    #     self.dragged_node = NODE        
    #     self.dragging_offset_mat = avango.gua.make_inverse_mat(self.prism_hand_node.WorldTransform.value) * self.dragged_node.WorldTransform.value # object transformation in pointer coordinate system

            
