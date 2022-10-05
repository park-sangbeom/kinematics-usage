import fcl 
import math 
import numpy as np
from tf.transformations import quaternion_from_euler 

""" Flexible Collision Library """
class PyFCL:
    def __init__(self, _num_max_contacts = 100, verbose = True):
        self.geom_lst = []
        self.objs_lst = []
        self.name_lst = [] 
        self.verbose  = verbose
        self.req      = fcl.CollisionRequest()
        self.res      = fcl.CollisionResult()
        self.cdata    = fcl.CollisionData(request = self.req)

    # capsule type: [position[0], position[1], position[2], rpy[0], rpy[1], rpy[2], height, radius]
    # box type: [position[0], position[1], position[2], rpy[0], rpy[1], rpy[2], size[0], size[1], size[2]]
    def fclize_obj(self, obj_info): 
        obj_name = obj_info["name"]
        obj_type = obj_info["type"]
        obj_pos  = obj_info["position"]
        obj_rpy  = obj_info["orientation"]
        obj_size = obj_info["size"]

        if obj_type == "capsule":
            obj_height = obj_size[0]
            obj_radius = obj_size[1]
            obj_q = quaternion_from_euler (obj_rpy[0], obj_rpy[1], obj_rpy[2]) 
            obj_t = np.array(obj_pos)
            obj_g = fcl.Capsule(obj_radius, obj_height)
            obj_t = fcl.Transform(obj_q, obj_t)
            obj_o = fcl.CollisionObject(obj_g, obj_t)
 
        if obj_type == "box": 
            obj_q = quaternion_from_euler (obj_rpy[0], obj_rpy[1], obj_rpy[2]) 
            obj_t = np.array(obj_pos)            
            obj_g = fcl.Box(obj_size[0], obj_size[1], obj_size[2])
            obj_t = fcl.Transform(obj_q, obj_t)
            obj_o = fcl.CollisionObject(obj_g, obj_t)
        self.geom_lst.append(obj_g)
        self.objs_lst.append(obj_o)
        self.name_lst.append(obj_name)
        return obj_o 

    def one2one_cc(self, fclize_obj1, fclize_obj2):
        collision = fcl.collide(fclize_obj1, fclize_obj2, self.req, self.res)
        """ Collision Check """
        if collision: 
            if self.verbose: 
                print("Collision Detection")
            return True # Collision detection
        else: 
            if self.verbose: 
                print("Collision Free")
            return False  # Collision free 

    def one2many_cc(self, obj, obj_list):
        manager = fcl.DynamicAABBTreeCollisionManager()
        manager.registerObjects(obj_list)
        manager.setup()
        manager.collide(obj, self.cdata, fcl.defaultCollisionCallback)
        collision = self.cdata.result.is_collision
        objs_in_collision = set() 
        # Create map from geometry IDs to objects
        geom_id_to_obj  = {id(geom) : obj for geom, obj in zip(self.geom_lst, self.objs_lst)}
        # Create map from geometry IDs to string names
        geom_id_to_name = {id(geom) : name for geom, name in zip(self.geom_lst, self.name_lst)}
        for contact in self.cdata.result.contacts:
            coll_geom_0 = contact.o1
            coll_geom_1 = contact.o2

            # Get their names
            coll_names = [geom_id_to_name[id(coll_geom_0)], geom_id_to_name[id(coll_geom_1)]]
            coll_names = tuple(sorted(coll_names))
            objs_in_collision.add(coll_names)
        if self.verbose: 
            for coll_pair in objs_in_collision:
                print('*WARNING* {} in collision with {}!'.format(coll_pair[0], coll_pair[1]))
        """ Collision Check """
        if collision: 
            return True # Collision detection
        else: 
            print("Collision Free")
            return False  # Collision free 