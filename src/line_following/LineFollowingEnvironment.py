"""
N-task line following scenario
robot starts at x=0, y depends on track, zu (on ground). Wall is 110m wide, 2m high and 0.5m in width -->
initial distance to wall is 0.75m, robot can go 0.4m max.
"""
from gazebo_sim.simulation import ThreePiTask, TwistAction, ThreePiEnvironment
from gazebo_sim.simulation import ThreePiManager
import os

from gz.msgs10.material_color_pb2 import MaterialColor
from gz.msgs10.color_pb2 import Color

import numpy as np ;
import math ;
import gymnasium as gym ;

from cl_experiment.parsing import Kwarg_Parser ;


# define names for colors, RGB values and the names of the LED links in the robot model. 
# names here must coincide with names in track_defs.txt
color_array = [ 
    ("red",      (1., 0, 0, 1.), "led1_link"),     # Rot    
    ("blue",    (0, 0, 1., 1.), "led2_link"),     # Blau
    ("green",    (0, 1., 0, 1.), "led3_link"),     # Grün
    ("cyan",     (0,1.,1.,1.), "led4_link"),     # Türkis
    ("magenta",  (1., 0, 1.,1.), "led5_link"),   # Magenta
    ("black",     (0, 0, 0,1.), "led6_link")        # Black
]


# mapping color_str --> color_index(0..5)
color_dict = { key:index for index, (key,tpl, link) in enumerate(color_array) } ;

# in 30 steps, robot can go at most 0.4m = 30 frames * 1/15 s/frame * 0.2m/s, since 0.2m/s is constant fwd speed whatever the direction
raw_action_space = np.array([
            [0.15, 0.25],                   #Left       [0.05, 0.35],
            [0.2, 0.2],                     #Forward 
            [0.25, 0.15],                   #Right      [0.35, 0.05]
            ]) ;

# action class that represents wheel speeds, as well as a LED to switch on
class Lamp_TwistAction(TwistAction):
    def __init__(self, name, wheel_speeds, Lamp_Materialt, separation=.1):
        super().__init__(name, wheel_speeds)
        self.Lamp_Materialt = Lamp_Materialt

    def return_material(self):
        return self.Lamp_Materialt.material
      
# wrapper class around a single led. The relevant instance is self.material, 
# can be used to feed a Gazebo MaterialColor message 
class Lamp_Material():
    def __init__(self, name, diffuse, ambient):
        self.name = f"{diffuse.r}-{diffuse.g}-{diffuse.b}-{ambient.r}-{ambient.g}-{ambient.b}"
        self.diffuse = diffuse
        self.ambient = ambient

        self.material = MaterialColor()
        # apply to all entities, apparently this is important, see https://github.com/gazebosim/gz-sim/blob/gz-sim10/src/systems/user_commands/UserCommands.cc, line 1705
        self.material.entity_match = int(1) ;
        self.material.entity.name = name
        self.material.diffuse.r = diffuse.r
        self.material.diffuse.g = diffuse.g
        self.material.diffuse.b = diffuse.b
        self.material.diffuse.a = diffuse.a
        
        self.material.ambient.r = ambient.r
        self.material.ambient.g = ambient.g
        self.material.ambient.b = ambient.b
        self.material.ambient.a = ambient.a

    def return_material(self):
        return self.material

    def set_entity(self, e: str):
        self.material.entity.name = e ;

    def to_string(self):
        return f'entity: {name: {self.name}} diffuse: {r:{self.diffuse.r}, g:d{self.diffuse.g}, b:{self.diffuse.b}, a:{self.diffuse.a}}   ambient: {r:{self.ambient.r}, g:{self.ambient.g}, b:{self.ambient.b},a:{self.ambient.a}} ' 


    def __str__(self):
        return f'entity: {name: {self.name}} diffuse: {r:{self.diffuse.r}, g:d{self.diffuse.g}, b:{self.diffuse.b}, a:{self.diffuse.a}}   ambient: {r:{self.ambient.r}, g:{self.ambient.g}, b:{self.ambient.b},a:{self.ambient.a}} '


# define high-level action space
# For proper online setting of robot colors, we need the format "world::model::link::visual" for the name of the entity. Beats me why but its a fact
def define_robot_actions(world_name, robot_name):
  robot_actions = [] ;
  for name, rgb, link_name in  color_array:
    link_id = f"{world_name}::{robot_name}::{link_name}::visual" ;
    for wheel_speeds in raw_action_space:
        diff = np.diff(wheel_speeds)
        if diff == 0: 
            robot_actions.append(
              Lamp_TwistAction('forward_'+name, wheel_speeds, 
                Lamp_Material(link_id,
                Color(r=rgb[0], g=rgb[1], b=rgb[2], a=rgb[3]),     # diffuse
                Color(r=rgb[0], g=rgb[1], b=rgb[2], a=rgb[3]))      # ambient
              ))
        elif diff < 0: 
            robot_actions.append(
              Lamp_TwistAction('right_'+name, wheel_speeds, 
                Lamp_Material(link_id,
                Color(r=rgb[0], g=rgb[1], b=rgb[2], a=rgb[3]),     # diffuse
                Color(r=rgb[0], g=rgb[1], b=rgb[2], a=rgb[3]))      # ambient
              ))
        elif diff > 0: 
            robot_actions.append(
              Lamp_TwistAction('left_'+name, wheel_speeds, 
                Lamp_Material(link_id,
                Color(r=rgb[0], g=rgb[1], b=rgb[2], a=rgb[3]),     # diffuse
                Color(r=rgb[0], g=rgb[1], b=rgb[2], a=rgb[3]))      # ambient
            ))

  return robot_actions ;


# Extended Manager class for controlling a 3pi robot via wheel speeds and LEDs
class ThreePiWithLampManager(ThreePiManager):
    def __init__(self,env_config, **kwargs):
        super().__init__(env_config, **kwargs)
        self.gz_change_material = self.advertise(f'{self.world_name}/material_color', MaterialColor)
        self.robot_name = env_config["robot_name"] ;
        self.black_material = Lamp_Material("black",
                Color(r=0, g=0, b=0, a=1),     # diffuse
                Color(r=0, g=0, b=0, a=1))      # ambient
        self.stop_action = TwistAction("stop", [0.,0.])


    def gz_perform_action(self, action: Lamp_TwistAction):
        # execute steering part
        ThreePiManager.gz_perform_action(self, action) ;
        # blank out all leds
        for (name, tpl, link_name) in color_array:
          self.black_material.set_entity(f"line_following_world::{self.robot_name}::{link_name}::visual") ;
          self.gz_change_material.publish(self.black_material.material) ;
        # switch on "new" led
        self.gz_change_materia(action)

    # a "stop" action is not part of some action spaces, so define it here in case we 
    # need to stop the robot
    def gz_perform_action_stop(self):
      ThreePiManager.gz_perform_action(self, self.stop_action ) ;

    def gz_change_materia(self, action:Lamp_TwistAction):
        self.gz_change_material.publish(action.return_material())


class LineFollowingEnvironment(ThreePiEnvironment):
    def __init__(self, step_duration_nsec=100 * 1000 * 1000, **kwargs) -> None:

        args = self.parse_args(**kwargs) ;
        self.config = args ;
        self.fake_inputs = args.fake_inputs ;
        self.root_path = args.root_dir ;

        if self.config.fake_inputs == "no" and self.config.external_steering == "yes":
          print("External steering xannot be used with real inputs!!") ;
          sys.exit(0) ;

        
        self.external_steering = args.external_steering
        
        self.min_right = 2
        self.min_left = 2  
    
        single_tasks = {}
        
        # read track definition file
        exp_count = 150 ;
        try:
            with open(self.root_path + '/track_defs.txt', 'r') as f:
                track_defs = [l.strip() for l in f.readlines()] ;
                print("Read track defs", len(track_defs))
        except FileNotFoundError:
            print("Datei nicht gefunden.")
            sys.exit(0) ;

        # read line colors, lamp colors and starting positions from file
        index = 0;
        for i in range(0,150):
            color_str = track_defs[index].split("-")[0].strip() ;
            lamp_str = track_defs[index].split("-")[1].strip() ;
            coord_str = track_defs[index].split("-")[2].strip() ;
            name = "track_"+str(index); 
            xnew = float(coord_str.split(" ")[0]) ;
            ynew = float(coord_str.split(" ")[1]) ;
            print(xnew, ynew, color_str) ;
            single_tasks[name] = {"name": name, "pos": [xnew,ynew,0.05], "colors": color_str, "lamps": lamp_str } ;
            index +=1;

        ## define agregate tasks that are composed from randonly selected tracks from "tasks.txt"
        tasks = [] ;            
        file_path = os.path.join(self.root_path, 'tasks.txt')   ;
        if os.path.exists(file_path):
          task_definitions = [l.strip() for l in open(file_path,"r").readlines()] ;
          for i,line in enumerate(task_definitions):
            if len(line) <= 1: continue ;
            task = ThreePiTask(name="task"+str(i)) ;
            print("spawned task" + str(i)) ;
            for subtask_name in line.split():
              print("adding subtask", subtask_name) ;
              tpl = single_tasks[subtask_name] ;
              trafo1 = ThreePiTask.Transform(position = tpl["pos"], euler_rotation = [0.0,0.0,25]) ;
              trafo2 = ThreePiTask.Transform(position = tpl["pos"], euler_rotation = [0.0,0.0,-25]) ;
              task.add_starting_point({"name": subtask_name + "_1", "trafo": trafo1, "colors":tpl["colors"], "lamps":tpl["lamps"]}) ; 
              task.add_starting_point({"name": subtask_name + "_2", "trafo": trafo2, "colors":tpl["colors"], "lamps":tpl["lamps"]}) ; 
            tasks.append(task) ;
            print ("Task is", task.starting_points) ;
        else:
          print("Task definition file tasks.txt not found!") ;
          sys.exit(0) ;
          
        print("\n\n\n\n\n\n",tasks)

        self.world_name = 'line_following_world' ;
        self.robot_name = '3pi_robot_with_line_cam' ;
        actions = define_robot_actions(self.world_name, self.robot_name) ;
        

        
        # observation space
        channels = 3 ;
        observation_shape = [2, 50, channels] if self.fake_inputs == "no" else [3, 3*len(color_array)] ;
        env_config = {"observation_shape":observation_shape,"tasks":tasks,"actions":actions,"robot_name":self.robot_name,"vehicle_prefix":'/vehicle',
                                       "lidar":'/vehicle/lidar',"world_name":"/world/" + self.world_name,"camera_topic":'/vehicle/camera', 
                                       } ;
                                       

        super().__init__(env_config,step_duration_nsec, **kwargs) ;
        
        self.info = {
            "sequence_length":3, 
            "input_dims": self.observation_shape,
            "number_actions": self.nr_actions
        }
        

    def set_manager(self,env_config:dict):
        self.manager = ThreePiWithLampManager(env_config) ;


    # for stable baselines /gymnasium
    @property
    def action_space(self):
      return gym.spaces.Discrete(6) if self.external_steering == "yes" else gym.spaces.Discrete(4) ;

    @property
    def observation_space(self):
      return gym.spaces.Box(0,1,shape=(3,18), dtype=np.float32) if self.external_steering == "yes" else gym.spaces.Box(0,1,shape=(2,50,3),dtype=np.float32) ;



    # returns a tuple of floats for logging purposes
    def get_current_status(self):
        obj_name = self.info['object']
        return (self.info['object'], )


    def switch(self, task_index):
        super().switch(task_index)
        self.task_id = self.tasks[task_index].name ;
        print("switching to task", self.task_id, task_index)
        self.current_task = self.tasks[task_index] ;
        self.current_name  = "" ;
        self.info['object'] = (None, ) ;


    def reset(self, seed = 0, **kwargs):
        super().reset()       
        random_starting_point= self.current_task.get_random_start()
        self.start_transform = random_starting_point["trafo"] ;
        self.current_colors = random_starting_point["colors"].split(" ") ;
        self.current_lamp_colors = random_starting_point["lamps"].split(" ") ;
        self.current_name = random_starting_point["name"] ;
        self.info["object"] = (self.current_name) ;
        self.info["track_colors"] = self.current_colors ;
        print("Reset to OR:", self.start_transform.orientation)

        self.manager.trigger_pause(False) # resume sim

        self.manager.gz_perform_action_stop()  # send stop action
        self.get_observation(nsec=500*1000*1000.)  # extend get_obs spinning loop to 0.5s (5e8) or 1s (1e9)

        self.manager.perform_reset(self.start_transform.position,self.start_transform.orientation)

        response, laser, pose, contact = self.get_observation(nsec=(500. * 1000. * 1000.)) # wait again
        
        self.manager.trigger_pause(True) # block sim
        
        vis_state = self.manager.convert_image_msg(response)[1:3,::2,:] ; # downsampling: 4x100x3 --> 2x50x3
        self.embed_lidar(vis_state, laser) ;
        
        # compute reward just for fake obs generation, in case we should need it
        _ = self._compute_reward(vis_state, laser, -1) ;
        
        #obs = self.glue_images(state, self.step_count)
        obs = vis_state ;
        
        self.last_obs =  (obs if self.fake_inputs == "no" else self.create_fake_img() );
        return self.last_obs, self.info ;

    def process_action(self,action, obs):
        if self.config.external_steering == "no": return action ;

        # in this case, config.fake_inputs must be true as well --> obs is a fake input with shape 3,18
        steering_guide = obs[2,:].argmax() ;
        if steering_guide  < 7:
          return (action*3)+0 ; # left
        elif steering_guide > 10:
          return (action*3)+1 ; # right
        else:
          return (action*3)+2 ; # straight

    def step(self, action_index:int):

        action_index = self.process_action(action_index,self.last_obs) ;
        super().step(action_index)
        
        self.perform_action(action_index=action_index) # perform action

        self.manager.trigger_pause(False) # resume sim        
        
        self.step_count +=1

        # get resulting observation
        response, laser, pose, contact = self.get_observation(nsec=self.step_duration_nsec)

        self.manager.trigger_pause(True)    # pause sim
  
        vis_state = self.manager.convert_image_msg(response)[1:3,::2,:] ; # downsampling: 4x100 --> 2x50
        self.embed_lidar(vis_state, laser) ;

        reward = self._compute_reward(vis_state, laser, action_index)
        
        terminated = reward < 0.0           #-0.75
 
        truncated = self.step_count>=self.max_steps_per_episode
         
        self.last_obs = vis_state if self.fake_inputs == "no" else self.create_fake_img() ;

        return self.last_obs, reward, terminated, truncated, self.info

    def embed_lidar(self, vis_state, lidar):
      """ take smallest lidar distance and embed it into 2nd row of image as a population code, assuming a maximal distance of 1m """
      vis_state[1,:,:] = 0 ;
      laser_beams = lidar ;
      valid_beams = laser_beams[ laser_beams != np.inf ] ;
      if valid_beams.shape[0] == 0: return ; # no wall detected --> leave vision unchanged, not a problem since we are off track anyway
      min_range = valid_beams.min() ;
      print("MINRAN_GE", min_range) ;
      pc_pos = int (((min_range-0.35) / 0.41) * 50.) ;
      if pc_pos < 0: pc_pos = 0 ; 
      if pc_pos > 49: pc_pos = 49 ; 
      vis_state[1,pc_pos,:] = 1.0 ;
      
      
    def create_fake_img(self):
        """ create a simplified image where all relevant information is encoded directly """
        

        arr = np.zeros((3, 3*len(color_array)), dtype=np.float32) ;
        # extract indices of left, middle and right floor color from infos gathered at reset time (self.current_color)
        color_indices = [ color_dict[self.current_colors[i]] for i in range(0,3) ] ;

        #Farbe codieren
        for i, index in enumerate(color_indices):
            arr[0,i*len(color_array)+index] = 1. ;

        # Position der linkne ecke codieren: use infos from reward computation
        index_l = int( self.left_border/self.w * (3*len(color_array) - 1))
        arr[1, index_l] = 1

        # encode distance  to wall from lidar. If no valid distance --> nothing will be set here
        index_d = int((0.75-self.min_dist) / 0.4 * 3*len(color_array)) ;
        if self.min_dist < 10000:
          arr[2, index_d] = 1

        #Bsp.:
        #   [ Left_color(6) | Mid_color(6) | Right_color(6) ]       # Farben der Linien auf dem Boden
        #   [                 Position (18)                 ]
        #   [                 Distance (18)                 ]
        return arr


    def find_color_transition(self, img):
               
        # Farbunterschiede horizontal berechnen (zwischen Pixel (y, x) und (y, x+1))
        diff_h = ((img[1, :-1, :] - img[1, 1:, :])**2).sum(axis=1)
        # hier nach shape (49,)
        
        # Schwellenwert für Farbwechsel
        threshold = 0.1        

        # Maske der Farbwechsel
        color_change_mask = diff_h > threshold
        
        # Maske auf Originalgroesse bringen -> shape (49,) zu shape (50,)
        color_change_mask = np.pad(color_change_mask, ((0,1),), mode='constant', constant_values=False) ; # (50,)
        
        xs = np.where(color_change_mask)[0]
        if xs.size > 0:
                left_border = xs.min()
        else:
                print("PROBLEM: NO BONDARIES DETECTED") ;
                return False, -1.5 ;

        return True, left_border ;
        
    
    
    # --------------------------------> REWARD
    def _compute_reward(self, img, lidar, action_index):    
        # measures lidar distance to the wall as the minimum range obtained
        valid_lidar = (lidar != np.inf) ;
        self.min_dist = 100000. if valid_lidar.shape[0] == 0 else lidar[valid_lidar].min() ;
        print("DIST", self.min_dist) ;
        track_pos = "first_half" if self.min_dist > 0.75-0.2 else "second_half" ;

        self.h, self.w, _ = img.shape            
        self.ok, self.left_border = self.find_color_transition(img) ;

        # if color transition is lost or too far on the ldft/right --> return punishment
        if self.ok == False or (self.left_border < self.min_left) or (self.left_border > self.w - self.min_right):
            print(" REWARD: ", -0.5)
            return -0.5 ;

        # otherwise, the follow reward is based on the centering of the leftmost transition
        follow_reward = 1.- math.fabs(self.left_border-self.w//2) / (self.w//2) ;

        # lamp reward depends on whether the correct led is switched on
        lamp_color = action_index // 3 ; # 0..5
        target_lamp_color = int(self.current_lamp_colors[0]) if  self.min_dist < (0.75-0.2) else int(self.current_lamp_colors[1]) ;
        print("lamp color actual vs target: ", lamp_color, target_lamp_color) ;
        lamp_reward = 1. if lamp_color == target_lamp_color else 0. ;
        print("REWARD: follow=", follow_reward, "lamp=", lamp_reward) ;

        return follow_reward + lamp_reward
    
    def parse_args(self, **kwargs):
            
        base_cfg = super().parse_args(**kwargs)  
        parser = Kwarg_Parser(**kwargs) ;

        # Neuer Parser mit zusätzlichen Argumenten
        parser.add_argument('--fake_inputs', type=str, default='no', choices=['yes', 'no'],     help='Enable this mode to receive only simplified images.')
        parser.add_argument("--external_steering", type = str, required = False, default = "no") ;

        child_cfg, _ = parser.parse_known_args()
        # merge dicts
        for k,v in child_cfg.__dict__.items():
          setattr(base_cfg, k, v) ;
        return base_cfg

