from gazebo_sim.learner import DQNLearner ;
import sys ;
import numpy as np ;

"""
This class allows some "hacks" for faster and more continual experiments for the Different-Forms-Scenario:
- the fake_inputs flag means that instead of the camera image, the environment creates population-coded arrays 
  for symbol, colro, form, distance to object, and viewing angle
- the external_steering flag ensures that the DNN only processes two actions: stop(0) and  "determine forward direction by external steering"(1).
  Basically, in the latter case the DNN can decide whether we stop, or we continue. The idrecition is not determined byt he DNN but by a 
  non-adaptive controller. Upshot is that a) properties of the object are available directly and invariantzly, without processing vision input
  and b) steering towards the object will happen automatically for all tasks, making the problem much simpler
- we can have three valid combinations: fake_inputs == "yes", external_steering == "yes"|"no" and dake_input =="no", external_steering=="no"
  The latter case is the case with no manipulations whatsoeverm so just full DNN control based on plain camera image
- if we have external steering, we assume that only a few of the DNN outputs are meaningful, the oethers are ignored and receive target values of 0 in learning.
"""
class ModifiedDQNLearner(DQNLearner):

  def __init__(self, n_actions, obs_space, config, **kwargs):
    self.config,_ = self.parse_args(**kwargs) ;
    if self.config.external_steering == "yes":
      DQNLearner.__init__(self, 6, obs_space, config) ;
    else:
      DQNLearner.__init__(self, n_actions, obs_space, config) ;

    # do consistency checks for new params
    # see error message!
    if self.config.fake_inputs == "no" and self.config.external_steering == "yes":
      print("External steering xannot be used with real inputs!!") ;
      sys.exit(0) ;

  def store_transition(self, state, action, reward, new_state, done):
    if self.config.external_steering == "yes":
      DQNLearner.store_transition(self, state, action//3, reward, new_state, done)
    else:
      DQNLearner.store_transition(self, state, action, reward, new_state, done)


  # new steering option: extrinsic --> left/right/straight steering is done by non-learning controller
  # only stop/continue decision is taken by trainable system based on color, form, symbol + dist popcodes
  def choose_action(self, observation):
    if self.config.external_steering == "yes" and self.config.fake_inputs == "yes":
      print(observation.shape)
      print(observation)

      # erase line popcode from observation
      steering_guide = observation[1,:].argmax() ; #index of left 1
      observation[1,:] = 0.0 ;
      
      # in this case, the DNN can just output two actions: stop(0) and continue(1). Just call the superclass method for exploration!
      action,randomly_chosen = DQNLearner.choose_action(self, observation) ; 
    
      # if this action is chosen by exploratrion 
      # -> bei mir ist eine Chance 1/6 mal 3 für jede Action = 18, welche Lampe angehen soll
      if randomly_chosen == True:
        #Action random bestimmen
        action = np.random.choice([0, 1, 2, 3, 4, 5]) ;
      else:
        # -> hier nicht Random, hier holen wir uns die Lampe aus dem Model
        #-> die [0:6] ist weil wir in der post_proceed-Funktion dass alles auf die erstn 6 Stellen machten für die Farben
        ret = (self.invoke_model(np.array([observation]))[0][0:6]) ;
        # Action von model bestimmen 
        # die action hier beinhaltet jetzt nur noch die Farbe. Sonst nix mehr an Daten                                        
        action = np.argmax(ret) ; 

      #print("                                        ---- ", action )
      #Left Forward Right

      # Hier bauen wir wieder alles zurück auf unsere ursprünglichen 18 Actionen 0 (included) to 17 (included)
      if steering_guide  < 7:
        return (action*3)+0, randomly_chosen ; # left
      elif steering_guide > 10:
        return (action*3)+1, randomly_chosen ; # right
      else:
        return (action*3)+2, randomly_chosen ; # straight
          
        """
            to right                    to left
        [0 0 0 0 0 0 | 0 0 0 || 0 0 0 | 0 0 0 0 0 0]
                   5                   12
        """      
          
    else:
      ##Steerring wird vom dqn erlernt
      return DQNLearner.choose_action(self, observation) ;


  def define_base_args(self, parser):
    DQNLearner.define_base_args(self, parser) ;
    parser.add_argument("--fake_inputs", type = str, required = False, default = "no") ;
    parser.add_argument("--external_steering", type = str, required = False, default = "no") ;

  """
  # This is called during learning, after having drawn a batch of transitions from the rewplay buffer
  # Here, we post-process these transitions: 1) erase angle representation from fake inputs (taken care of by automatic steering)
  # and 2) re-code targets to 0  to 5 for each lamp color
  def post_process_buffer(self, states, actions, rewards, states_, terminal):
    if self.config.fake_inputs == "yes":
      if self.config.external_steering == "yes":
        #states[:,2,:] = 0.0 ; # erase angle popcode, do we need to do that really?
        # re-code actions so as to give the right popcodes target values for DNN
        # Hier nach sind nur noch actions im Bereich von 0-5 gesetzt
        actions[:] = np.array([(a // 3) for a in actions]) ;
        print(actions)
  """
        
  """        
         0  1  2 => 0
         3  4  5 => 1
         6  7  8 => 2
         9 10 11 => 3
        12 13 14 => 4
        15 16 17 => 5        
        
  """        
  








































