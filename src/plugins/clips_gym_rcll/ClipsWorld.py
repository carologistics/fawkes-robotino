"""/***************************************************************************
 *  ClipsWorld.py -
 *
 *  Created:
 *  Copyright
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
  *  it under the terms of the GNU General Public License as published by
  *  the Free Software Foundation; either version 2 of the License, or
  *  (at your option) any later version.
  *
  *  This program is distributed in the hope that it will be useful,
  *  but WITHOUT ANY WARRANTY; without even the implied warranty of
  *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  *  GNU Library General Public License for more details.
  *
  *  Read the full text in the LICENSE.GPL file in the doc directory.
  */"""

import sys
from tokenize import String

import libfawkes_clips_gym
import clips_gym_rcll

import time
import numpy as np
import gym
from gym import spaces
import inspect
import ast
import pandas as pd
from itertools import product
from stable_baselines3.common.monitor import ResultsWriter
import time
from typing import List
import threading

def expand_grid(dictionary):
   return pd.DataFrame([row for row in product(*dictionary.values())], 
                       columns=dictionary.keys())


#def generateActionSpace(clips_gym_rcll):
		


class ClipsWorld(gym.Env):
  """
  Custom Environment that follows gym interface.
  This is a simple env where the agent must learn to go always left. 
  """
  # Because of google colab, we cannot implement the GUI ('human' render mode)
  metadata = {'render.modes': ['console']}
  # Define constants for clearer code
  #LEFT = 0
  #RIGHT = 1

  def __init__(self, agent_name=""):
    super(ClipsWorld, self).__init__()

    #generate action space
    print("ClipsWorldRCLL init: before generateActionSpace")
    p = clips_gym_rcll.ClipsGymRCLLThread.getInstance()
    action_space = p.generateActionSpace()
    print("ClipsWorldRCLL init: after generateActionSpace\n action_space = ", action_space)
    """ 
    #-----------
    #Get Goal Class list
    goalClasses = p.getGoalClassList()
    print("ClipsWorld: goalClasses: ", goalClasses)
    #for(std::string goalClass: goalClasses)
    partial = {}

    action_space_2=[];
    
    #goalClass = "TOWER-C1"
    for goalClass in goalClasses:
      #get key value map of param-name and param-type (buttom - block)
      mapParamNameType =  p.getParamsNameTypeMapOfGoal(goalClass)
      print("clipsWorld: mapParamNameType ", mapParamNameType)

      if not mapParamNameType:
        c = (goalClass + "#").replace(" ","")
        p.log('GoalClass without params: ' + c)
        action_space_2 += c
        continue;


      for pair in sorted(mapParamNameType.items()):
        #print(x)
      #For each param of the goal do:
      #for key, value in mapParamNameType:
        #print("Key: {} Value:{}", key, value)
        #{buttom#a, buttom#b,...}	
        
        #paramNameDOComb = p.getParamNameDomainObjectsComb(key, value);
        paramNameDOComb = p.getParamNameDomainObjectsComb(pair[0], pair[1]) #x,mapParamNameType[x])
        #print(paramNameDOComb)
        #partial[x]=	paramNameDOComb
        partial[pair[0]] = paramNameDOComb
      
      df = expand_grid(partial)
      df = df.reindex(sorted(df.columns), axis=1)
      #print(df)
      df.insert(0,'Class', goalClass)
      #print("DataFrame: ", df)
      x = df.to_string(header=False,index=False,index_names=False).split('\n')
      vals = ['#'.join(ele.split()) for ele in x]
      #print(vals)
      action_space_2 += vals
      
    #-----------------------------------------
    p.log('Actionspace: '+' '.join(action_space_2[:10])+ ' '.join(action_space_2[-10:]))
    p.log('Actionspace size: '+str(len(action_space_2)))
    action_space = action_space_2
    """
    #print("ClipsWorld init: after generateActionSpace\n action_space = ", action_space)

    #generate observation space
    print("ClipsWorldRCLL init: before generateObservationSpace")
    obs_space = p.generateObservationSpace()

    #-----------------------------------------
    

    obs_space_2=[];
    
    # Key: predicateName Value:ParamName-Type-Dict
    domainPredicatesDict = p.getDomainPredicates()
    # predicate = "ontable"
    for predicate in domainPredicatesDict:
      predicateParamsObjectComb = {}
      #print("ClipsWorld: predicate ", predicate)
      #get key value map of param-name and param-type (buttom - block)
      mapParamNameType =  domainPredicatesDict[predicate]
      #print("clipsWorld: mapParamNameType ", mapParamNameType)
      if not mapParamNameType:
        continue

      for x in mapParamNameType:
        #For each param of the predicate do:
        #{buttom#a, buttom#b,...}	
        #paramNameDOComb = p.getParamNameDomainObjectsComb(x,domainPredicatesDict[predicate][x]) #mapParamNameType[x]);
        paramNameDOComb = p.getDomainObjects(mapParamNameType[x])
        if not paramNameDOComb:
          continue
        #print(paramNameDOComb)
        predicateParamsObjectComb[x]=	paramNameDOComb
        break
      
      #print(predicateParamsObjectComb)
      obs_df = expand_grid(predicateParamsObjectComb)
      #print(obs_df)
      #obs_df = obs_df.reindex(sorted(obs_df.columns), axis=1)
      #print(df)
      obs_df.insert(0,'(','(')
      obs_df.insert(len(obs_df.columns),')',')')
      obs_df.insert(0,'Predicate', predicate)
      #print(obs_df)
      obs_str = obs_df.to_string(header=False,index=False,index_names=False).split('\n')
      vals = ['#'.join(ele.split()) for ele in obs_str]
      vals = [w.replace('#(#','(') for w in vals]
      vals = [w.replace('#)',')') for w in vals]
      obs_space_2 += vals

      obs_space = obs_space_2
    #-----------------------------------------
    p.log('Observationspace: '+' '.join(obs_space_2[:10]))
    p.log('Observationspace size: '+str(len(obs_space_2)))

    #print("ClipsWorld init: after generateObservationSpace\n obs_space = ", obs_space_2)

    sorted_actions = sorted(set(action_space)) #action_space_as_string_array))
    sorted_obs = sorted(set(obs_space))#obs_space_as_string_array))

    #action dict: key: number; value: goal
    set_keys = range(0,len(sorted_actions))
    self.action_dict = dict(zip(set_keys, sorted_actions))
    #inv_action_dict: key: goal; value number
    self.inv_action_dict = (dict (zip (sorted_actions, set_keys)))

    #obs dict: key: number; value: facts
    set_keys_obs = range(0,len(sorted_obs))
    self.obs_dict = dict(zip(set_keys_obs, sorted_obs))
    #inv_obs_dict: key: facts; value: number
    self.inv_obs_dict = (dict (zip (sorted_obs, set_keys_obs)))
    
    # Define action and observation space
    # They must be gym.spaces objects
    # Example when using discrete actions, we have two: left and right
    self.n_actions = len(sorted_actions)
    self.action_space = spaces.Discrete(self.n_actions)
    
    # The observation will be the coordinate of the agent
    # this can be described both by Discrete and Box space
    self.n_obs = len(sorted_obs)
    self.observation_space = gym.spaces.Box(0, 1, (self.n_obs,))
    self.team_points = 0
    self.in_reset = False
    self.first_after_reset = False
    #logging
    self.t_start = time.time()
    self.results_writer = ResultsWriter()
    p.log(f"MonitorWriter: {self.results_writer.__dir__}")
    self.rewards: List[float] = []
    self.episode_returns: List[float] = []
    self.episode_lengths: List[int] = []
    self.episode_times: List[float] = []
    self.total_steps = 0
    self.point_file_path = "game-points_"+agent_name+".txt"

    try:
      self.state = np.zeros(self.n_obs)
      print("self.state: ", self.state)
    except:
      print("State problem")

    #TODO add gametime
    

  def reset(self, seed = None, options = None):
    """
    Important: the observation must be a numpy array
    :return: (np.array) 
    """
    print("ClipsWorldRCLL: start reset function")
    print("ClipsWorldRCLL: reset: before get ClipsGymRCLLThread instance")
    p = clips_gym_rcll.ClipsGymRCLLThread.getInstance()
    if self.team_points != 0:
      self.team_points = p.getGamePointsForTeam("CYAN")
      with open(self.point_file_path, "a+") as f:
        f.write(str(self.team_points)+"\n")
    self.team_points = 0
    #print("NOT IMPLEMENTED ",inspect.currentframe().f_code.co_name)
    self.first_after_reset = True
    result = p.resetCX()
    print("ClipsWorldRCLL: reset: Finished resetCX ")

    print("ClipsWorldRCLL: reset: call create_rl_env_state_from_facts")
    fact_string = p.create_rl_env_state_from_facts()
    raw_facts = ast.literal_eval(fact_string)
    state = self.get_state_from_facts(raw_facts)
    
    self.rewards = []
    self.needs_reset = False
    p.log("ClipsWorldRCLL: end reset function")
    #Optional reset information, not used here
    info = {}
    time.sleep(0.1)
    self.in_reset = False
    return np.array(state).astype(np.int_), info
    #state #

  def logOnEpisodeEnd(self):
    #observation, reward, done, info = self.env.step(action)
    
    self.needs_reset = True
    ep_rew = sum(self.rewards)
    ep_len = len(self.rewards)
    p = clips_gym_rcll.ClipsGymRCLLThread.getInstance()
    p.log(f"\n\nClipsWorldRCLL: logOnEpisodeEnd Reward:{ep_rew} Length: {ep_len}\n\n")
    ep_info = {"r": ep_rew, "l": ep_len, "t": round(time.time() - self.t_start, 6)}
    #for key in self.info_keywords:
    #    ep_info[key] = info[key]
    self.episode_returns.append(ep_rew)
    self.episode_lengths.append(ep_len)
    self.episode_times.append(time.time() - self.t_start)
    #ep_info.update(self.current_reset_info)
    if self.results_writer:
        self.results_writer.write_row(ep_info)
    #info["episode"] = ep_info
      

  def step(self, action):
    current_thread = threading.get_ident()
    print(f"ClipsWorldRCLL: in step function (Thread {current_thread})")

    goal = self.action_dict[action]
    print(f"ClipsWorldRCLL: Before getInstance (Thread {current_thread})")
    p = clips_gym_rcll.ClipsGymRCLLThread.getInstance()
      
    p.log(f"ClipsWorldRCLL: step '{action}': '{goal}' Thread {current_thread}")
    if self.first_after_reset:
          p.getAllFormulatedExecutableGoals()
          self.first_after_reset = False
    result = p.step(goal) #+"#")
    print(("ClipsWorldRCLL: p.step result: ", result))
    #print("ClipsWorld: observation ", result.observation)
    p.log(f"ClipsWorldRCLL: info  '{result.info}' Thread {current_thread}")
    p.log(f"ClipsWorldRCLL: reward '{result.reward}'Thread {current_thread}")
    p.log(f"ClipsWorldRCLL: team-points '{result.team_points}'Thread {current_thread}")
    #TODO check action valid (if not done - reward -1) (da durch action masking nur valide actions ausgesucht werden sollten, außer es gibt keine validen mehr)

    # Create observation from clips
    raw_facts = ast.literal_eval(result.observation)#fact_string)
    #print("\nfacts: ", raw_facts)
    state = self.get_state_from_facts(raw_facts)
    print("New env state from facts: ",state)
    """
    time_sec =  p.getRefboxGameTime()
    phase = p.getRefboxGamePhase()

    p.log(f"ClipsWorldRCLL: Time: '{time_sec}' Phase: '{phase}'")
    game_time = 1200 #180 #in sec = normally 1200

    if (phase == 'POST_GAME' or time_sec > game_time+100) and not self.needs_reset:
      p.log(f"ClipsWorldRCLL: Done: due to POST_GAME or TIME")
      # game over (e.g. if over 300 points you might won the game - extra check with refbox necessary / no logic for game extension!)
      done = True
    elif len(self.rewards) >= 149 and sum(self.rewards) < 100:
      p.log(f"ClipsWorldRCLL: Done: due to rewards len over 200 and sum rewards < 100")
      # due to a machine reset some workpieces are lost
      done = True
    else:
      #there are still executable goals (middle of the game)
      done = False
    """

     # Optionally we can pass additional info and a truncation condition, we are not using that for now
    truncated = False
    info = {}
 


    step_reward = result.reward
    if result.info == "Game Over" and not self.in_reset and result.reward == 0:
      step_reward = 0
      info['outcome'] = "Game Over"
      done = True
      self.in_reset = True
    elif self.in_reset:
      info['outcome'] = "RESET"
      while self.in_reset:
        done = False
      
      time.sleep(5)
    else:
      info['outcome'] = result.info.partition("Outcome ")[-1]
      done = False
      self.team_points = result.team_points
    #done = False if len(executableGoals) else True 
    p.log(f"\n\nClipsWorldRCLL: done '{done}' step reward {step_reward} total reward {sum(self.rewards)+step_reward}\n")
  
    
    self.rewards.append(step_reward)
    if done:
      p.log(f"Episode done, Thread {current_thread}")
      self.logOnEpisodeEnd()
    self.total_steps += 1

    return state, step_reward, done, truncated, info
    #return np.array([self.n_obs]).astype(np.int_), reward, done, info

  #Exposes a method called action_masks(), which returns masks for the wrapped env.
  # action_mask_fn: A function that takes a Gym environment and returns an action mask,
  #      or the name of such a method provided by the environment.
  def action_masks(self) -> np.ndarray:
    print("ClipsWorldRCLL: in action_masks")
    # Returns the action mask for the current env. 
    #def mask_fn(env: gym.Env) -> np.ndarray:
    p = clips_gym_rcll.ClipsGymRCLLThread.getInstance()
    p.waitForFreeRobot()
    if self.first_after_reset:
      p.unlockRobot()
      p.waitForFreeRobot()
      self.first_after_reset = False
    executable_goals = p.getExecutableGoalsForFreeRobot()
    print("ClipsWorldRCLL: action_masks executable goals: ")
    import json
    p.log(json.dumps(self.inv_action_dict))

    valid_actions = np.zeros((self.n_actions), dtype=int)
    print("ClipsWorldRCLL: action_masks size: {0} {1}".format(len(valid_actions), valid_actions[:5]))
    for g in executable_goals:
      goal = g.getGoalString()
      p.log("Executable goal: "+goal)
      pos = self.inv_action_dict.get(goal)
      if (pos is not None):
        print(pos)
        valid_actions[pos]=1
    return valid_actions

    def action_masks_exec(self) -> np.ndarray:
      print("ClipsWorldRCLL: in action_masks")
      # Returns the action mask for the current env. 
      #def mask_fn(env: gym.Env) -> np.ndarray:
      p = clips_gym_rcll.ClipsGymRCLLThread.getInstance()
      p.waitForFreeRobot()
      executable_goals = p.getExecutableGoalsForFreeRobot()
      p.unlockRobot()
      print("ClipsWorldRCLL: action_masks executable goals: ")
      import json
      p.log(json.dumps(self.inv_action_dict))

      valid_actions = np.zeros((self.n_actions), dtype=int)
      print("ClipsWorldRCLL: action_masks size: {0} {1}".format(len(valid_actions), valid_actions[:5]))
      for g in executable_goals:
        goal = g.getGoalString()
        p.log("Executable goal: "+goal)
        pos = self.inv_action_dict.get(goal)
        if (pos is not None):
          print(pos)
          valid_actions[pos]=1
      return valid_actions

  def close(self) -> None:
    """
    Closes the environment
    """
    super().close()
    if self.results_writer is not None:
        self.results_writer.close()

  def render(self, mode='console'):
    if mode != 'console':
      raise NotImplementedError()
    # agent is represented as a cross, rest as a dot
    #print("." * self.agent_pos, end="")
    #print("x", end="")
    #print("." * (self.grid_size - self.agent_pos))

  def close(self):
    pass

  def get_state_from_facts(self,obs_f):
    print("ClipsWorldRCLL: in get_state_from_facts function")
    new_state = np.zeros(self.n_obs)
    #print("new state np array")
    #print("Obs space: ", self.obs_dict)
    for f in obs_f:
      if self.inv_obs_dict.get(f) is not None:
        pos = self.inv_obs_dict[f]
        #print(pos)
        new_state[pos]=1
      #else:
        #print(f"No key: '{f}' in dict")
    return new_state

  def set_state_from_facts(self,obs_f):
    print("ClipsWorldRCLL: set_state_from facts")
    new_state = np.zeros(self.n_obs)
    for f in obs_f:
      if self.inv_obs_dict.get(f) is not None:
        pos = self.inv_obs_dict[f]
        new_state[pos]=1
    print("New env state from facts: ",new_state)
    self.state = new_state
  
  def get_state(self):
    return self.state
    #
        #literals =[]
        #for f in obs_f:
            #x = re.split("\(|,|\)",f)
            #print(x)
            #for key, value in self.observations.items():
                #x = key.__str__().replace(":block", '').replace(":robot",'')
                #is_literal = True
                #print (f,x)        
                #if f == x:
                    #literals.append(self._inverted_obs[value])
        #print(literals)
        #self.env.env.set_state(self.get_state().with_literals(literals))

  def getCurrentObs(self):
    print(f"ClipsWorldRCLL: getCurrentObs ")
    p = clips_gym_rcll.ClipsGymRCLLThread.getInstance()
    fact_string = p.create_rl_env_state_from_facts()
    print("ClipsWorldRCLL: fact_string: ", fact_string)
    raw_facts = ast.literal_eval(fact_string)
    print("ClipsWorldRCLL: getCurrentObs facts: ", raw_facts)
    state = self.get_state_from_facts(raw_facts)
    print("ClipsWorldRCLL: getCurrentObs new env state: ",state)
    return state

  def getGoalIdOfAction(self, discrete_action):
    p = clips_gym_rcll.ClipsGymRCLLThread.getInstance()
    p.log(f"ClipsWorldRCLL: getGoalIdOfAction ")
    executableGoals = p.getAllFormulatedExecutableGoals()
    goal_id = p.getGoalIdByString(executableGoals, discrete_action)
    p.log(f"ClipsWorldRCLL: action {discrete_action} maps goal {goal_id} ")
    return goal_id
    