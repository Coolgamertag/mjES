import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib
import torch
from torch import nn
import cma
from es import SimpleGA, CMAES, PEPG, OpenES
import mujoco as mj
from mujoco.glfw import glfw
import os
import math as m
import mediapy as media

device = 'cuda' if torch.cuda.is_available() else 'cpu'

xml_path = '.xml'
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

model = mj.MjModel.from_xml_path(xml_path)
data = mj.MjData(model)
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

def evalLoop(solutions, SIM_DURATION, neuralNetwork, recording):
    n_steps = int(SIM_DURATION / model.opt.timestep)
    fitness_list = []
    stepsPerNNCall = 4.0
    jumpNum = 0
    for i in range(len(solutions)):
        changeParams(solutions[i], neuralNetwork)
        mj.mj_resetData(model, data)
        jumped = False
        for x in range(int(n_steps/stepsPerNNCall)):
            for z in range(int(stepsPerNNCall)):
                mj.mj_step(model, data)
            clock = m.sin(x*.1)
            inputData = torch.from_numpy(np.append(data.sensordata, clock)).type(torch.float32)
            data.ctrl = neuralNetwork(inputData).detach().numpy()
            if data.sensordata[2] > .3:
                jumped = True
                jumpNum += 1
                break
        if jumped or data.sensordata[2] < .01001:
            fitness_list.append(-200)
        else:
            fitness_list.append(abs(m.sqrt((data.sensordata[0]+27)**2+(data.sensordata[1]**2)))*-2)
            
    if recording:
        bestSolution = np.array(fitness_list).argmax()
        changeParams(solutions[bestSolution], neuralNetwork)
        mj.mj_resetData(model, data)
        glfw.init()
        window = glfw.create_window(1200, 900, "Demo", None, None)
        glfw.make_context_current(window)
        glfw.swap_interval(1)
        mj.mjv_defaultCamera(cam)
        mj.mjv_defaultOption(opt)
        cam.azimuth = 90.0 ; cam.elevation = -45.0 ; cam.distance =  2.0
        cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])
        scene = mj.MjvScene(model, maxgeom=10000)
        context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)
        for x in range(SIM_DURATION*60):
            time_prev = data.time

            while (data.time - time_prev < 1.0/60.0):
                mj.mj_step(model, data)
                clock = m.sin(x*.1)
                inputData = torch.from_numpy(np.append(data.sensordata, clock)).type(torch.float32)
                data.ctrl = neuralNetwork(inputData).detach().numpy()

            # get framebuffer viewport
            viewport_width, viewport_height = glfw.get_framebuffer_size(
                window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            # Update scene and render
            mj.mjv_updateScene(model, data, opt, None, cam,
            mj.mjtCatBit.mjCAT_ALL.value, scene)
            mj.mjr_render(viewport, scene, context)

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()
        glfw.terminate()



    return fitness_list
        

class controlModel(nn.Module):
    def __init__(self, input_features, output_features, hidden_units=8):
        super().__init__()
        self.layer_stack = nn.Sequential(
            nn.Linear(input_features, hidden_units),
            nn.Tanh(),
            nn.Linear(hidden_units, hidden_units*2),
            nn.ReLU(),
            nn.Linear(hidden_units*2, hidden_units),
            nn.ReLU(),
            nn.Linear(hidden_units, output_features),
            nn.Tanh()
        )
    
    def forward(self, x):
        return self.layer_stack(x)

def changeParams(params, model):
    state_dict = model.state_dict()
    i=0
    for name, param in state_dict.items():
        list = []
        for z in param:
            if z.dim() > 0:
                subList = []
                for x in z:
                    subList.append(params[i])
                    i+=1
                list.append(subList)
            else:
                list.append(params[i])
                i+=1

        transformed_param = torch.tensor(list)
        param.copy_(transformed_param)

def getModelInfo():
    return len(data.sensordata) + 1, len(data.ctrl)

NUM_CLASSES, NUM_FEATURES = getModelInfo()
SIM_DURATION = 12

controlMod = controlModel(NUM_CLASSES,
                            NUM_FEATURES,
                            hidden_units=16)
    
def count_parameters(model):
    return sum(p.numel() for p in model.parameters() if p.requires_grad)
NPARAMS = count_parameters(controlMod)
NPOPULATION = 101
MAX_ITERATION = 100000

def test_solver(solver, model):
  recording = True
  history = []
  model.train()
  for j in range(MAX_ITERATION):
    solutions = solver.ask()
    fitness_list = np.zeros(solver.popsize)
    fitness_list = evalLoop(solutions, SIM_DURATION, model, recording)
    solver.tell(fitness_list)
    result = solver.result() # first element is the best solution, second element is the best fitness
    history.append(result[1])
    if (j+1) % 1 == 0:
      print("fitness at iteration", (j+1), result[1])
    if (j+1) % 25 == 0:
        recording = True
    else:
        recording = False
  print("local optimum discovered by solver:\n", result[0])
  print("fitness score at this local optimum:", result[1])
  return history

# defines OpenAI's ES algorithm solver. Note that we needed to anneal the sigma parameter
# this version turns on antithetic sampling. It doesn't really help, and sometimes hurts performance.
oes_antithetic = OpenES(NPARAMS,            # number of model parameters
                 sigma_init=0.5,            # initial standard deviation
                 sigma_decay=0.999,         # don't anneal standard deviation
                 learning_rate=0.03,         # learning rate for standard deviation
                 learning_rate_decay=1.0,   # annealing the learning rate
                 popsize=NPOPULATION+1,     # population size
                 antithetic=True,           # whether to use antithetic sampling
                 weight_decay=0.00,         # weight decay coefficient
                 rank_fitness=False,        # use rank rather than fitness numbers
                 forget_best=False)

oes_antithetic_history = test_solver(oes_antithetic, controlMod)