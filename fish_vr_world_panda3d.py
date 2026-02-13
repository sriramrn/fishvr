"""
TODO:
1) DOCUMENTATION
2) Known limitations:
    a) Stimulation zones can only be defined as circles of a specified radius
3) NOTE: Scaling of coordinates and distances for optic flow and stimulation outputs
   is not done when they are not used (i.e. when self.optic_flow or self.stimulation is False)

DONE:
1) Save trial info ------ json format, add fields as necessary
2) Separate trial index and trial iteration. 
    Potential bug in counting. Could not reproduce. Fixed by blocking trial index update during intertrials? That was likely it. Hasn't happened since then
    a) move to next index only if a certain percentage of current trial has been completed
    b) wait for keyboard input to initiate experiment and move forward from the first trial
    c) Optional manual input to move ahead from current trial
3) Print trial info during experiment ------ add info as needed
4) Forcibly end 2P acquisition while exiting VR
5) Save data ------ save VR data as one long continuous block which can be chunked later. Data saved as csv
6) Make it optional to trigger 2P acquisition
7) Modes of triggering 2P : once every trial or once every experiment
8) Resize screens and aspect ratio to match VR projectors
9) Fix trial start bug where teleportation occurs the next frame after fade ------ fixed by updating x, y and heading when new scene is loaded 
10) Add 'location' trigger for pulsed stimulation in addition to 'time', 'crossing' and 'zone' modes
11) Load parameters from config file
    a) Initialize and setup panda3D in code and not from file
    b) load experiment specific parameters from file
    c) save a copy of loaded parameters in the trial info file
    d) Take a call on how to do 
        i) repetitions of trials
        ii) deal with trial index exceeding trial count in param file
12) Implement TV cards to show landmarks/movies
    ------ Current method for mirroring landmarks may have unknown issues but works in a limited context
13) Implement collision handling
    ------ Collision checks with complex visible geometries slows down frame rate
14) Implement UDP communication with tail tracking code 
        a) Clear out all data in the buffer when reading ------ Done
        b) dtheta and dvelocity will be zero when no values are received from the tail tracking code.
           This will result in jerky update to position and heading but should ideally not occur when
           panda3D framerate is lower than the tail tracking frame rate. 
           udp sync status is saved in csv to a) keep log and align VR with high speed video
                                              b) assess performance and find processing bottlenecks
                                              c) find a working set of parameters (frame rates and timeouts for serial and udp communication)
15) Handle overwriting of data (easy fix but must be done)
    ------ User input requested to overwrite and proceed or abort
16) Optimize tracking parameters for speed. A workable set of parameters has been found. 
    Parameters on laptop:
                         a) VR fps: 60Hz
                         b) Tail tracking tested with video file. 100-150 fps should work, test with camera
                         c) Teensy sync timeout of 0.004 seconds
                         b) UDP timeout of 0.001 seconds and UDP buffer size of 64 bytes
17) Implement optic flow stimulus delivery and trialwise global gain setting
18) Implement screen splitting based on tank geometry to allow different configurations
19) Displacement based estimation of trial completion does not work well under all conditions, for example in a VR with optic flow or mazes
    Use a list of coordinates to determine trial completion. Check minimum distance of fish from end coordinates. 
20) Add velocity, heading and inactivity flag to log file
21) Distances are scaled based on the scale parameter estimated using the calibration protocol (see scale_calibration.py)
22) Added multichannel stimulation support for upto 8 digital outputs
23) Update VR position and heading only when fish is swimming, as detected by the tail tracking code. 
        a) self.update_only_if_swimming must be set to True or False in this code, not in the config file. True by default
        b) 'Swimming' status is received from the tail tracking code via UDP along with velocity and heading
        c) Set curvature threshold correctly in the tail tracking code
        d) Appears to work better than thresholding velocity and heading. 
            - In practice, set the threshold values in the tail tracking code to 0. and set the
              curvature threshold such that swimming is detected correctly, as indicated in the gui
24) Movie onset and start of delay timer are saved to the log encoded as an integer. 
    - Use the function included in this code to decode to a boolean list.
    - The booleans are associated with the background texture which is removed to reveal the movie.
      The indices therefore correspond to the indices of the background textures and not the movie texture itself.
25) Added a manual 2p acquisition start trigger. Press 't' to trigger acquisition. 
    - Can be used in the event that acquisition needed to be stopped and restarted manually within a VR experiment.
    - Generating the trigger manually does not affect trial progression.
26) Log UDP sync failures to csv file 
"""

import sys
import socket
import struct
import builtins
from direct.showbase.ShowBase import ShowBase
from direct.task import Task
from direct.interval.LerpInterval import LerpColorScaleInterval
from direct.interval.IntervalGlobal import LerpFunc
from panda3d.core import AmbientLight, DirectionalLight, Vec4, Fog, ClockObject, loadPrcFileData
from panda3d.core import CardMaker, TextureStage
from panda3d.core import CollisionHandlerPusher, CollisionNode, CollisionSphere, CollisionTraverser, BitMask32
import teensytimer
import numpy as np
import json
import csv
from datetime import datetime
import configparser
import tkinter as tk
from tkinter import filedialog
root = tk.Tk()
root.withdraw()
root.update()


######################
# Initialize panda3D #
######################

setupfile = None
if setupfile is None:
    setupfile = filedialog.askopenfilename(initialdir='setup/', title='Select VR setup file', filetypes=[('INI files', '*.ini')])

setup = configparser.ConfigParser()
setup.read(setupfile)

framerate = setup.getint('params', 'framerate')                 # Will apply only when syncvideo is False
syncvideo = setup.getboolean('params', 'syncvideo')             # Decouple spinCameraTask from monitor refresh (False sets user specified fps, True clamps to monitor refresh)
use_timer = setup.getboolean('params', 'use_timer')             # Use teensy timer for synchronization
savepath = setup.get('params', 'savepath')                      # Save data to this directory. Can be changed in file dialog
configpath = setup.get('params', 'configpath')                  # Path to experiment configuration files. Can be changed in file dialog

serial_port = setup.get('params', 'serial_port')                # Serial port for teensy timer
serial_timeout = setup.getfloat('params', 'serial_timeout')     # Time to wait for response from teensy
udp_ip = setup.get('params', 'udp_ip')                          # Address to receive UDP message from tail tracking code
udp_port = setup.getint('params', 'udp_port')                   # UDP port
udp_buffer_size = setup.getint('params', 'udp_buffer_size')     # Buffer size for UDP communication. Set large enough to hold multiple values
udp_timeout = setup.getfloat('params', 'udp_timeout')           # Timeout to listen for a UDP message before proceeding

projection_width = setup.getint('params', 'projection_width')   # combined width of screen across 3 projectors
projection_height = setup.getint('params', 'projection_height') # height of screen
window_origin_x = setup.getint('params', 'window_origin_x')     # panda3D window position
window_origin_y = setup.getint('params', 'window_origin_y')

mainlight_rgba = [float(x) for x in setup.get('params', 'mainlight_rgba').split(',')]
mainlight_hpr = [float(x) for x in setup.get('params', 'mainlight_hpr').split(',')]
ambient_rgba = [float(x) for x in setup.get('params', 'ambient_rgba').split(',')]

ncameras = setup.getint('params', 'ncameras') # number of projectors                   
projector_aspect_ratio = (projection_width/ncameras)/projection_height
projector_fov = setup.getfloat('params', 'projector_fov') # field of view of each projector
projector_hpr = []  # angle, pitch, roll of each projector (measure with angle finder, adjust as needed to fix misalignments)
display_region = [] # display region for each projector in relative coordinates
hpr_keys = ['projector{}_hpr'.format(x) for x in range(ncameras)]
dr_keys = ['display_region{}'.format(x) for x in range(ncameras)]

for i in range(ncameras):

    hpr = setup.get('params', hpr_keys[i]).split(',')
    hpr = [float(x) for x in hpr]
    projector_hpr.append(hpr)

    dr = setup.get('params', dr_keys[i]).split(',')
    dr = [float(x) for x in dr]
    display_region.append(dr)

globalClock = ClockObject.getGlobalClock()
globalClock.setMode(ClockObject.MLimited)
globalClock.setFrameRate(framerate) 

loadPrcFileData('', 'win-size {} {}'.format(projection_width, projection_height))
loadPrcFileData('', 'win-origin {} {}'.format(window_origin_x, window_origin_y))
loadPrcFileData('', 'sync-video {}'.format(syncvideo))
loadPrcFileData('', 'threading-model Cull/Draw')
loadPrcFileData('', "background-color 0 0 0")

######################
#        END         #
######################


class ZebrafishVirtualReality(ShowBase):

    def __init__(self, configpath=configpath, save=True, savepath=savepath, use_timer=use_timer, 
                 serial_port=serial_port, serial_timeout=serial_timeout, udp_ip=udp_ip, 
                 udp_port=udp_port, udp_buffer_size=udp_buffer_size, udp_timeout=udp_timeout,
                 ncameras=ncameras, projector_hpr=projector_hpr, projector_fov=projector_fov, 
                 display_region=display_region, mainlight_rgba=mainlight_rgba, mainlight_hpr=mainlight_hpr,
                 ambient_rgba=ambient_rgba):

        self.configfile = filedialog.askopenfilename(initialdir=configpath, title='Select config file', filetypes=[('JSON files', '*.json')])
        if len(self.configfile) == 0:
            print('No configuration selected, session aborted')
            sys.exit()

        self.save = save
        self.savepath = savepath

        if self.save:
            logfile = filedialog.asksaveasfilename(initialdir=savepath, initialfile='_vrlog', title='Save log file as', defaultextension='.csv', filetypes=[('csv files', '*.csv')])
            if len(logfile) == 0:
                print('No log file specified, will continue without saving')
                self.save = False
            else:
                jsonfile = logfile.split('.')[0] + '_vrinfo.json'
                date_and_time = datetime.now().strftime("%y%m%d_%H:%M:%S")

                self.infofile = open(jsonfile, 'w', encoding='utf-8')
                self.infofile.write("{\"timestamp\": \"%s\",\n" % date_and_time)
                self.infofile.write("\"trials\":[\n")

                self.datafile = open(logfile, 'w', encoding='utf-8',  newline='')
                self.logger = csv.writer(self.datafile, delimiter=',', quotechar='|', quoting=csv.QUOTE_MINIMAL)
                self.logger.writerow(['trial', 'trial index', 'trial onset counter', '2p framecount', 'timestamp', 'xpos', 'ypos', 'heading', 
                                      'velocity', 'dheading', '2p state', 'optic flow state', 'optic flow direction', 'optic flow velocity', 
                                      'stim state', 'stim mode pulsed', 'tail tracking framecount', 'vr gain', 'inactivity timer', 'is swimming',
                                      'movie timer started', 'movie revealed', 'udp sync failed'])
                
        ShowBase.__init__(self)
        self.distance_to_nearest_end_point = np.inf
        self.trial = 0
        self.trial_onset_counter = 0
        self.repcount = 0
        self.trial_index = 0
        self.force_progress = False

        self.loadConfig()

        self.frame_onset_time = 0.
        self.frame_dt = 0.
        self.XPos = 0.
        self.YPos = 0.
        self.ZPos = 0.
        self.heading = 0.
        self.prevX = 0.
        self.prevY = 0.
        self.inactivity_timer = 0.
        self.inactivity_triggered_flow_timer = 0.
        self.inactivity_triggered_flow_state = False

        self.use_timer = use_timer
        self.serial_port = serial_port

        if self.use_timer:
            self.teensy = teensytimer.TeensyTimer(self.serial_port)
            self.teensy.receive_timeout = serial_timeout
            self.teensy.open()

        self.state = "intertrial" # "trial" or "intertrial"
        self.trial_complete = False
        self.end_trial = False
        self.trial_end_timer = 0.
        self.start_trial_end_timer = False
        self.state_onset_time = 0.
        self.initialized = False
        self.start_2p = False
        self.end_2p = False
        self.optic_flow_state = False
        self.flow_in_world = False
        self.framecount_2p = 0.
        self.twopstate = False
        self.end_vr = False
        self.stim_state_8bitcode = 0.
        self.stim_mode_pulsed_8bitcode = 0.

        #Fade in and out while teleporting
        self.fade_timer = 0.
        self.fade_out = False
        self.fade_in = False

        #Landmarks
        self.landmark_nodes = [] #container to hold all landmark nodes
        self.landmark_nodes_mirrored = [] #container to hold mirrored copies of landmarks on the other side of a corridor
        self.textures = []

        #UDP port and socket for communicating with tail tracking code
        self.udp_ip = udp_ip
        self.udp_port = udp_port
        self.udp_buffer_size = udp_buffer_size
        self.udp_timeout = udp_timeout #setting this parameter to 0 is equivalent to non-blocking mode
        self.tail_tracking_framecount = 0
        self.udpfailed = False

        #Variables from tail tracking code or keyboard
        self.velocity = 0.
        self.theta = 0.
        self.is_swimming = False
        self.update_only_if_swimming = True # set to True to update VR position and heading only when fish is considered to be swimming as per the tail tracking code
                                            # this parameter is not in the setup file or config file and must be set here in the code

        """
        ######################
        Definition of movement 
        ######################
        """
        builtins.base.disableMouse()
        builtins.base.buttonThrowers[0].node().setKeystrokeEvent('keystroke')
        
        # -*- starting position
        builtins.base.camera.setPos(*self.init_position)

        self.taskMgr.add(self.spinCameraTask, 'SpinCameraTask')

        if self.use_keyboard:
            self.keyMap = {
            'forward': 0, 'backward': 0, 'right': 0, 'left': 0}
            self.accept('arrow_up', self.setKey, ['forward', True])
            self.accept('arrow_down', self.setKey, ['backward', True])
            self.accept('arrow_right', self.setKey, ['right', True])
            self.accept('arrow_left', self.setKey, ['left', True])
            # Detect when keys are released:
            self.accept("arrow_up-up", self.setKey, ["forward", False])
            self.accept('arrow_down-up', self.setKey, ['backward', False])
            self.accept("arrow_right-up", self.setKey, ["right", False])
            self.accept("arrow_left-up", self.setKey, ["left", False])
        else:
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP socket
            self.udp_socket.bind((self.udp_ip, self.udp_port))
            # self.udp_socket.setblocking(0) # using settimeout(0.0) is equivalent to setblocking(0) 
            self.udp_socket.settimeout(self.udp_timeout)


        # self.accept('w', self.updateTrialIndexWithKey)
        self.accept('keystroke', self.updateTrialIndexWithKey)
        self.accept('escape', self.exitVR)


        """
        ################
        Screen-Splitting 
        ################
        """
        builtins.base.camLens.setFov(projector_fov) # originally 40, but tank walls angles indicate projectors are at 60 degrees to each other
        builtins.base.camLens.setNear(0.001)
        builtins.base.camLens.setFar(200)
        builtins.base.setAspectRatio(projector_aspect_ratio)

        builtins.base.camNode.setActive(0)
        
        [builtins.base.makeCamera(builtins.base.win, displayRegion=display_region[i], lens=builtins.base.camLens) for i in range(ncameras)]
        [builtins.base.camList[i+1].setHpr(*projector_hpr[i]) for i in range(ncameras)]

        self.camlist = builtins.base.camList

        mainLight = DirectionalLight("main light")
        mainLight.setColor(Vec4(*mainlight_rgba))
        self.mainLightNodePath = self.render.attachNewNode(mainLight)
        self.mainLightNodePath.setHpr(*mainlight_hpr)
        self.render.setLight(self.mainLightNodePath)

        ambientLight = AmbientLight("ambient light")
        ambientLight.setColor(Vec4(*ambient_rgba))
        self.ambientLightNodePath = self.render.attachNewNode(ambientLight)
        self.render.setLight(self.ambientLightNodePath)
        self.render.setShaderAuto()

        self.setupScene()


    def loadConfig(self):
        with open(self.configfile, 'r') as f:
            self.expconfig = json.load(f)        
        f.close()

        self.ntrials = len(self.expconfig['trials'])
        self.setParamsForCurrentTrial()


    def setParamsForCurrentTrial(self):

        self.trialconfig = self.expconfig['trials'][self.trial_index]

        self.trial_id                           = self.trialconfig['trial_id']
        self.repeats                            = self.trialconfig['repeats']
        self.use_keyboard                       = self.trialconfig['use_keyboard']
        self.keyboard_gain                      = self.trialconfig['keyboard_gain']
        self.keyboard_heading_increment         = self.trialconfig['keyboard_heading_increment']
        self.vr_gain                            = self.trialconfig['vr_gain']
        self.enable2d                           = self.trialconfig['enable2d']
        self.invert_turn_direction              = self.trialconfig['invert_turn_direction']
        self.trial_end_coords                   = self.trialconfig['trial_end_coords']
        self.trial_start_coord                  = self.trialconfig['trial_start_coord']
        self.distance_threshold                 = self.trialconfig['distance_threshold']
        self.trial_end_delay                    = self.trialconfig['trial_end_delay']
        self.model                              = self.trialconfig['model']
        self.init_position                      = self.trialconfig['init_position']
        self.init_hpr                           = self.trialconfig['init_hpr']
        self.scale                              = self.trialconfig['scale']
        self.intertrial_duration                = self.trialconfig['intertrial_duration']
        self.max_trial_duration                 = self.trialconfig['max_trial_duration']
        self.repeat_incomplete_trials           = self.trialconfig['repeat_incomplete_trials']
        self.inactivity_threshold               = self.trialconfig['inactivity_threshold']
        self.max_inactive_duration              = self.trialconfig['max_inactive_duration']
        self.inactivity_leads_to                = self.trialconfig['inactivity_leads_to']
        self.inactivity_triggered_flow_duration = self.trialconfig['inactivity_triggered_flow_duration']

        if self.invert_turn_direction:
            self.heading_multiplier = -1
        else:
            self.heading_multiplier = 1

        #imaging triggers
        self.do2p               = self.trialconfig['do2p']
        self.continuous2p       = self.trialconfig['continuous2p']

        #Fade in and out while teleporting
        self.fade_duration      = self.trialconfig['fade_duration']

        #Stimulation triggers
        self.stimulation            = self.trialconfig['stimulation']
        self.nstimchannels          = self.trialconfig['nstimchannels']
        self.stimulation_mode       = self.trialconfig['stimulation_mode']
        self.stim_intensitiy        = self.trialconfig['stim_intensity']
        self.stim_coord             = self.trialconfig['stim_coord']
        self.crossing_direction     = self.trialconfig['crossing_direction']
        self.stim_radius            = self.trialconfig['stim_radius']
        self.stim_onset_time        = self.trialconfig['stim_onset_time']
        self.stim_duration          = self.trialconfig['stim_duration']
        self.stim_pulse_duration    = self.trialconfig['stim_pulse_duration']
        self.use_channel            = self.trialconfig['use_channel']

        if self.stimulation:

            # 8 channel stimulation
            self.stim_state = [False]*self.nstimchannels
            self.stim_mode_pulsed = [False]*self.nstimchannels
            self.stim_onset_time_log = [0.]*self.nstimchannels # log of stimulus onset time for each channel for software timing

            if not isinstance(self.use_channel, list):
                self.use_channel = [self.use_channel] * self.nstimchannels

            if isinstance(self.stimulation_mode, str):
                self.stimulation_mode = [self.stimulation_mode] * self.nstimchannels
            
            if not isinstance(self.stim_coord[0], list):
                self.stim_coord = [self.stim_coord] * self.nstimchannels
            
            if isinstance(self.crossing_direction, str):
                self.crossing_direction = [self.crossing_direction] * self.nstimchannels
            
            if isinstance(self.stim_radius, int) or isinstance(self.stim_radius, float):
                self.stim_radius = [self.stim_radius] * self.nstimchannels

            if isinstance(self.stim_onset_time, int) or isinstance(self.stim_onset_time, float):
                self.stim_onset_time = [self.stim_onset_time] * self.nstimchannels

            if isinstance(self.stim_duration, int) or isinstance(self.stim_duration, float):
                self.stim_duration = [self.stim_duration] * self.nstimchannels


            for i in range(self.nstimchannels):
                if self.stimulation_mode[i] == 'location':
                    self.stim_mode_pulsed[i] = True
                elif self.stimulation_mode[i] == 'location_software_timed':
                    self.stim_mode_pulsed[i] = False
                elif self.stimulation_mode[i] == 'crossing':
                    self.stim_mode_pulsed[i] = True
                elif self.stimulation_mode[i] == 'crossing_software_timed':
                    self.stim_mode_pulsed[i] = False
                elif self.stimulation_mode[i] == 'zone':
                    self.stim_mode_pulsed[i] = False
                elif self.stimulation_mode[i] == 'time':
                    self.stim_mode_pulsed[i] = False


        #Optic flow
        self.optic_flow                 = self.trialconfig['optic_flow']
        self.optic_flow_reference_frame = self.trialconfig['optic_flow_reference_frame']
        self.optic_flow_direction       = self.trialconfig['optic_flow_direction']
        self.optic_flow_velocity        = self.trialconfig['optic_flow_velocity']
        self.optic_flow_mode            = self.trialconfig['optic_flow_mode']
        self.optic_flow_coord           = self.trialconfig['optic_flow_coord']
        self.optic_flow_radius          = self.trialconfig['optic_flow_radius']

        #Cue cards for showing landmarks and displaying video
        self.landmark_scale         = self.trialconfig['landmark_scale']
        self.show_landmarks         = self.trialconfig['show_landmarks']
        self.mirror_landmarks       = self.trialconfig['mirror_landmarks']
        self.mirror_distance        = self.trialconfig['mirror_distance']
        self.landmark_number        = self.trialconfig['landmark_number']
        self.landmark_position      = self.trialconfig['landmark_position']
        self.landmark_hpr           = self.trialconfig['landmark_hpr']
        self.landmarks              = self.trialconfig['landmarks']
        self.landmark_flag          = self.trialconfig['landmark_flag']
        self.hide_movie             = self.trialconfig['hide_movie']
        self.movie_onset_distance   = self.trialconfig['movie_onset_distance']
        self.movie_onset_delay      = self.trialconfig['movie_onset_delay']
        self.background_texture     = self.trialconfig['background_texture']

        #Collision handling
        self.handle_collisions  = self.trialconfig['handle_collisions']
        self.collision_radius   = self.trialconfig['collision_radius']
        self.collider_flag      = self.trialconfig['collider_flag']  
        self.hide_collider      = self.trialconfig['hide_collider']

        self.scaleParams()

    
    def scaleParams(self):
        self.trial_end_coords = [[x*self.scale for x in y] for y in self.trial_end_coords]
        self.trial_start_coord = [x*self.scale for x in self.trial_start_coord]
        self.distance_threshold *= self.scale
        self.init_position = [x*self.scale for x in self.init_position]

        if self.show_landmarks:
            self.landmark_position = [[x*self.scale for x in y] for y in self.landmark_position]
            if isinstance(self.mirror_distance, list):
                self.mirror_distance = [x*self.scale for x in self.mirror_distance]
            if self.hide_movie:
                self.movie_onset_distance *= self.scale

        if self.handle_collisions:
            self.collision_radius = self.collision_radius * self.scale

        if self.optic_flow:
            self.optic_flow_coord = [x*self.scale for x in self.optic_flow_coord]
            self.optic_flow_radius *= self.scale

        if self.stimulation:
            if isinstance(self.stim_coord[0], list):
                self.stim_coord = [[x*self.scale for x in y] for y in self.stim_coord]
            else:
                self.stim_coord = [x*self.scale for x in self.stim_coord]

            if isinstance(self.stim_radius, list):
                self.stim_radius = [x*self.scale for x in self.stim_radius]
            else:
                self.stim_radius *= self.scale


    def setupScene(self):
    
        if self.trial > 0:
            self.scene.removeNode()

        # Load the environment model.
        self.scene = self.loader.loadModel(self.model)
        
        # Apply scale and position transforms on the model.
        self.scene.setScale(self.scale)
        
        # rendering related. mostly has no consequence
        self.scene.setTwoSided(True)

        # Reparent the model to render.
        self.scene.reparentTo(self.render)

        self.drawLandmarks() # this function can be called dynamically to toggle landmark visibility using flags    

        # -------------------------------------------------
        # Fog with linear falloff of visibility
        # -------------------------------------------------
        self.fog_color = Vec4(0.7, 0.8, 1.0, 1)
        self.setBackgroundColor(self.fog_color)
        self.fog = Fog("sceneFog")
        self.fog.setMode(Fog.MLinear)
        self.fog.setLinearRange(0.1, 0.2) #start and end distance from camera (in metres), reaches 0 visibility at end distance.
        self.fog.setColor(self.fog_color)
        self.render.setFog(self.fog)


        # set up collision handling
        # doing this before updating position from origin seems to be important
        # collision detection fails if initial position is updated before setting up collision handling
        if self.trial == 0:
            self.initializeColliderObject()

        self.updateCollisionPushers()

        # self.scene.setPos(*self.init_position)  
        builtins.base.camera.setPos(*self.init_position)
        builtins.base.camera.setHpr(*self.init_hpr)
        self.YPos = builtins.base.camera.getY()
        self.XPos = builtins.base.camera.getX()
        self.heading = builtins.base.camera.getH()

        self.formatTrialInfo()

        if self.save:
            if self.trial > 0:
                self.infofile.write(',\n')
            json.dump(self.trialinfo, self.infofile, ensure_ascii=False)
            self.infofile.flush()


    def getNodes(self):
        try:
            self.nodes = dict([(x.getName(), list(x.getPos())) for x in self.scene.getChildren()])
        except:
            self.nodes = []
    

    def formatTrialInfo(self):
        self.getNodes()
        self.trialinfo = {"trial_number" : self.trial, "trial_index" : self.trial_index,
                          "repeat" : self.repcount, "trial_config" : self.trialconfig,
                          "nodes" : self.nodes}
        

    def fadeFog(self, t):
        r = self.fog_color[0] * (1 - t)
        g = self.fog_color[1] * (1 - t)
        b = self.fog_color[2] * (1 - t)
        self.fog.setColor(r, g, b)
            

    def drawLandmarks(self):
        #Landmarks can be redrawn dynamically in spinCameraTask by setting flags accordingly
        #This can be used to introduce closed-loop contigencies for implementing learning assays
        #Hint: 1) use a function to check what to do with landmarks in each loop of spinCameraTask
        #      2) redraw landmarks when they need to be changed
        if self.show_landmarks:
            self.movie_revealed = [False]*self.landmark_number
            self.movie_timer_started = [False]*self.landmark_number
            self.movie_trigger_time = [None]*self.landmark_number
            [x.removeNode() for x in self.landmark_nodes if x is not None]
            [x.removeNode() for x in self.landmark_nodes_mirrored if x is not None]
            cm = CardMaker('card')
            self.landmark_nodes = []
            self.landmark_nodes_mirrored = []
            self.textures = []
            for i in range(self.landmark_number):
                
                tex, lm, lm2 = self.initializeLandmarkNode(cm, self.landmarks[i], self.landmark_flag[i], self.landmark_position[i], 
                                                        self.landmark_hpr[i], self.mirror_landmarks[i], self.mirror_distance[i])

                self.textures.append(tex)
                self.landmark_nodes.append(lm)
                self.landmark_nodes_mirrored.append(lm2)


    def initializeLandmarkNode(self, cardmaker, texture_file, flag, position, hpr, mirror=False, mirror_distance=0):
        lmtex =  self.loader.loadTexture(texture_file)
        lmnode = None
        lmnode2 = None
        if flag:
            lmnode = self.render.attachNewNode(cardmaker.generate())
            lmnode = self.centerPivot(lmnode)
            lmnode.setDepthTest(1)
            lmnode.setDepthWrite(1)
            lmnode.setColor(Vec4(1, 1, 1, 1))
            lmnode.setPos(tuple(position))
            lmnode.setHpr(tuple(hpr))
            lmnode.setScale(self.landmark_scale)
            lmnode.setTexture(lmtex, 1)

            if mirror:
                pos_mirror = [x+y for x,y in zip(position,[mirror_distance,0,0])]
                h_mirror = [-hpr[0],hpr[1],hpr[2]]

                lmnode2 = self.render.attachNewNode(cardmaker.generate())
                lmnode2 = self.centerPivot(lmnode2)
                lmnode2.setDepthTest(1)
                lmnode2.setDepthWrite(1)
                lmnode2.setColor(Vec4(1, 1, 1, 1))
                lmnode2.setPos(tuple(pos_mirror))
                lmnode2.setHpr(tuple(h_mirror))
                lmnode2.setScale(self.landmark_scale)
                lmnode2.setTexScale(TextureStage.getDefault(),-1,1)
                lmnode2.setTexture(lmtex, 1)

            if texture_file.split('.')[-1] in ['avi','mov','mp4']:
                lmtex.play()
                lmtex.setLoop(True)

        return lmtex, lmnode, lmnode2
    

    def centerPivot(self, cardnode):
        """https://discourse.panda3d.org/t/again-rotation-question/2004/8"""
        pivot=cardnode.getBounds().getCenter()
        parent=cardnode.getParent()
        # create the new parent node under the original parent
        newnode=parent.attachNewNode('theName')
        # bring the new parent to the center
        newnode.setPos(pivot)
        # and relocate the node without changing any of it's transformation
        cardnode.wrtReparentTo(newnode)
        return newnode
    

    def initializeColliderObject(self, radius=0.):

        print(self.collision_radius)
        if self.collision_radius is not None:
            radius = self.collision_radius

        self.cTrav = CollisionTraverser()

        self.collider_node = CollisionNode("fish")

        self.collider_node.addSolid(CollisionSphere(self.XPos, self.YPos, self.init_position[2], radius))
        self.collider_node.setCollideMask(BitMask32.bit(0))
        self.collider_object = builtins.base.camera.attachNewNode(self.collider_node)

        self.pusher = CollisionHandlerPusher()
        self.pusher.addCollider(self.collider_object, builtins.base.camera)


    def updateCollisionPushers(self):
        
        self.cTrav.clearColliders()

        if self.handle_collisions:

            self.collider_node_list = []
            hide = []
            for i in range(len(self.collider_flag)):
                p = self.collider_flag[i]
                matches = list(self.scene.findAllMatches('**/*{}*;+h'.format(p)))
                self.collider_node_list += matches
                hide += [self.hide_collider[i]]*len(matches)

            [x.setCollideMask(BitMask32.bit(0)) for x in self.collider_node_list]       

            self.cTrav.addCollider(self.collider_object, self.pusher)

            [self.collider_node_list[x].hide() for x in range(len(self.collider_node_list)) if hide[x]]


    def int_to_bools(self, n: int, length: int) -> list[bool]:
        """Convert integer to list of bools (MSB first)."""
        """Use this to invert the bools_to_int function for interpreting the logged data"""
        return [(n >> i) & 1 == 1 for i in range(length - 1, -1, -1)]

    
    def bools_to_int(self, bools):
        result = 0
        for b in bools:
            result = (result << 1) | b
        return result
    

    def setKey(self, key, value):
        self.keyMap[key] = value

    
    def updateTrialIndexWithKey(self, keyname):
        
        if self.state == 'trial':

            if keyname == 'w':
                self.force_progress = True
                print("Will progress to next trial regardless of outcome")
            if keyname == 's':
                self.force_progress = False
                print("Will progress only if trial completed")
            if keyname == 'e':
                self.end_trial = True
                self.force_progress = True
                print('Progressing to the next trial')
            if keyname == 'p':
                print(self.XPos, self.YPos, self.heading)
            if keyname == 't':
                self.start_2p = True
                print('2p imaging triggered manually')


    def updateCameraWithKeyboard(self):

        self.velocity = 0
        self.theta = 0

        if self.keyMap["forward"]:
            self.velocity = self.keyboard_gain
            builtins.base.camera.setY(builtins.base.camera, self.velocity * self.frame_dt)

        elif self.keyMap["backward"]:
            self.velocity = -self.keyboard_gain
            builtins.base.camera.setY(builtins.base.camera, self.velocity * self.frame_dt)

        if self.keyMap["right"]:
            self.theta = -self.keyboard_heading_increment * self.heading_multiplier
            builtins.base.camera.setH(builtins.base.camera, self.theta * self.frame_dt)

        elif self.keyMap["left"]:
            self.theta = self.keyboard_heading_increment * self.heading_multiplier
            builtins.base.camera.setH(builtins.base.camera, self.theta * self.frame_dt)


    def updateCameraWithTailTracking(self):
        data, addr, self.udpfailed = None, None, False

        # Read all data in the buffer and clear it out. Most recent values will be used for updating VR
        # Number of packets stuck in the buffer can be minimized by setting a broadcast rate close to the framerate of panda3D
        while True:
            try:
                data, addr = self.udp_socket.recvfrom(self.udp_buffer_size) # buffer size is 24 bytes
            except:
                break
        
        if data is not None:
            self.velocity, self.theta, self.is_swimming, self.tail_tracking_framecount = struct.unpack('>dd?i', data)
            self.velocity = self.velocity * self.vr_gain
            self.theta = self.theta * self.vr_gain * self.heading_multiplier
            if self.update_only_if_swimming:
                self.velocity = self.velocity * self.is_swimming
                self.theta = self.theta * self.is_swimming
        else:
            self.velocity, self.theta, self.udpfailed = 0, 0, True

        builtins.base.camera.setY(builtins.base.camera, self.velocity * self.frame_dt)
        if self.enable2d:
            builtins.base.camera.setH(builtins.base.camera, self.theta * self.frame_dt)


    def updateCameraWithOpticFlow(self):

        if self.optic_flow_state == True:

            if self.optic_flow_reference_frame == 'allocentric':
                current_h = builtins.base.camera.getH() # get current heading
                builtins.base.camera.setH(self.optic_flow_direction) # set heading to the desired optic flow direction
                builtins.base.camera.setY(builtins.base.camera, self.optic_flow_velocity * self.frame_dt)
                builtins.base.camera.setH(current_h) # reset heading to fish heading
                
            if self.optic_flow_reference_frame == 'egocentric':    
                builtins.base.camera.setY(builtins.base.camera, self.optic_flow_velocity * self.frame_dt)        

    
    def euclidean_distance(self, coordinate1, coordinate2):
        distance = np.linalg.norm(np.array(coordinate1) - np.array(coordinate2))
        return distance


    def distance_to_nearest_target(self, reference_coordinate, target_coordinates):
        distance_to_nearest = min([self.euclidean_distance(reference_coordinate, x) for x in target_coordinates])
        return distance_to_nearest
    

    def setOpticFlowState(self):

        if self.state == "trial":

            if self.optic_flow == True:

                if self.optic_flow_mode == "trial" and self.optic_flow_state == False:
                        self.optic_flow_state = True
                        self.flow_in_world = True

                if self.optic_flow_mode == "zone":
                    distance = self.euclidean_distance([self.XPos,self.YPos], self.optic_flow_coord)
                    if distance <= self.optic_flow_radius:
                        self.optic_flow_state = True
                        self.flow_in_world = True
                    else:
                        self.optic_flow_state = False
                        self.flow_in_world = False

            if self.inactivity_leads_to == "flow" and self.flow_in_world == False:
                if self.inactivity_triggered_flow_state == True:
                    self.optic_flow_state = True
                else:
                    self.optic_flow_state = False

        else:
            if self.optic_flow_state:
                self.optic_flow_state = False
            if self.flow_in_world:
                self.flow_in_world = False


    def updateStimState(self):
        
        if self.state == "trial":

            for i in range(self.nstimchannels):

                if not self.use_channel[i]:
                    self.stim_state[i] = False
                    continue

                if self.stimulation_mode[i] == "location":
                    prev_distance = self.euclidean_distance([self.prevX,self.prevY], self.stim_coord[i])
                    distance = self.euclidean_distance([self.XPos,self.YPos], self.stim_coord[i])
                    if prev_distance > self.stim_radius[i] and distance <= self.stim_radius[i]:
                        self.stim_state[i] = True
                    else:
                        self.stim_state[i] = False


                elif self.stimulation_mode[i] == "location_software_timed":
                    prev_distance = self.euclidean_distance([self.prevX,self.prevY], self.stim_coord[i])
                    distance = self.euclidean_distance([self.XPos,self.YPos], self.stim_coord[i])
                    if prev_distance > self.stim_radius[i] and distance <= self.stim_radius[i]:
                        if not self.stim_state[i]:
                            self.stim_onset_time_log[i] = self.frame_onset_time
                            self.stim_state[i] = True
                        if self.stim_state[i] and (self.frame_onset_time - self.stim_onset_time_log[i] >= self.stim_duration[i]):
                            self.stim_state[i] = False
                        

                elif self.stimulation_mode[i] == "crossing":
                    if self.crossing_direction[i] == "x":
                        if self.prevX < self.stim_coord[i][0] and self.XPos >= self.stim_coord[i][0]:
                            self.stim_state[i] = True
                        else:
                            self.stim_state[i] = False

                    elif self.crossing_direction[i] == "y":
                        if self.prevY < self.stim_coord[i][1] and self.YPos >= self.stim_coord[i][1]:
                            self.stim_state[i] = True
                        else:
                            self.stim_state[i] = False


                elif self.stimulation_mode[i] == "crossing_software_timed":
                    if self.crossing_direction[i] == "x":
                        if self.prevX < self.stim_coord[i][0] and self.XPos >= self.stim_coord[i][0]:
                            if not self.stim_state[i]:
                                self.stim_onset_time_log[i] = self.frame_onset_time
                                self.stim_state[i] = True
                        if self.stim_state[i] and (self.frame_onset_time - self.stim_onset_time_log[i] >= self.stim_duration[i]):
                            self.stim_state[i] = False

                    elif self.crossing_direction[i] == "y":
                        if self.prevY < self.stim_coord[i][1] and self.YPos >= self.stim_coord[i][1]:
                            if not self.stim_state[i]:
                                self.stim_onset_time_log[i] = self.frame_onset_time
                                self.stim_state[i] = True
                        if self.stim_state[i] and (self.frame_onset_time - self.stim_onset_time_log[i] >= self.stim_duration[i]):
                            self.stim_state[i] = False


                elif self.stimulation_mode[i] == "zone":
                    distance = self.euclidean_distance([self.XPos,self.YPos], self.stim_coord[i])
                    if distance <= self.stim_radius[i]:
                        self.stim_state[i] = True
                    else:
                        self.stim_state[i] = False


                elif self.stimulation_mode[i] == "time":
                    if (self.frame_onset_time - self.state_onset_time >= self.stim_onset_time[i] and 
                        (self.frame_onset_time - (self.state_onset_time + self.stim_onset_time[i]) <= 
                         (self.stim_duration[i]))):
                        self.stim_state[i] = True
                    else:
                        self.stim_state[i] = False

        else:
            if sum(self.stim_state) > 0 or sum(self.stim_mode_pulsed) > 0:
                self.stim_state = [False]*self.nstimchannels


    def setTrigState(self):

        if self.stimulation:
            self.updateStimState()

        if self.use_timer:
            if self.stimulation:
                self.teensy.stim_duration = self.stim_pulse_duration
                self.teensy.stim_intensity = self.stim_intensitiy
                self.teensy.trig_stim[0:self.nstimchannels] = self.stim_state
                self.teensy.trig_mode_pulsed[0:self.nstimchannels] = self.stim_mode_pulsed
            self.teensy.trig_2p_start = self.start_2p
            self.teensy.trig_2p_end = self.end_2p

        if self.start_2p:
            self.twopstate = True
        if self.end_2p:
            self.twopstate = False

        self.start_2p = False
        self.end_2p = False


    def syncTimer(self):
        if self.use_timer:
            if not self.end_vr:
                self.teensy.sync()
            else:
                self.teensy.trig_2p_start = False
                self.teensy.trig_2p_end = True
                self.teensy.trig_stim = [False]*8
                self.teensy.trig_mode_pulsed = [False]*8
                self.teensy.sync()    
            self.stim_state_8bitcode = self.teensy.trig_stim_float
            self.stim_mode_pulsed_8bitcode = self.teensy.trig_mode_pulsed_float

            self.framecount_2p = self.teensy.frameclock # framecount is the number of 2P frames that have occurred since the previous sync event
            # if self.teensy.frameclock is not None:
            #     self.framecount_2p += self.teensy.frameclock # framecount is the index of the most recent 2P frame

    
    def logData(self):

        """
        Write data in the same order as row headers (see above)
        """
        if self.save:
            self.logger.writerow([self.trial, self.trial_index, self.trial_onset_counter, self.framecount_2p, self.frame_onset_time,
                                  self.XPos, self.YPos, self.heading, self.velocity, self.theta, self.twopstate,
                                  self.optic_flow_state, self.optic_flow_direction, self.optic_flow_velocity, 
                                  self.stim_state_8bitcode, self.stim_mode_pulsed_8bitcode, self.tail_tracking_framecount, 
                                  self.vr_gain, self.inactivity_timer, self.is_swimming, self.bools_to_int(self.movie_timer_started), 
                                  self.bools_to_int(self.movie_revealed), self.udpfailed])
            

    def printTrialInfo(self):
        if self.trial_index == 0:
            print("Starting trial: {}, index: {}, id: {}, repeat: until user presses w or e".format(self.trial, self.trial_index, self.trial_id))
        else:
            print("Starting trial: {}, index: {}, id: {}, repeat: {} of {}".format(self.trial, self.trial_index, self.trial_id, self.repcount, self.repeats-1))


    def updateTrialParams(self):

        self.trial += 1

        if self.trial_index == 0:
            if self.force_progress == True:
                self.trial_index += 1
        else:
            if self.repeat_incomplete_trials:
                if self.trial_complete or self.force_progress:
                    self.repcount += 1
            else:
                self.repcount += 1

            if self.repcount == self.repeats:
                self.repcount = 0
                self.trial_index += 1

        self.start_trial_end_timer = False
        self.trial_complete = False
        self.force_progress = False
        self.end_trial = False

        if self.do2p and self.twopstate and not self.continuous2p:
            self.end_2p = True
        self.fade_out = False
        self.fade_in = False
        self.inactivity_timer = 0.
        self.inactivity_triggered_flow_state = False

        if self.trial_index == self.ntrials:
            print("All trials executed. Ending experiment...")
            self.exitVR()
        else:
            self.setParamsForCurrentTrial()


    # Define a procedure to move the camera.
    def spinCameraTask(self, task):
        
        self.frame_onset_time = globalClock.getFrameTime()
        self.frame_dt = builtins.globalClock.getDt()

        if self.use_keyboard:
            self.updateCameraWithKeyboard()
        else:
            self.updateCameraWithTailTracking()
        
        if self.optic_flow or self.inactivity_leads_to == "flow":
            self.updateCameraWithOpticFlow()

        self.YPos = builtins.base.camera.getY()
        self.XPos = builtins.base.camera.getX()
        self.ZPos = builtins.base.camera.getZ()
        self.heading = builtins.base.camera.getH()

        if self.velocity <= self.inactivity_threshold and self.initialized:
            self.inactivity_timer += self.frame_dt
        if self.velocity > self.inactivity_threshold and self.inactivity_timer > 0.:
            self.inactivity_timer = 0.

        if self.initialized == False:
            self.initialized = True
            self.state_onset_time = self.frame_onset_time
            self.state = "trial"
            # Use first trial to find FOV, acquire reference stack and set up motion correction
            # No triggered 2P acquisition needed in this phase. 
            # Remember to toggle self.do2p to turn off imaging trigger. 
            # Uncomment the block below for consistency in case imaging is needed for the very first trial
            # if self.do2p:
            #     self.start_2p = True
            self.printTrialInfo()

        if self.state == "trial":

            self.distance_to_nearest_end_point = self.distance_to_nearest_target([self.XPos,self.YPos], self.trial_end_coords)
            
            if self.distance_to_nearest_end_point <= self.distance_threshold:
            
                if self.start_trial_end_timer == False:
                    self.trial_complete = True
                    self.start_trial_end_timer = True
                    self.trial_end_timer = self.frame_onset_time
            
            if ((self.start_trial_end_timer and self.frame_onset_time - self.trial_end_timer >= self.trial_end_delay) or 
                (self.frame_onset_time - self.state_onset_time >= self.max_trial_duration) or 
                (self.inactivity_timer >= self.max_inactive_duration and self.inactivity_leads_to == "reset")):
                self.end_trial = True
                self.start_trial_end_timer = False

            if self.inactivity_timer >= self.max_inactive_duration and self.inactivity_leads_to == "flow" and self.inactivity_triggered_flow_state == False:
                self.inactivity_triggered_flow_timer = self.frame_onset_time
                self.inactivity_triggered_flow_state = True

            if (self.inactivity_triggered_flow_state == True and 
                self.frame_onset_time - self.inactivity_triggered_flow_timer >= self.inactivity_triggered_flow_duration):
                self.inactivity_triggered_flow_state = False
                self.inactivity_timer = 0. # remove this line to continue counting inactivity time after flow is triggered

            if self.end_trial:

                self.updateTrialParams()

                self.printTrialInfo()

                self.state = "intertrial"
                self.state_onset_time = self.frame_onset_time

        if self.state == "intertrial":

            distance_to_start_point = self.euclidean_distance([self.XPos,self.YPos], self.trial_start_coord)

            if self.fade_out == False:
                LerpColorScaleInterval(self.render, self.fade_duration, (0, 0, 0, 1.0)).start()
                LerpFunc(self.fadeFog, fromData=0, toData=1, duration=self.fade_duration).start()
                self.fade_timer = self.frame_onset_time
                self.fade_out = True

            if self.frame_onset_time - self.fade_timer >= self.fade_duration and self.fade_in == False:
                self.setupScene()
                LerpColorScaleInterval(self.render, self.fade_duration, (1, 1, 1, 1.0)).start()
                LerpFunc(self.fadeFog, fromData=1, toData=0, duration=self.fade_duration).start()
                self.fade_in = True  

            if (self.frame_onset_time - self.state_onset_time >= self.intertrial_duration or 
                distance_to_start_point <= self.distance_threshold and self.fade_in == True):
                self.state = "trial"
                self.state_onset_time = self.frame_onset_time
                self.trial_onset_counter += 1
                if self.do2p and self.trial_index > 0:
                    if self.continuous2p and self.twopstate == False:
                        self.start_2p = True
                    else:
                        self.start_2p = True
                    

        if self.show_landmarks and self.hide_movie and self.state == "trial":
            for i in range(self.landmark_number):
                if self.landmark_flag[i] and self.background_texture[i]:
                    if self.euclidean_distance([self.XPos, self.YPos], self.landmark_position[i][0:2]) <= self.movie_onset_distance:
                        
                        if not self.movie_timer_started[i]:
                            self.movie_timer_started[i] = True
                            self.movie_trigger_time[i] = self.frame_onset_time
                            print('movie triggered at %.2fs at X: %.2f, Y: %.2f' %(self.frame_onset_time, self.XPos, self.YPos))

                        if self.movie_timer_started[i] and self.frame_onset_time - self.movie_trigger_time[i] >=self.movie_onset_delay and not self.movie_revealed[i]:
                            if self.landmark_nodes[i] is not None or str(self.landmark_nodes[i]) != '**removed**':
                                try:
                                    self.landmark_nodes[i].removeNode()
                                    if self.mirror_landmarks[i]:
                                        self.landmark_nodes_mirrored[i].removeNode()
                                    self.movie_revealed[i] = True
                                    print('movie revealed at %.2fs at X: %.2f, Y: %.2f' %(self.frame_onset_time, self.XPos, self.YPos))
                                except:
                                    print('background texture does not exist')


        self.setOpticFlowState()

        self.setTrigState()

        self.syncTimer()

        self.logData()

        self.prevX = self.XPos
        self.prevY = self.YPos

        #Lock ZPosition in case it changes with collisions
        if self.ZPos != self.init_position[2]:
            builtins.base.camera.setZ(self.init_position[2])

        return Task.cont


    def exitVR(self):
        
        self.end_vr = True

        if self.use_timer:
            self.syncTimer()
            self.teensy.close()

        if self.save:
            self.infofile.write("\n]}")
            self.infofile.close()
            self.datafile.close()

        print('Exiting VR')
        sys.exit()


if __name__ == "__main__":
    vr = ZebrafishVirtualReality()
    vr.run()