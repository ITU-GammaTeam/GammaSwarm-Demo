class SimulationParameter:
    def __init__(self):
        self.gui = True
        self.user_debug_gui = False
        self.num_drones = None
        self.INIT_XYZS = None
        self.INIT_RPYS = None
        self.drone = "cf2x"
        self.physics = "pyb_drag"        # ALL PHYSICS MODE : pyb,  dyn,  pyb_gnd,  pyb_drag,  pyb_dw,  pyb_gnd_drag_dw
        self.neighbourhood_radius = 10
        self.vision = False
        self.simulation_freq_hz = 240
        self.control_freq_hz = 48
        self.aggregate = True
        self.obstacles = False #Bu TRUE OLURSA ENGEL EKLER
        self.record_video = False
        
