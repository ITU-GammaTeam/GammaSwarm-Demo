# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# You should have received a copy of the GNU General Public License
# along with this program. If not, see <https://www.gnu.org/licenses/>.
import time
from collections import namedtuple
from threading import Thread

from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

SwarmPosition = namedtuple('SwarmPosition', 'x y z')
SwarmOrientation = namedtuple('SwarmOrientation', 'roll pitch yaw')

#TODO İçerisine eğer bağlanamazsak bize hangisine bağlanamadı bildirmeli ve güncellemeli

class _Factory:
    """
    Default Crazyflie factory class.
    """

    def construct(self, uri):
        return SyncCrazyflie(uri)


class CachedCfFactory:
    """
    Factory class that creates Crazyflie instances with TOC caching
    to reduce connection time.
    """

    def __init__(self, ro_cache=None, rw_cache=None):
        self.ro_cache = ro_cache
        self.rw_cache = rw_cache

    def construct(self, uri):
        cf = Crazyflie(ro_cache=self.ro_cache, rw_cache=self.rw_cache)
        return SyncCrazyflie(uri, cf=cf)

def linkQualityCB(self, percentage):
        self._linkQuality = percentage

class Swarm:
    """
    Runs a swarm of Crazyflies. It implements a functional-ish style of
    sequential or parallel actions on all individuals of the swarm.

    When the swarm is connected, a link is opened to each Crazyflie through
    SyncCrazyflie instances. The instances are maintained by the class and are
    passed in as the first argument in swarm wide actions.
    """

    def __init__(self, uris, factory=_Factory()):
        """
        Constructs a Swarm instance and instances used to connect to the
        Crazyflies

        :param uris: A set of uris to use when connecting to the Crazyflies in
        the swarm
        :param factory: A factory class used to create the instances that are
         used to open links to the Crazyflies. Mainly used for unit testing.
        """
        self._cfs = {}
        self._is_open = False
        self._positions = dict()
        self._orientations = dict()
        self._link_status = dict()
        self._lh_status = dict()
        self._acc_values = dict()
        self._kill_counter = dict()
        self._unshifted_dict = dict()
        
        #For linkQuality add some functionalities to SyncCrazyFlie Class, after that we can sample this class
        SyncCrazyflie._linkQuality = 100
        SyncCrazyflie.linkQualityCB = linkQualityCB 
        #TODO eger kalkmama problemi devam ederse buraya urilere bakılcak
        for uri in uris:
            self._positions[uri] = SwarmPosition(0,0,0) 
            self._orientations[uri] = SwarmOrientation(0,0,0)
            self._link_status[uri] =  True #Can be defined False, after open_links execution we can switch all to True. But not needed for our case.
            self._lh_status[uri] = True
            self._acc_values[uri] = 0.1
            self._kill_counter[uri] = 999
            self._unshifted_dict[uri] = True
            self._cfs[uri] = factory.construct(uri)
        self.open_links()
        self.add_loggings()


    def __open_link(self,scf):
        scf.open_link()
        #After connection add linkQuality Callback
        scf.cf.link_quality_updated.add_callback(scf.linkQualityCB)
        print("CONNECTED",scf.cf.link_uri)


    def open_links(self):
        """
        Open links to all individuals in the swarm
        """
        if self._is_open:
            raise Exception('Already opened')

        try:
            self.parallel_safe(self.__open_link)
            self._is_open = True
        except Exception as e:
            self.close_links()
            raise e
    
    def add_loggings(self):
        self.parallel_safe(self.__add_logging)

    def __add_logging(self,scf):
        _lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
        _lg_stab.add_variable('stateEstimate.x', 'float')
        _lg_stab.add_variable('stateEstimate.y', 'float')
        _lg_stab.add_variable('stateEstimate.z', 'float')
        scf.cf.log.add_config(_lg_stab)
        _lg_stab.data_received_cb.add_callback(self._stab_log_data)
        _lg_stab.error_cb.add_callback(self._stab_log_error)

        _lg_stab.start()

    def _stab_log_data(self, timestamp, data, logconf):
        cf = logconf.cf
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        z = data['stateEstimate.z']
        self._positions[cf.link_uri] = SwarmPosition(x, y, z)
        """Callback from a the log API when data arrives"""
        roll = 0
        pitch = 0
        yaw = 0
        self._orientations[cf.link_uri] = SwarmOrientation(roll, pitch, yaw)


    def _stab_log_error(self):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def add_loggings(self):
        self.parallel_safe(self.__add_logging)

    def __add_logging(self,scf):
        _lg_stab = LogConfig(name='Stabilizer', period_in_ms=100)
        _lg_stab.add_variable('stateEstimate.x', 'float')
        _lg_stab.add_variable('stateEstimate.y', 'float')
        _lg_stab.add_variable('stateEstimate.z', 'float')
        _lg_stab.add_variable('lighthouse.status', 'uint8_t')
        _lg_stab.add_variable('acc.x', 'float')
        _lg_stab.add_variable('acc.y', 'float')
        scf.cf.log.add_config(_lg_stab)
        _lg_stab.data_received_cb.add_callback(self._stab_log_data)
        _lg_stab.error_cb.add_callback(self._stab_log_error)

        _lg_stab.start()


    def _stab_log_data(self, timestamp, data, logconf):
        cf = logconf.cf
        x = data['stateEstimate.x']
        y = data['stateEstimate.y']
        z = data['stateEstimate.z']
        status = data['lighthouse.status']
        _ax = data['acc.x']
        _ay = data['acc.y']
        """Callback from a the log API when data arrives"""
        roll = 0
        pitch = 0
        yaw = 0
        self._positions[cf.link_uri] = SwarmPosition(x, y, z)
        self._orientations[cf.link_uri] = SwarmOrientation(roll, pitch, yaw)

        if status == 2:
            self._lh_status[cf.link_uri] = True
        else:
            self._lh_status[cf.link_uri] = False 
        
        self._acc_values[cf.link_uri] = abs(_ax) + abs(_ay)


    def _stab_log_error(self):
        """Callback from the log API when an error occurs"""
        print('Error while logging')


    def close_links(self):
        """
        Close all open links
        """
        for uri, cf in self._cfs.items():
            cf.close_link()

        self._is_open = False

    # def __enter__(self):
    #     self.open_links()
    #     return self

    # def __exit__(self, exc_type, exc_val, exc_tb):
    #     self.close_links()
    


    def get_estimated_positions(self):
        """
        Return a `dict`, keyed by URI and with the SwarmPosition namedtuples as
        value, with the estimated (x, y, z) of each Crazyflie in the swarm.
        """
        #There is also always we have position value. When you want you can get
        return self._positions,self._orientations


    def get_all_bs_visibility(self):
        """
        Return a `dict`, keyed by URI and with the state of bs visibility
        """
        #!! Also always we have lh status for every drone in every time
        #self.parallel(self.__get_bs_detect_status)
        return self._lh_status
    
    
    def get_all_acc_values(self):
        """
        Return a `dict`, keyed by URI and with the state of acceptable tilt
        """

        return self._acc_values

    def get_estimated_activations(self):
        return self._unshifted_dict


    def __wait_for_position_estimator(self, scf):
        log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
        log_config.add_variable('kalman.varPX', 'float')
        log_config.add_variable('kalman.varPY', 'float')
        log_config.add_variable('kalman.varPZ', 'float')

        var_y_history = [1000] * 10
        var_x_history = [1000] * 10
        var_z_history = [1000] * 10

        threshold = 0.002

        with SyncLogger(scf, log_config) as logger:
            for log_entry in logger:
                data = log_entry[1]

                var_x_history.append(data['kalman.varPX'])
                var_x_history.pop(0)
                var_y_history.append(data['kalman.varPY'])
                var_y_history.pop(0)
                var_z_history.append(data['kalman.varPZ'])
                var_z_history.pop(0)

                min_x = min(var_x_history)
                max_x = max(var_x_history)
                min_y = min(var_y_history)
                max_y = max(var_y_history)
                min_z = min(var_z_history)
                max_z = max(var_z_history)

                if (max_x - min_x) < threshold and (
                        max_y - min_y) < threshold and (
                        max_z - min_z) < threshold:
                    break

    def __reset_estimator(self, scf):
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        self.__wait_for_position_estimator(scf)

    def reset_estimators(self):
        """
        Reset estimator on all members of the swarm and wait for a stable
        positions. Blocks until position estimators finds a position.
        """
        self.parallel_safe(self.__reset_estimator)

    def sequential(self, func, args_dict=None):
        """
        Execute a function for all Crazyflies in the swarm, in sequence.

        The first argument of the function that is passed in will be a
        SyncCrazyflie instance connected to the Crazyflie to operate on.
        A list of optional parameters (per Crazyflie) may follow defined by
        the `args_dict`. The dictionary is keyed on URI and has a list of
        parameters as value.

        Example:
        ```python
        def my_function(scf, optional_param0, optional_param1)
            ...

        args_dict = {
            URI0: [optional_param0_cf0, optional_param1_cf0],
            URI1: [optional_param0_cf1, optional_param1_cf1],
            ...
        }


        swarm.sequential(my_function, args_dict)
        ```

        :param func: The function to execute
        :param args_dict: Parameters to pass to the function
        """
        for uri, cf in self._cfs.items():
            args = self._process_args_dict(cf, uri, args_dict)
            func(*args)

    def parallel(self, func, args_dict=None):
        """
        Execute a function for all Crazyflies in the swarm, in parallel.
        One thread per Crazyflie is started to execute the function. The
        threads are joined at the end. Exceptions raised by the threads are
        ignored.

        For a more detailed description of the arguments, see `sequential()`

        :param func: The function to execute
        :param args_dict: Parameters to pass to the function
        """
        try:
            self.parallel_safe(func, args_dict)
        except Exception:
            pass

    def parallel_safe(self, func, args_dict=None):
        """
        Execute a function for all Crazyflies in the swarm, in parallel.
        One thread per Crazyflie is started to execute the function. The
        threads are joined at the end and if one or more of the threads raised
        an exception this function will also raise an exception.

        For a more detailed description of the arguments, see `sequential()`

        :param func: The function to execute
        :param args_dict: Parameters to pass to the function
        """
        threads = []
        reporter = self.Reporter()

        for uri, scf in self._cfs.items():
            args = [func, reporter] + \
                self._process_args_dict(scf, uri, args_dict)

            thread = Thread(target=self._thread_function_wrapper, args=args)
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

        if reporter.is_error_reported():
            first_error = reporter.errors[0]
            raise Exception('One or more threads raised an exception when '
                            'executing parallel task') from first_error

    def _thread_function_wrapper(self, *args):
        reporter = None
        try:
            func = args[0]
            reporter = args[1]
            func(*args[2:])
        except Exception as e:
            if reporter:
                reporter.report_error(e)

    def _process_args_dict(self, scf, uri, args_dict):
        args = [scf]

        if args_dict:
            #print(args,args_dict[uri])
            args += args_dict[uri]

        return args
    
    def applyCommand(self,scf: SyncCrazyflie, args):     #TODO Thread'e verilecek ana fonksiyon bu olacak! Her bir drone icin yapılması gereken islemler burada olmalı
        commander = scf.cf.commander

        vx, vy, vz, yawrate = args[0] , args[1] , args[2] , args[3]
        for i in range(0,10):
            commander.send_velocity_world_setpoint(vx, vy, vz, yawrate)
        #print("Command Sended",time.time())


    def __get_link_status(self, scf):
        """
        Retrieve the link status for an individual drone
        """
        if scf._linkQuality == 70.0:
            self._link_status[scf.cf.link_uri] = False
        else:
            self._link_status[scf.cf.link_uri] = True   #(scf._linkQuality > 71)
        #print(scf._linkQuality)


    def get_all_link_status(self):
        """
        Return a `dict`, keyed by URI and with the state of link between CrazyRadio and each Crazyflie in the swarm.
        """
        self.parallel(self.__get_link_status) 
        
        return self._link_status


    
    def disarmCrazyFlie(self,scf):
        cf = scf.cf
        cf.commander.send_stop_setpoint()
        cf.close_link() #Must be try!


    #TODO Drone kapatmak icin send_stop_setpoint --> Motor off kullanmali
    def disarmSwarm(self,scf: SyncCrazyflie, args):
        pass

    
    #SAFETY CHECK
    def activateLedBitMask(self,scf: SyncCrazyflie):
        scf.cf.param.set_value('led.bitmask', 255)

    def deactivateLedBitMask(self,scf: SyncCrazyflie):
        scf.cf.param.set_value('led.bitmask', 0)

    def individualLightCheck(self,scf: SyncCrazyflie):
        self.activateLedBitMask(scf)
        time.sleep(1)
        self.deactivateLedBitMask(scf)
        time.sleep(1)
    
    def lightCheck(self):
        #TODO sequential yaparak sirasi ile listedeki dronelar gerçekte hanisine denk geliyor check edilebilir.
        self.sequential(self.individualLightCheck)

    
    #TODO baska bir seyi kontrole ihtiyac olursa yine buraya bir fonksiyon koyup threadle calıstırabiliriz!

    class Reporter:
        def __init__(self):
            self.error_reported = False
            self._errors = []

        @property
        def errors(self):
            return self._errors

        def report_error(self, e):
            self.error_reported = True
            self._errors.append(e)

        def is_error_reported(self):
            return self.error_reported















#---------------------------------------------------<        YEDEK      >---------------------------------------------------------------------




    # def __get_bs_detect_status(self, scf):
    #     """
    #     Retrieve the bs detection status for an individual
    #     """
    #     # lh_log_config = LogConfig(name='lighthouse', period_in_ms=10)
    #     # lh_log_config.add_variable('lighthouse.status', 'uint8_t')
    #     # with SyncLogger(scf, lh_log_config) as logger:
    #     #     for entry in logger:
    #     #         status = entry[1]['lighthouse.status']
    #     #         if status == 2:
    #     #             self._lh_status[scf.cf.link_uri] = True
    #     #         else:
    #     #             self._lh_status[scf.cf.link_uri] = False
    #     #         break
    #     self._lh_status[scf.cf.link_uri] = True 







    # def __get_estimated_position(self, scf):

#########################3          YEDEK #################
# def __get_estimated_position(self, scf):

    #     pos_log_config = LogConfig(name='stateEstimate', period_in_ms=10)
    #     pos_log_config.add_variable('stateEstimate.x', 'float')
    #     pos_log_config.add_variable('stateEstimate.y', 'float')
    #     pos_log_config.add_variable('stateEstimate.z', 'float')


    #     # orientation_log_config = LogConfig(name='stabilizer', period_in_ms=10) #TODO ms'ler değişilebilir
    #     # orientation_log_config.add_variable('stabilizer.roll', 'float')
    #     # orientation_log_config.add_variable('stabilizer.pitch', 'float')
    #     # orientation_log_config.add_variable('stabilizer.yaw', 'float')

    #     #TODO Roll pitch yaw dışında Velocity de gerekli olabilir

    #     with SyncLogger(scf, pos_log_config) as logger:
    #         for entry in logger:
    #             x = entry[1]['stateEstimate.x']
    #             y = entry[1]['stateEstimate.y']
    #             z = entry[1]['stateEstimate.z']
    #             self._positions[scf.cf.link_uri] = SwarmPosition(x, y, z)
    #             break
    #     # with SyncLogger(scf,orientation_log_config) as logger:
    #     #     for entry in logger:
    #     #         print("ENTRY",entry)
    #     #         roll = entry[1]['stabilizer.roll']
    #     #         pitch = entry[1]['stabilizer.pitch']
    #     #         yaw = entry[1]['stabilizer.yaw']
    #     #         self._orientations[scf.cf.link_uri] = SwarmOrientation(roll, pitch, yaw)
    #     #         break
    #     roll = 0
    #     pitch = 0
    #     yaw = 0
    #     self._orientations[scf.cf.link_uri] = SwarmOrientation(roll, pitch, yaw)

    # def get_estimated_positions(self):
    #     """
    #     Return a `dict`, keyed by URI and with the SwarmPosition namedtuples as
    #     value, with the estimated (x, y, z) of each Crazyflie in the swarm.
    #     """
    #     self.parallel(self.__get_estimated_position)

    #     return self._positions,self._orientations

    #     return self._positions,self._orientations
    

