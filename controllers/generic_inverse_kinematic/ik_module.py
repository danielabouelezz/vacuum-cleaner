# Copyright 2020 Simon Steinmann
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import sys
import tempfile
import numpy as np

try:
    from scipy.spatial.transform import Rotation as R
except ImportError:
    sys.exit('The "scipy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install scipy with this command: "pip install scipy"')

try:
    import ikpy
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')

class inverseKinematics():
    def __init__(self, supervisor, last_link_vector=None):
        # Initialize the Webots Supervisor.
        self.supervisor = supervisor
        self.timeStep = int(self.supervisor.getBasicTimeStep())  
        n = self.supervisor.getNumberOfDevices()
        self.motor_names = []
        self.sensor_names = []
        for i in range(n):
            device = self.supervisor.getDeviceByIndex(i)
            print(device.getName() , '  -  NodeType = ' , device.getNodeType())
            if device.getNodeType() == 54:
                sensor = device.getPositionSensor()
                self.motor_names.append(device.getName())
                try:
                    self.sensor_names.append(sensor.getName())
                except:
                    print('Rotational Motor: ' + device.getName() + ' has no Position Sensor')

     
        # Create the arm chain.         
        filename = None
        # (uncomment next two lines, if you want to save a copy of the generated urdf file)
        with open('filename1.urdf',  'w') as file:
            file.write(supervisor.getUrdf())   
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
            filename = file.name
            file.write(self.supervisor.getUrdf().encode('utf-8'))        
        self.armChain = ikpy.chain.Chain.from_urdf_file(filename, last_link_vector=last_link_vector)
        print(self.armChain)
        
        # make sure only links with a motor are active for IK calculations
        active_links = [False] * len(self.armChain.links)
        for i in range(len(self.armChain.links)):
            link_name = self.armChain.links[i].name
            active_links[i] = link_name in self.motor_names or link_name in self.sensor_names
            if active_links[i]:
                # ikpy includes the bounds as valid, In Webots they have to be less than the limit
                new_lower = new_upper = None                
                if self.armChain.links[i].bounds[0] is not None:
                    new_lower = self.armChain.links[i].bounds[0] + 0.00001
                if self.armChain.links[i].bounds[1] is not None:
                    new_upper = self.armChain.links[i].bounds[1] - 0.00001       
                self.armChain.links[i].bounds = (new_lower, new_upper)   
                self.last_active = i  
        if not any(active_links):
            print('WARNING: could not identify which links are active. Setting all links to active by default.')      
            active_links = [True] * len(self.armChain.links)    
        self.armChain.active_links_mask = active_links    
        print(self.armChain)

    def get_ik(self, target_pos):
        return self.armChain.inverse_kinematics(target_pos)        