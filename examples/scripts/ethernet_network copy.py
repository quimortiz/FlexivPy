import easy_controllers
from FlexivPy.robot.robot_client import Flexiv_client
import numpy as np

# Copy the following into a config file, e.g, cyclonedds.xml
# Before running the script
# export CYCLONEDDS_URI=file://path_to/cyclonedds.xml
# For example:
# export CYCLONEDDS_URI=file://./examples/cyclonedds.xml

# <CycloneDDS>
# <Domain id="any">
#         <General>
#             <NetworkInterfaceAddress>192.168.2.102</NetworkInterfaceAddress>
#         </General>
#     <Discovery>
#         <Peers>
#             <Peer address="192.168.2.101"/>
#         </Peers>
#         <ParticipantIndex>auto</ParticipantIndex>
#     </Discovery>
#     </Domain>
# </CycloneDDS>


# When you start the server in the robot config, use the config file
# with the exchange of the IP addresses of the robot server and the contoller computer

# remeber that when running sudo, the environment variables are not passed by default
# use sudo -E to pass the environment variables
# <CycloneDDS>
# <Domain id="any">
#         <General>
#             <NetworkInterfaceAddress>192.168.2.101</NetworkInterfaceAddress>
#         </General>
#     <Discovery>
#         <Peers>
#             <Peer address="192.168.2.102"/>
#         </Peers>
#         <ParticipantIndex>auto</ParticipantIndex>
#     </Discovery>
#     </Domain>
# </CycloneDDS>


robot = Flexiv_client()

kp_scale = 2.0
kv_scale = 2.0

status = easy_controllers.run_controller(
    robot,
    easy_controllers.GoJointConfiguration(
        qdes=np.array([0.3, -0.698, 0.000, 1.571, -0.000, 0.698, -0.000]),
        max_v=0.3,
        max_extra_time_rel=0.2,
        kp_scale=kp_scale,
        kv_scale=kv_scale,
    ),
    dt=0.005,
    max_time=120,
)

print("status is:", status)
