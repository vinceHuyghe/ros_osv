import rospy
import tf.transformations
import tf2_ros
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Vector3

from industrial_reconstruction_msgs.msg import NormalFilterParams
from industrial_reconstruction_msgs.srv import (
    StartReconstruction,
    StartReconstructionRequest,
    StopReconstruction,
    StopReconstructionRequest,
)


from ros_osv.py_openshowvar import OpenShowVar
import math

# reconstruction parameters
start_srv_req = StartReconstructionRequest()
start_srv_req.tracking_frame = "tool0"
start_srv_req.relative_frame = "base_link"
start_srv_req.translation_distance = 0.0
start_srv_req.rotational_distance = 0.0
start_srv_req.live = True
start_srv_req.tsdf_params.voxel_length = 0.02
start_srv_req.tsdf_params.sdf_trunc = 0.04
start_srv_req.tsdf_params.min_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.tsdf_params.max_box_values = Vector3(x=0.0, y=0.0, z=0.0)
start_srv_req.rgbd_params.depth_scale = 1000
start_srv_req.rgbd_params.depth_trunc = 0.5
start_srv_req.rgbd_params.convert_rgb_to_intensity = False

stop_srv_req = StopReconstructionRequest()
# stop_srv_req.archive_directory = '/dev_ws/src.reconstruction/'
stop_srv_req.mesh_filepath = '/home/v/test.ply'
# stop_srv_req.normal_filters = [NormalFilterParams(
#                     normal_direction=Vector3(x=0.0, y=0.0, z=1.0), angle=90)]
# stop_srv_req.min_num_faces = 1000

digital_output_trigger = 25


class OsvRecon:
    def __init__(self, ip: str, port: int) -> None:

        rospy.init_node('osv')
        self.name = rospy.get_name()
        # init osv connection
        self.client = OpenShowVar(ip=ip, port=port)
        self.tf_br = tf2_ros.TransformBroadcaster()
        rospy.loginfo(
            f'{self.name} node started, KVP CONNECTED {self.client.can_connect}'
        )
        rospy.wait_for_service('/start_reconstruction')
        rospy.loginfo('robot program: waiting for /start_reconstruction srv')
        self.start_recon = rospy.ServiceProxy(
            '/start_reconstruction', StartReconstruction
        )
        self.stop_recon = rospy.ServiceProxy('/stop_reconstruction', StopReconstruction)

    def read_and_broadcast(self):

        recon_started = False

        while not rospy.is_shutdown():

            # Read digital output
            var = self.client.read(f'$OUT[{digital_output_trigger}]', debug=False)
            var = var.decode()
            if var == 'TRUE':
                output_state = True
            if var == 'FALSE':
                output_state = False


            if output_state:
                # Start reconstruction with service srv_req
                resp = self.start_recon(start_srv_req)
                if resp:
                    rospy.loginfo(f'{self.name}: reconstruction started successfully')
                    recon_started = True
                else:
                    rospy.loginfo(f'{self.name}: failed to start reconstruction')

            if not output_state and recon_started:
                # Stop reconstruction with service srv_req
                resp = self.stop_recon(stop_srv_req)

                if resp:
                    rospy.loginfo(f'{self.name}: reconstruction stopped successfully')
                else:
                    rospy.loginfo(f'{self.name}: failed to stop reconstruction')

            # Read tcp position relative to base frame
            var = self.client.read('$POS_ACT', debug=False)
            self.tf_br.sendTransform(OsvRecon.pos_act_to_tf(var.decode()))

    @staticmethod
    def pos_act_to_tf(var: str) -> TransformStamped:

        # POS_ACT is relative to base frame
        var = var.replace(',', '').split(' ')
        pos = list(map(float, [var[2], var[4], var[6], var[8], var[10], var[12]]))

        quat = tf.transformations.quaternion_from_euler(
            math.radians(pos[3]), math.radians(pos[4]), math.radians(pos[5]), 'rxyz'
        )

        tfs = TransformStamped()

        tfs.header.stamp = rospy.Time.now()
        tfs.header.frame_id = 'base_link'
        tfs.child_frame_id = 'tcp'
        tfs.transform.translation.x = pos[0] / 1000
        tfs.transform.translation.y = pos[1] / 1000
        tfs.transform.translation.z = pos[2] / 1000
        tfs.transform.rotation.x = quat[0]
        tfs.transform.rotation.y = quat[1]
        tfs.transform.rotation.z = quat[2]
        tfs.transform.rotation.w = quat[3]

        return tfs


if __name__ == '__main__':

    osv = OsvRecon("192.168.1.100", 7000)
    osv.read_and_broadcast()
