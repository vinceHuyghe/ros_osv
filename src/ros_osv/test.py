from py_openshowvar import OpenShowVar
from tf import transformations
from tf.msg import tfMessage
from geometry_msgs.msg import Transform

client = OpenShowVar(ip='192.168.1.100', port=7000)

if client.can_connect:
    var = client.read('$POS_ACT', debug=False)
    var = var.decode()
    var = var.split()
    pos = [var[2].strip(','),var[4].strip(','),var[6].strip(','),var[8].strip(','),var[10].strip(','),var[12].strip(','),var[14].strip(',')]
    
    tf_msg = Transform()
    tf_msg.translation.x = pos[0]
    tf_msg.translation.y = pos[1]
    tf_msg.translation.z = pos[2]
    
    quat = transformations.quaternion_from_euler(
        float(pos[4]),
        float(pos[5]),
        float(pos[6]))
    
    tf_msg.rotation.x = quat[0]
    tf_msg.rotation.y = quat[1]
    tf_msg.rotation.z = quat[2]
    tf_msg.rotation.w = quat[3]
    
    print(tf_msg)
else:
    print('unable to connect')

# {E6POS: X 12.84936, Y -394.9386, Z 368.6748, A 17.71607, B 89.99682, C -1.162400, S 18, T 10, E1 90.00000, E2 0.0, E3 0.0, E4 0.0, E5 0.0, E6 0.0}
