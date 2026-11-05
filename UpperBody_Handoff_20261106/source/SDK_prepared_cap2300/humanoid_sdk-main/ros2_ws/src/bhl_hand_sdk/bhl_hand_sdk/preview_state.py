"""File-to-RViz estimate bridge. Never opens CAN; topic is not /joint_states."""
import json,math,time
from pathlib import Path

def read_estimate(path,now=None):
    now=time.time() if now is None else now
    d=json.loads(Path(path).read_text());age=now-float(d['utc'])
    if not 0<=age<=.5:return None
    q=d['joints']
    if not isinstance(q,dict) or any(not isinstance(k,str) or isinstance(v,bool) or not math.isfinite(float(v)) or abs(v)>math.pi for k,v in q.items()):raise ValueError('Invalid estimated joint data')
    if 'NOT measured' not in d.get('source','') and 'not calibrated' not in d.get('source',''):raise ValueError('Estimate must be labeled as an estimate')
    return d

def main():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    class Preview(Node):
        def __init__(self):
            super().__init__('hand_camera_estimate')
            self.declare_parameter('input_file',str(Path.home()/'.local/state/bhl/camera_retarget_preview.json'))
            self.path=Path(self.get_parameter('input_file').value)
            self.pub=self.create_publisher(JointState,'/hand_camera/estimated_joint_states',10)
            self.info=self.create_publisher(String,'/hand_camera/preview_info',10);self.q={};self.problem=''
            from .hand_retarget import JOINTS
            self.names={j for names in JOINTS.values() for j in names};self.q={j:0. for j in self.names}
            self.create_timer(.05,self.tick)
        def tick(self):
            try:
                d=read_estimate(self.path)
                if d is None:return
                if any(k not in self.names for k in d['joints']):raise ValueError('Unknown joint name')
                self.q.update({k:float(v) for k,v in d['joints'].items()})
                msg=JointState();msg.header.stamp=self.get_clock().now().to_msg();msg.name=sorted(self.q);msg.position=[self.q[n] for n in msg.name];self.pub.publish(msg)
                note=String();note.data=json.dumps({'source':d['source'],'motor_streaming':d.get('motor_streaming',False),'measured_robot_state':False});self.info.publish(note);self.problem=''
            except (OSError,ValueError,KeyError,TypeError) as e:
                if self.problem!=str(e):self.get_logger().warning(str(e));self.problem=str(e)
    rclpy.init();node=Preview()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.shutdown()
