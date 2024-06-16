# 发布者开发记录 # 

## 环境配置 ##  

参考：  
ROS2

简易办法就是直接下在/usr/bin/python3    
conda 有和ros的奇怪冲突问题，稍后有空解决  

运行python publisher  
```
rosdep install -i --from-path src --rosdistro foxy -y  
colcon build --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3  
. install/setup.bash  # 新开终端要跑这个
colcon build --cmake-args -DPYTHON_EXECUTABLE=/usr/bin/python3
ros2 run pub control_pub
ros2 run sub control_sub
```
杀进程  
```
ps aux | grep control_sub
kill -9 <pid> 
```

当前topic有
> 控制信号`control_msg`
> 状态`state_msg`

```
ros2 topic list
ros2 topic echo control_msg  
rqt_graph
```

库版本
> opencv==4.2.0
> pip install pyrealsense2==2.50.0.3812
> librealsense是在/usr/local的，2.50
> numpy==1.17.4  <--> 1.24.4
> 其它标准库，math之类的应该不影响

### 开发

参考：  
python并发  
同时订阅与发布
https://blog.csdn.net/qq_16893195/article/details/113123386  

为什么现在中止要control+c两次  

引入子模块解决方案  https://blog.csdn.net/scarecrow_sun/article/details/127589380

同时订阅与发布可能解决方案  
```py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TalkerAndListener(Node):
    def __init__(self):
        super().__init__('talker_and_listener')
        # 创建发布者，发布到"chatter"话题，消息类型为String
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        # 创建订阅者，订阅"chatter_two"话题，消息类型为String
        self.subscription_ = self.create_subscription(
            String,
            'chatter_two',
            self.listener_callback,
            10
        )
        # 创建定时器，周期为0.5秒，调用self.timer_callback
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        # 创建消息
        msg = String()
        msg.data = 'Hello, world! %d' % (self.get_clock().now().nanoseconds / 1e9)
        # 发布消息
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def listener_callback(self, msg):
        # 订阅者回调函数，当接收到消息时调用
        self.get_logger().info('Received: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = TalkerAndListener()
    rclpy.spin(node)  # 启动事件循环
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```   

### ROS2
- 库都要放在package.xml吗？  
不用，应该直接pip  
- 多线程  
timer和while rclpy.ok()有什么区别吗  
c++和py的写法ok吗  


### 算法优化

realsense圈眼球不如内窥镜  
还有些未解决的bug  

需要看看cnn + realsense怎么样  



### nature配置  
必须要先装个clash，此前要想办法将文件弄过去  
然后装python那几个库  