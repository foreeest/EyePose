# 我的开发note

## 复现

`pip install -r requirements.txt`报这个错，req里都没有keras

ERROR: Could not find a version that satisfies the requirement keras-nightly~=2.5.0.dev (from tensorflow==2.5.3->-r requirements.txt (line 2)) (from versions: none)
ERROR: No matching distribution found for keras-nightly~=2.5.0.dev (from tensorflow==2.5.3->-r requirements.txt (line 2))

在mac的虚拟机上更离谱的报错，但这边`pip list`比较正常没有一堆ros啥的  
单装一个tensorflow,报错一致    
`pip install --upgrade pip`文心一言馊主意试试，20->24  
问题更多  

```
ERROR: Ignored the following versions that require a different python version: 1.25.0 Requires-Python >=3.9; 1.25.0rc1 Requires-Python >=3.9; 1.25.1 Requires-Python >=3.9; 1.25.2 Requires-Python >=3.9; 1.26.0 Requires-Python <3.13,>=3.9; 1.26.0b1 Requires-Python <3.13,>=3.9; 1.26.0rc1 Requires-Python <3.13,>=3.9; 1.26.1 Requires-Python <3.13,>=3.9; 1.26.2 Requires-Python >=3.9; 1.26.3 Requires-Python >=3.9; 1.26.4 Requires-Python >=3.9; 2.0.0b1 Requires-Python >=3.9; 2.0.0rc1 Requires-Python >=3.9; 2.0.0rc2 Requires-Python >=3.9; 2.14.1 Requires-Python >=3.9; 2.15.0 Requires-Python >=3.9; 2.15.1 Requires-Python >=3.9; 2.15.2 Requires-Python >=3.9; 2.16.0 Requires-Python >=3.9; 2.16.1 Requires-Python >=3.9; 2.16.2 Requires-Python >=3.9
ERROR: Could not find a version that satisfies the requirement keras-nightly~=2.5.0.dev (from tensorflow) (from versions: none)
ERROR: No matching distribution found for keras-nightly~=2.5.0.dev
```

重开然后用默认源`https://pypi.org/simple/`试试  
`python3 -m venv --clear envPupil`  
`pip install -i https://pypi.org/simple/ tensorflow==2.5.3`  
新报错
```
ERROR: Failed building wheel for wrapt
Running setup.py clean for wrapt
Failed to build termcolor wrapt
其一
Building wheel for termcolor (setup.py) ... error
  ERROR: Command errored out with exit status 1:
   command: /home/robot/developEye/Pupil-locator/envPupil/bin/python3 -u -c 'import sys, setuptools, tokenize; sys.argv[0] = '"'"'/tmp/pip-install-lfazwkif/termcolor/setup.py'"'"'; __file__='"'"'/tmp/pip-install-lfazwkif/termcolor/setup.py'"'"';f=getattr(tokenize, '"'"'open'"'"', open)(__file__);code=f.read().replace('"'"'\r\n'"'"', '"'"'\n'"'"');f.close();exec(compile(code, __file__, '"'"'exec'"'"'))' bdist_wheel -d /tmp/pip-wheel-fis3rifo
       cwd: /tmp/pip-install-lfazwkif/termcolor/
  Complete output (6 lines):
  usage: setup.py [global_opts] cmd1 [cmd1_opts] [cmd2 [cmd2_opts] ...]
     or: setup.py --help [cmd1 cmd2 ...]
     or: setup.py --help-commands
     or: setup.py cmd --help
  
  error: invalid command 'bdist_wheel'
  ----------------------------------------
  ERROR: Failed building wheel for termcolor
  Running setup.py clean for termcolor

```

`pip install -i https://pypi.org/simple/ opencv-python==4.2.0.34`  
success,hub也是  

tensorflow跟python版本对得上https://blog.csdn.net/ly869915532/article/details/124542362  

`pip install termcolor`显示已经有了  
难绷，开搜https://blog.csdn.net/u012680687/article/details/104199692  
但问题是`pip list`有tensorflow 2.5.3，但还有报错  

```
Traceback (most recent call last):
  File "inferno.py", line 7, in <module>
    import tensorflow.compat.v1 as tf
  File "/home/robot/developEye/Pupil-locator/envPupil/lib/python3.8/site-packages/tensorflow/__init__.py", line 41, in <module>
    from tensorflow.python.tools import module_util as _module_util
  File "/home/robot/developEye/Pupil-locator/envPupil/lib/python3.8/site-packages/tensorflow/python/__init__.py", line 40, in <module>
    from tensorflow.python.eager import context
  File "/home/robot/developEye/Pupil-locator/envPupil/lib/python3.8/site-packages/tensorflow/python/eager/context.py", line 32, in <module>
    from tensorflow.core.framework import function_pb2
  File "/home/robot/developEye/Pupil-locator/envPupil/lib/python3.8/site-packages/tensorflow/core/framework/function_pb2.py", line 16, in <module>
    from tensorflow.core.framework import attr_value_pb2 as tensorflow_dot_core_dot_framework_dot_attr__value__pb2
  File "/home/robot/developEye/Pupil-locator/envPupil/lib/python3.8/site-packages/tensorflow/core/framework/attr_value_pb2.py", line 16, in <module>
    from tensorflow.core.framework import tensor_pb2 as tensorflow_dot_core_dot_framework_dot_tensor__pb2
  File "/home/robot/developEye/Pupil-locator/envPupil/lib/python3.8/site-packages/tensorflow/core/framework/tensor_pb2.py", line 16, in <module>
    from tensorflow.core.framework import resource_handle_pb2 as tensorflow_dot_core_dot_framework_dot_resource__handle__pb2
  File "/home/robot/developEye/Pupil-locator/envPupil/lib/python3.8/site-packages/tensorflow/core/framework/resource_handle_pb2.py", line 16, in <module>
    from tensorflow.core.framework import tensor_shape_pb2 as tensorflow_dot_core_dot_framework_dot_tensor__shape__pb2
  File "/home/robot/developEye/Pupil-locator/envPupil/lib/python3.8/site-packages/tensorflow/core/framework/tensor_shape_pb2.py", line 36, in <module>
    _descriptor.FieldDescriptor(
  File "/home/robot/developEye/Pupil-locator/envPupil/lib/python3.8/site-packages/google/protobuf/descriptor.py", line 621, in __new__
    _message.Message._CheckCalledFromGeneratedFile()
TypeError: Descriptors cannot be created directly.
If this call came from a _pb2.py file, your generated code is out of date and must be regenerated with protoc >= 3.19.0.
If you cannot immediately regenerate your protos, some other possible workarounds are:
 1. Downgrade the protobuf package to 3.20.x or lower.
 2. Set PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python (but this will use pure-Python parsing and will be much slower).

More information: https://developers.google.com/protocol-buffers/docs/news/2022-05-06#python-updates
```

看到一个bot说tensorflow2.11，试试？
`pip install --upgrade -i https://pypi.org/simple/ tensorflow==2.11.1`  
完全没报错  

把输出改称realtime 效果很好！现在用的的是默认INC网络，不知道YOLO的咋样  

## 开始缝合

想法：把Eyetab的坐标算法和这个拼接一下就可以部署了  

### 接口

pycharm 快捷指令啥来着？
还有现在跑一遍太慢了  

接口是个ellipse  
坐标应该问题不大，朝向感觉会有点问题  
写了个geometry.py  
朝向没细看，先相信文心一言  

坐标转换算法测试，没问题，能跑  

### 滤波

卡尔曼滤波会考虑时间  
中值滤波应该还不错，而且非常的简单  
RANSAC是神魔  
还有之前比较直觉想的聚类，取点数最多的类的中心

- 那先试试分别取xyz的中位数  


### 环境配置
**GazeML**  
开一个虚拟环境  
pip install setup.py里的东西  
tensorflow版本  
下了2.13，那个pip好像自己找其它对应版本  
这好像tensorflow要1.x，只能装2.x  

跑Pytorch版  
这个一定要有CUDA，因为用了`cuda()`    
`nvcc --version`无  

**Eyestalker**  
https://github.com/tbrouns/eyestalker  
跑是跑了，但不是源码跑的，这要c++很多库  

**pupil-nystagmus**  
这个挺好的，就是有点小延迟  


**deepVOG**
这也要nvida  

**GazeHub**  
https://phi-ai.buaa.edu.cn/Gazehub/3D-method/  
先挑了个net  https://github.com/yihuacheng/Dilated-Net  


## 角度估计

不知道gazeML怎么估计的朝向等  
如果原版跑不了可以跑其它版  
https://github.com/J094/GazeML_torch   

- [x] 现在locator + nystagmus  
这两个都是圈虹膜，感觉远的照片根本看不出来瞳孔  
Eyetab用的也是虹膜大小，ok了  

> 人类平均虹膜直径12mm,中国人的平均虹膜直径略小,大约11.4mm
> 和虹膜比起来,瞳孔的大小几乎是随时在改变,范围可由1.2mm到9.6mm不等

- [x] nystagmus卡的问题
> 居然我换成locator的环境来跑就顺畅了，应该是opencv版本的问题，有毒啊

- [ ] nystagmus的版本numpy版本较旧  
> utils.py改了下float
它的cv2也旧，可能也有问题  

- [ ] nystagmus可能有w=0的值，或者长轴更短  
但是为啥会这样？    
> 加个异常检测

- [ ] myTest怎么变成圈瞳孔了?而且改完，inferno.py变差了效果  
> 需要理解一下RANSAC那段？理解不了，但应该没什么关系  
> 图像大小、channel问题？用一样的来跑，应该不是这个  
> rgb和灰色的问题？  

- [ ] 切片问题，roi那里
这比例也不是完全确定    

- realsense 打开默认是IR
查看官方使用手册：https://www.intelrealsense.com/developers/ 

- 将取面积>1000换成取面积最大，不知为何有报错，还每次不一样  
应该是有特殊情况，不是很鲁棒  
在getEllipse改了几行，应该ok

- 现在感觉不怎么鲁棒，为什么之前跑感觉又没啥问题

- locator远的时候，虹膜小，也识别不到

- [ ] 怎么传文件是个好问题
我mac ssh 被refused，10.1.1.122说是私有的ip，所以要在同一个局域网下才能访问？  
ssh不行，那ftp和filezila应该也同理不行吧  

## TODO
- [ ] 跑代码
  - [ ] 跑deepVOG  
  - [ ] 跑GazeHub
  - [ ] 跑GazeML
- [ ] 初版
  - [ ] locator + nystagmus
  - [ ] realsense导出数据
  - [x] 虹膜和瞳孔有点区别的，如果圈的是瞳孔，那要改一下6mm的预设