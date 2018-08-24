# DQN_of_DWA_matlab
learning the weight of each paras in DWA(Dynamic Window Approach) by using DQN(Deep Q-Learning)
## 1、关于本项目
  使用matlab平台及deeplearning工具包，利用DQN(Deep Q-Learning)对DWADynamic Window Approach)算法中各参数的权重进行学习。  
  
  
## 2、如何使用
  直接运行DQN.m即可。  
  
  
## 3、文件与参数说明
###   3.1 DQN.m
  主训练函数，负责调用Agent与Environment的各个模块；训练时直接运行该文件即可。  

###   3.2 Agent/
  
  对应RL（增强学习）中的Agent模块，完成学习网络的建立（新建或加载已有模型），DWA参数预测（前向传播）与更新。
  
  #### 3.2.1 AgentOutput.m
  通过cnn（卷积神经网络），由当前的状态得到此时各个选择的Q值，进而得到当前状态下的参数。    
  #### 3.2.2 AgentLearn.m
  由上一步得到的参数前进后环境反馈得到的reward，更新卷积神经网络，到达对卷及神经网络进行学习的目的。
  #### 3.2.3 NetSetUp.m & CreateNeuralNetneural.m
  卷积神经网络的建立与定义，参照了[DeepLearnToolbox](https://github.com/rasmusbergpalm/DeepLearnToolbox)并进行了部分修改。
  #### 3.2.4 ForwardPro.m & BackPro.m
  神经网络的前向传播与残差的后向传播，参照了[DeepLearnToolbox](https://github.com/rasmusbergpalm/DeepLearnToolbox)并进行了部分修改；可支持多通道、batch训练。  
  

### 3.3 Environment/
  对应RL（增强学习）中的Environment模块，完成对DWA算法的仿真。
  
  #### 3.3.1 Environtment.m
  接受DQN.m传递的参数，利用仿真环境得到小车的下一帧周围环境、本次前进的奖励、小车的状态（是否到达终点，是否绕圈、停止等）并返回。
  #### 3.3.2 LocalPathPlanning.m
  DWA主体部分，完成对各个路径的评价。
  #### 3.2.3 GoalRandGen.m
  随机在地图上生成起始点和终点，要求避免两者距离太小或有一在障碍物中的情况。
  #### 3.2.4 GetCurSurOb.m
  按照坐标得到小车周围的障碍物信息。  
 
### 3.4 Model/
  由DQN.m保存的当前模型参数信息，文件名为保存模型时的时间戳，可通过修改DQN.m中调用CreateNeuralNetneural.m的形式决定加载已有模型进行训练或者创建一个新模型。  
  
### 3.5 Ob.png
  仿真的障碍物信息，可根据需要修改。
  约定：白色（高像素值）部分为障碍物，黑色（低像素值）部分为可行区域。  
  
## 4、其他
  目前尚未得到较好的训练结果。  
  建议修改方向：神经网络的结构、奖励的形式与大小等。  
  联系方式【Email:<zhaoxrthu@gmail.com>】
  
  
  
  
  
  

