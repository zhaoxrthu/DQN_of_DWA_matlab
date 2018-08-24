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

  
  

