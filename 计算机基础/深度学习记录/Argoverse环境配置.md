
1. ubuntu下，轨迹预测需要下载 

https://www.argoverse.org/  argoverse主界面 下载 
Map数据
Motion Forecasting数据 train、val、test-obs

argoverse-api from github
```
git clone https://github.com/argoai/argoverse-api.git

```
2.需要的环境 python3.6以上，（sudo apt-get 安装 不使用pip）numpy1.9，scikit-build0.10.0（低版本），cmake3.13.3(低版本)， python3.7-dev

3. CD argoverse-api 上一层目录
例如 ./MCTS-P2net/argoverse-api
cd /MCTS-P2net

```
pip install -e argoverse-api/
 
```

4.将下载好的数据放入 argoverse-api文件夹下，即可使用相对路径调用argoverse-api

5.使用import argoverse调用api


6.ROIALIGN

https://github.com/longcw/RoIAlign.pytorch

7.显卡算力对照表
 https://developer.nvidia.com/cuda-gpus
 
 watch \-n 1 nvidia\-smi

