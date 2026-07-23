# cam_navigation

基于深度相机的导航系统最小实现。

## 功能
- 根据深度图将视野分为左/中/右三个区域
- 当前方区域安全时前进
- 当前方受阻时向更安全一侧转向
- 三个区域都过近时停车

## 快速运行测试
```bash
python -m unittest discover -s tests -p 'test_*.py'
```
