from glob import glob

from setuptools import find_packages, setup

package_name = 'yolo_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/detector.launch.py']),
        # 模型权重是构建产物，不一定存在（.gitignore 忽略了 *.pt）。
        # 有就装，没有就跳过 —— detector_node 的 `_resolve_model()` 会依次查
        # $YOLO_MODEL_DIR、CWD 及各级父目录、包内 models/，全都找不到时给出
        # 明确报错，而不是静默联网下载 COCO 权重（D6/D24）。
        *([('share/' + package_name + '/models', glob('models/*.pt'))]
          if glob('models/*.pt') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='YOLOv8 object detection node for ROS2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detector_node = yolo_detector.detector_node:main',
        ],
    },
)
