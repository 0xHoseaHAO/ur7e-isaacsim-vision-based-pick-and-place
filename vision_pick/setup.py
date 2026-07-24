from setuptools import find_packages, setup


package_name = 'vision_pick'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vision Pick',
    maintainer_email='maintainer@example.com',
    description='HSV colour segmentation and RGB-D object localization for Isaac Sim.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'segmentation_node = vision_pick.segmentation_node:main',
            'yolo_detection_node = vision_pick.yolo_detection_node:main',
            'pick_place_node = vision_pick.pick_place_node:main',
        ],
    },
)
