from setuptools import find_packages, setup

package_name = 'detection_and_segmentation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kalash',
    maintainer_email='kalashjain513@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection_node = detection_and_segmentation.object_detection_node:main',
            'video_publisher = detection_and_segmentation.video_publisher:main',
        ],
    },
)
