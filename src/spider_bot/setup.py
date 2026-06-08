from setuptools import setup
import os
from glob import glob

package_name = 'spider_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['models/model_1499.pt']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rasheed',
    maintainer_email='rasheedo.kit@gmail.com',
    description='spider bot package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'spider_bot_control = spider_bot.spider_bot_control:main',
        'test_node = spider_bot.test_node:main', # <--- 加上這行
        'run = spider_bot.run:main', # <--- 加上這行
        'standup = spider_bot.standup:main',
        'tank = spider_bot.tank:main',
        'imu_reader = spider_bot.imu_reader:main',
        'plot_thesis = spider_bot.plot_thesis:main',
        ],
    },
)
