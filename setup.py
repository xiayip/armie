from glob import glob
from setuptools import setup

setup(
    name='armie', version='0.2.0', packages=['armie'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/armie']),
        ('share/armie', ['package.xml', 'README.md']),
        ('share/armie/config', glob('config/*.yaml')+glob('config/*.png')),
        ('share/armie/launch', glob('launch/*.launch.py')),
        ('share/armie/docs', glob('docs/*.md')),
        ('share/armie/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Armie developers', maintainer_email='dev@example.com',
    description='Rebot Arm desktop pet MVP', license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'game = armie.game:main', 'senses = armie.senses:main', 'panel = armie.panel:main',
        'grasp_preview = armie.grasp_preview:main',
        'table_scene = armie.table_scene:main',
        'controller_overrides = armie.controller_overrides:main',
        'tcp_calibration = armie.tcp_calibration:main',
        'pet = armie.pet:main', 'vision = armie.vision:main',
        'hmi = armie.hmi:main',
    ]},
)
