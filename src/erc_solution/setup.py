import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'erc_solution'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'models'), glob('models/*.npz')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gloria',
    maintainer_email='gloriasasikiran@gmail.com',
    description='ERC 2026 Phase 1 solution - Library Assistant Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'book_retrieval = erc_solution.mission:main',
            'recording_view = erc_solution.recording_view:main',
        ],
    },
)
