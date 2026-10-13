from setuptools import find_packages, setup

package_name = 'carla_adapter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    package_data={
        '': ['py.typed'],
    },
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='kaushik',
    maintainer_email='kaushik@todo.todo',
    description='CARLA to ROS 2 adapter for AI Traffic Middleware',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'carla_adapter = carla_adapter.carla_adapter_node:main',
        ],
    },
)
