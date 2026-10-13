from setuptools import find_packages, setup

package_name = 'autonomous_driver'

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
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='kaushik',
    maintainer_email='kaushik@todo.todo',
    description='Autonomous driving controller for AI Traffic Middleware',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'autonomous_driver = autonomous_driver.autonomous_driver_node:main',
        ],
    },
)
