from setuptools import setup

package_name = 'demo_package'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Eenvoudige ROS2 node',
    license='MIT',
    entry_points={
        'console_scripts': [
            'demo_node = demo_package.demo_node:main',
        ],
    },
)

