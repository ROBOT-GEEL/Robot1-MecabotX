from setuptools import find_packages, setup

package_name = 'kul_nav2_tester'

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
    maintainer='wheeltec',
    maintainer_email='wout.welvaarts@gmail.com',
    description='Nav2 accuracy testing package',
    license='Apache-2.0',  # Changed to proper license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_nav2 = kul_nav2_tester.test_nav2:main',  # Added this line
        ],
    },
)
