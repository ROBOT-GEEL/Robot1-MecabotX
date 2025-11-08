from setuptools import setup

package_name = 'no_go_zones'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='matthijs.mondelaers@student.kuleuven.be',
    description='no_go_zone',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Gebruik de correcte package folder naam
            'no_go_zones = no_go_zones.no_go_zones:main',
        ],
    },
)

