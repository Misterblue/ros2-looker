from setuptools import setup
from setuptools import find_packages

package_name = 'ros2_adafruit_pwmhat_node'

setup(
    name            = package_name,
    description     = 'ROS2 Node for Adafruit Raspberry Pi 3 PWM 16-channel HAT',
    keywords        = [
        'ROS2'
    ],
    license         = 'Apache License, Version 2.0',
    version         = '0.1.0',
    classifiers     = [
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
        'Topic :: System :: Hardware',
    ],
    author          = 'Robert Adams',
    author_email    = 'misterblue@misterblue.com',
    maintainer      = 'Robert Adams',
    maintainer_email= 'misterblue@misterblue.com',

    packages        = [
        'ros2_adafruit_pwmhat_node'
    ],
    data_files      = [
       ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
       ('share/' + package_name, ['package.xml'])
    ],
    install_requires= [
        'setuptools',
        'Adafruit-GPIO>=0.6.5',
        'Adafruit_PCA9685'
    ],
    dependency_links= [
        'https://github.com/adafruit/Adafruit_Python_GPIO/tarball/master#egg=Adafruit-GPIO-0.6.5'
    ],

    entry_points    = {
        'console_scripts': [
            'service = ros2_adafruit_pwmhat_node.ros2_adafruit_pwmhat_node:main'
        ]
    },
    zip_safe=True,
    tests_require   = [
        'pytest'
    ]
)
