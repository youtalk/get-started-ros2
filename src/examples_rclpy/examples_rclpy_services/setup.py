from setuptools import setup

package_name = 'examples_rclpy_services'

setup(
    name=package_name,
    version='0.6.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Mikael Arguedas',
    author_email='mikael@osrfoundation.org',
    maintainer='Yutaka Kondo',
    maintainer_email='yutaka.kondo@youtalk.jp',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Examples of service servers/clients using rclpy.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'service = ' + package_name + '.service:main',
            'client = ' + package_name + '.client:main',
        ],
    },
)
